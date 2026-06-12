//! Pure Rust CPU fallback engine.
//!
//! Implements all 9 ISP block operations as software pixel processing,
//! with no dependency on ONNX Runtime or MNN.
//!
//! Pipeline:
//!   RawInput(INT16) → Normalize(FLOAT) → CFA(4ch) → BLC → WB
//!   → Demosaic(RGB) → CCM → Tone → Display(UINT8 BGRA)

use std::sync::Mutex;

use log::info;
use cam_types::ToneParams;

use crate::engine::{IspEngine, EngineFactory, register_engine};
use crate::pipeline::{IspBlock, IspFrame};
use crate::controller::IspController;

// ── Engine registration ──

pub struct CpuEngine {
    loaded: bool,
    _target_width: u32,
    /// ISP controller for AWB/AE/CCM/tone parameter estimation.
    pub controller: Mutex<IspController>,
}

impl CpuEngine {
    pub fn new() -> Self {
        Self { loaded: false, _target_width: 0, controller: Mutex::new(IspController::new()) }
    }
}

impl IspEngine for CpuEngine {
    fn backend_name(&self) -> &'static str { "CPU" }
    fn priority(&self) -> i32 { 70 }
    fn is_loaded(&self) -> bool { self.loaded }

    fn build(
        &mut self,
        _pipeline_head: Box<dyn IspBlock>,
        _aux_blocks: Vec<Box<dyn IspBlock>>,
        _warp_block: Option<Box<dyn IspBlock>>,
        _opset_version: i64,
    ) -> Result<(), String> {
        self.loaded = true;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn process(
        &self,
        width: u32,
        height: u32,
        _stride_width: u32,
        buf: &[u8],
        _sensor_max: f32,
        target_width: u32,
        ccm_matrix: Option<&[f32; 9]>,
        _tone_params: &ToneParams,
        bayer_gains: Option<&[f32; 4]>,
        awb_gains: Option<&[f32; 3]>,
        _analog_gain: f32,
        _scene_change: f32,
        _lsc_gains: Option<&[f32]>,
        blc_values: Option<&[f32; 4]>,
        _warp_grid: Option<&[f32]>,
    ) -> Result<IspFrame, String> {
        let t0 = std::time::Instant::now();

        info!("CpuEngine::process {}x{} → target {}", width, height, target_width);

        // ── 1. RawInput: interpret as INT16 Bayer ──
        // Input is raw 16-bit sensor data (Bayer pattern)
        let raw: Vec<u16> = if buf.len() >= (width * height * 2) as usize {
            buf.chunks_exact(2)
                .take((width * height) as usize)
                .map(|c| u16::from_le_bytes([c[0], c[1]]))
                .collect()
        } else {
            // Simulate raw Bayer data from RGBA test pattern
            generate_simulated_raw(width, height, buf)
        };

        // ── 2. Normalize: INT16 → FLOAT [0, 1] ──
        let max_val = if _sensor_max > 0.0 { _sensor_max } else { 65535.0 };
        let float: Vec<f32> = raw.iter().map(|&v| v as f32 / max_val).collect();

        // ── 2b. Defective pixel correction (hot pixel removal) ──
        let dpc_data = apply_dpc(&float, width as usize, height as usize, _lsc_gains.map(|g| g[0]).unwrap_or(0.15));

        // ── 2c. Gaussian denoise (optional, light blend) ──
        let denoised = apply_gaussian_denoise(&dpc_data, width as usize, height as usize, 0.3);

        // ── 3. Auto white balance (use controller if no external gains) ──
        let awb_gains = if let Some(g) = awb_gains { *g } else {
            let ctrl = self.controller.lock().unwrap();
            ctrl.get_awb_gains()
        };

        let default_bayer_gains = if bayer_gains.is_some() {
            *bayer_gains.unwrap()
        } else {
            [awb_gains[0], awb_gains[1], awb_gains[1], awb_gains[2]]
        };
        let wb_gains: &[f32; 4] = bayer_gains.unwrap_or(&default_bayer_gains);

        // ── 4. Apply BLC + WB per-pixel on raw CFA ──
        let blc = *blc_values.unwrap_or(&[0.0, 0.0, 0.0, 0.0]);
        let blc_wb = apply_blc_wb_raw(&denoised, width as usize, height as usize, &blc, wb_gains);

        // ── 4b. Lens shading correction (vignetting removal) ──
        let lsc_k = _lsc_gains.and_then(|g| g.first()).copied().unwrap_or(0.15);
        let corrected = apply_lsc(&blc_wb, width as usize, height as usize, lsc_k);

        // ── 5. Malvar demosaic: raw CFA → RGB ──
        let rgb = demosaic_malvar(&corrected, width as usize, height as usize, Some(&awb_gains));

        // ── 5b. Compute stats from RGB and update controller ──
        let channel_means = compute_channel_means(&rgb);
        let tone_stats = compute_tone_stats(&rgb);
        let histogram = compute_histogram(&rgb);
        let zone_stats = compute_zone_stats(&rgb, width as usize, height as usize, 6, 8);
        let ctrl_ccm = {
            let mut ctrl = self.controller.lock().unwrap();
            // Initialize zone stats on first use
            if !ctrl.zone_stats_enabled {
                ctrl.init_zone_stats(6, 8);
            }
            ctrl.update_channel_stats(&channel_means);
            ctrl.update_tone_stats(&tone_stats);
            ctrl.update_histogram(&histogram);
            // Feed zone stats
            ctrl.update_zone_stats(&zone_stats);
            // Compute exposure time + ISO from scene stats
            let bias = ctrl.brightness_bias;
            let (_exp_ns, _iso) = ctrl.compute_exposure(bias);
            info!("Ctrl AWB gains: {:.3} {:.3} {:.3}, CCT: {:?}, AE gain: {:.3}",
                ctrl.awb_gains[0], ctrl.awb_gains[1], ctrl.awb_gains[2],
                ctrl.estimated_cct, ctrl.get_effective_exposure_gain());
            ctrl.get_ccm()
        };

        // ── 6. Auto exposure (use controller or fallback simple AE) ──
        let ae_gain = if _analog_gain <= 0.0 {
            self.controller.lock().unwrap().get_effective_exposure_gain()
        } else {
            calculate_ae_gain(&rgb)
        };

        // ── 7. CCM: 3×3 color matrix (use controller if no external) ──
        let ccm: &[f32; 9] = ccm_matrix.unwrap_or(&ctrl_ccm);
        let ccm_applied = apply_ccm(&rgb, ccm);

        // ── 8. Tone: apply tone curve (with AE gain) ──
        let adjusted = apply_ae_gain(&ccm_applied, ae_gain);
        let toned = apply_tone(&adjusted, _tone_params, width as usize, height as usize);

        // ── 9. Display: resize + convert to UINT8 BGRA ──
        let out_width = if target_width > 0 { target_width } else { width };
        let out_bytes = display_output(&toned, width as usize, height as usize, out_width as usize);

        let elapsed = t0.elapsed();
        info!("CpuEngine: processed in {:?} → {}×{} output ({} bytes)",
            elapsed, out_width, height, out_bytes.len());

        Ok(IspFrame {
            width: out_width,
            height,
            format: cam_types::FrameFormat::Rgba8888,
            data: out_bytes,
            float_data: None,
            aux: None,
        })
    }
}

// ── Pixel processing functions ──

/// Generate simulated raw Bayer data from RGBA test pattern.
fn generate_simulated_raw(width: u32, height: u32, rgba: &[u8]) -> Vec<u16> {
    let mut raw = Vec::with_capacity((width * height) as usize);
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize * 4;
            let r = if idx + 3 < rgba.len() { rgba[idx] as u32 } else { 128 };
            let g = if idx + 2 < rgba.len() { rgba[idx + 1] as u32 } else { 128 };
            let b = if idx + 3 < rgba.len() { rgba[idx + 2] as u32 } else { 128 };

            // BGGR pattern
            let raw_val = if y % 2 == 0 {
                if x % 2 == 0 { b } else { g } // B at (0,0), G at (1,0)
            } else {
                if x % 2 == 0 { g } else { r } // G at (0,1), R at (1,1)
            };
            raw.push((raw_val * 257) as u16); // stretch 8bit → ~16bit
        }
    }
    raw
}

/// Calculate auto white balance gains using gray world assumption on raw CFA.
/// Works on the normalized float CFA (BGGR pattern).
/// Returns [R_gain, Gr_gain, Gb_gain, B_gain].
#[allow(dead_code)]
fn calculate_awb_gains(cfa: &[f32], width: usize, height: usize) -> [f32; 4] {
    let mut sum_r = 0.0f32;
    let mut sum_gr = 0.0f32;
    let mut sum_gb = 0.0f32;
    let mut sum_b = 0.0f32;
    let mut count_r = 0u32;
    let mut count_gr = 0u32;
    let mut count_gb = 0u32;
    let mut count_b = 0u32;

    for y in 0..height {
        for x in 0..width {
            let v = cfa[y * width + x];
            if y % 2 == 0 {
                if x % 2 == 0 { sum_b += v; count_b += 1; }
                else { sum_gb += v; count_gb += 1; }
            } else {
                if x % 2 == 0 { sum_gr += v; count_gr += 1; }
                else { sum_r += v; count_r += 1; }
            }
        }
    }

    let avg_r = sum_r / count_r as f32;
    let avg_gr = sum_gr / count_gr as f32;
    let avg_gb = sum_gb / count_gb as f32;
    let avg_b = sum_b / count_b as f32;

    // Average of all green
    let avg_g = (avg_gr + avg_gb) * 0.5;

    // Gains to make each channel equal to green
    // Clamp to prevent division by zero or extreme gains
    let eps = 0.001;
    let gain_r = if avg_r > eps { avg_g / avg_r } else { 1.0 };
    let gain_gr = if avg_gr > eps { avg_g / avg_gr } else { 1.0 };
    let gain_gb = if avg_gb > eps { avg_g / avg_gb } else { 1.0 };
    let gain_b = if avg_b > eps { avg_g / avg_b } else { 1.0 };

    // Limit to reasonable range
    [
        gain_r.min(4.0).max(0.25),
        gain_gr.min(4.0).max(0.25),
        gain_gb.min(4.0).max(0.25),
        gain_b.min(4.0).max(0.25),
    ]
}

/// Calculate auto exposure gain to bring scene to target brightness (18% gray = 0.18).
fn calculate_ae_gain(rgb: &[f32]) -> f32 {
    let count = rgb.len() / 3;
    if count == 0 { return 1.0; }

    // Compute average luminance
    let mut total_luma = 0.0f32;
    for i in 0..count {
        let r = rgb[i * 3];
        let g = rgb[i * 3 + 1];
        let b = rgb[i * 3 + 2];
        total_luma += 0.299 * r + 0.587 * g + 0.114 * b;
    }
    let avg_luma = total_luma / count as f32;

    let target_luma = 0.18;
    if avg_luma > 0.001 {
        (target_luma / avg_luma).min(8.0).max(0.125)
    } else {
        1.0
    }
}

/// Apply auto exposure gain to RGB data.
fn apply_ae_gain(rgb: &[f32], gain: f32) -> Vec<f32> {
    rgb.iter().map(|&v| (v * gain).max(0.0).min(1.0)).collect()
}

/// Apply black level correction and white balance on raw CFA data.
/// `blc` = [R_bias, Gr_bias, Gb_bias, B_bias]  (subtract this)
/// `gains` = [R_gain, Gr_gain, Gb_gain, B_gain]  (multiply by this)
/// BGGR pattern:
///   even row:    B Gb B Gb ...
///   odd row:     Gr R Gr R ...
fn apply_blc_wb_raw(float: &[f32], width: usize, height: usize, blc: &[f32; 4], gains: &[f32; 4]) -> Vec<f32> {
    let mut out = Vec::with_capacity(float.len());
    for y in 0..height {
        for x in 0..width {
            let idx = y * width + x;
            let raw_val = float[idx];
            let (bias, gain) = if y % 2 == 0 {
                if x % 2 == 0 { (blc[3], gains[3]) }   // B pixel
                else { (blc[2], gains[2]) }              // Gb pixel
            } else {
                if x % 2 == 0 { (blc[1], gains[1]) }    // Gr pixel
                else { (blc[0], gains[0]) }               // R pixel
            };
            let corrected = (raw_val - bias).max(0.0) * gain;
            out.push(corrected.max(0.0).min(1.0));
        }
    }
    out
}

/// Defective pixel correction — detect hot pixels by 3×3 median deviation.
/// Any pixel deviating more than `threshold` from the median of its neighborhood
/// is replaced with that median. Operates on raw CFA data.
fn apply_dpc(cfa: &[f32], w: usize, h: usize, threshold: f32) -> Vec<f32> {
    if w < 3 || h < 3 {
        return cfa.to_vec();
    }
    let mut out = cfa.to_vec();
    for y in 1..h-1 {
        for x in 1..w-1 {
            let idx = y * w + x;
            let center = cfa[idx];

            // Collect 3×3 neighborhood
            let mut neighbors: [f32; 8] = [0.0; 8];
            let mut ni = 0;
            for dy in 0..3 {
                for dx in 0..3 {
                    if dy == 1 && dx == 1 { continue; }
                    let ny = y + dy - 1;
                    let nx = x + dx - 1;
                    neighbors[ni] = cfa[ny * w + nx];
                    ni += 1;
                }
            }
            // Median of 8 neighbors (average of 4th and 5th in sorted order)
            neighbors.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let median = (neighbors[3] + neighbors[4]) * 0.5;

            // If center deviates too much, replace
            let diff = (center - median).abs();
            if diff > threshold {
                out[idx] = median;
            }
        }
    }
    out
}

/// Lens shading correction (vignetting removal).
/// Applies radial gain: center pixels unchanged, corners boosted.
/// `k` controls strength (0 = off, 0.3 = moderate, 0.6 = strong).
/// Gain = 1 + k * r^2 where r is normalized [0,1] from image center.
fn apply_lsc(raw: &[f32], w: usize, h: usize, k: f32) -> Vec<f32> {
    if k <= 0.0 { return raw.to_vec(); }
    let cx = w as f32 / 2.0;
    let cy = h as f32 / 2.0;
    let max_r2 = (cx * cx + cy * cy) as f32;
    let mut out = Vec::with_capacity(raw.len());
    for y in 0..h {
        for x in 0..w {
            let dx = x as f32 - cx;
            let dy = y as f32 - cy;
            let r2 = (dx * dx + dy * dy) / max_r2;
            let gain = 1.0 + k * r2;
            out.push((raw[y * w + x] * gain).min(1.0));
        }
    }
    out
}

/// Malvar (2004) demosaic — high-quality gradient-based interpolation.
/// BGGR pattern expected.
/// Reference: Malvar, He, Cutler "High-Quality Linear Interpolation for Demosaicing of Bayer-Patterned Color Images" ICASSP 2004.
fn demosaic_malvar(cfa: &[f32], width: usize, height: usize, _awb: Option<&[f32; 3]>) -> Vec<f32> {
    let mut rgb = vec![0.0f32; width * height * 3];

    for y in 0..height {
        for x in 0..width {
            let idx = y * width + x;
            let is_even_row = y % 2 == 0;
            let is_even_col = x % 2 == 0;

            match (is_even_row, is_even_col) {
                // ── Blue pixel at (even, even) and Red at (odd, odd) ──
                (true, true) | (false, false) => {
                    // B at (even, even); R at (odd, odd)
                    // We'll compute green and red (or blue) for this location
                    let is_blue = is_even_row; // true → B, false → R

                    // Sample the CFA value at this location
                    let cf = cfa[idx];

                    // ── Green interpolation ──
                    // Use 5x5 directional interpolation with gradient detection
                    let g = interpolate_green_at_rb(cfa, width, height, x, y, is_blue);

                    // ── Red (or Blue) interpolation ──
                    // At B location: interpolate R using color difference across G
                    let other = if is_blue {
                        interpolate_red_at_blue(cfa, width, height, x, y, g)
                    } else {
                        interpolate_blue_at_red(cfa, width, height, x, y, g)
                    };

                    if is_blue {
                        // B location: B is known, R and G are interpolated
                        set_rgb(&mut rgb, x, y, width, other, g, cf);
                    } else {
                        // R location: R is known, G and B are interpolated
                        set_rgb(&mut rgb, x, y, width, cf, g, other);
                    }
                }
                // ── Green pixel at (even, odd) or (odd, even) ──
                (true, false) | (false, true) => {
                    // Gb or Gr
                    let g_val = cfa[idx];
                    // Interpolate R and B using color differences
                    let r = interpolate_red_at_green(cfa, width, height, x, y);
                    let b = interpolate_blue_at_green(cfa, width, height, x, y);
                    set_rgb(&mut rgb, x, y, width, r, g_val, b);
                }
            }
        }
    }
    rgb
}

/// Interpolate green at a red or blue location using gradient detection.
fn interpolate_green_at_rb(cfa: &[f32], width: usize, height: usize, x: usize, y: usize, _is_blue: bool) -> f32 {
    // Horizontal and vertical gradients
    let hv_grad = |x: usize, y: usize| -> (f32, f32) {
        let c = |dx: isize, dy: isize| -> f32 {
            let nx = (x as isize + dx).clamp(0, width as isize - 1) as usize;
            let ny = (y as isize + dy).clamp(0, height as isize - 1) as usize;
            cfa[ny * width + nx]
        };

        // Horizontal gradient: |G(x-1,y) - G(x+1,y)| + |2*C(x,y) - C(x-2,y) - C(x+2,y)|
        let gh = (c(-1, 0) - c(1, 0)).abs() + (2.0 * c(0, 0) - c(-2, 0) - c(2, 0)).abs();
        // Vertical gradient: |G(x,y-1) - G(x,y+1)| + |2*C(x,y) - C(x,y-2) - C(x,y+2)|
        let gv = (c(0, -1) - c(0, 1)).abs() + (2.0 * c(0, 0) - c(0, -2) - c(0, 2)).abs();
        (gh, gv)
    };

    let (gh, gv) = hv_grad(x, y);

    let c = |dx: isize, dy: isize| -> f32 {
        let nx = (x as isize + dx).clamp(0, width as isize - 1) as usize;
        let ny = (y as isize + dy).clamp(0, height as isize - 1) as usize;
        cfa[ny * width + nx]
    };

    // Horizontal interpolation
    let g_h = (c(-1, 0) + c(1, 0)) / 2.0 + (2.0 * c(0, 0) - c(-2, 0) - c(2, 0)) / 4.0;
    // Vertical interpolation
    let g_v = (c(0, -1) + c(0, 1)) / 2.0 + (2.0 * c(0, 0) - c(0, -2) - c(0, 2)) / 4.0;

    // Weighted combination based on gradients
    if gh + gv < 1e-6 { (g_h + g_v) / 2.0 } else { (g_v * gh + g_h * gv) / (gh + gv) }
}

/// Interpolate R at a B location using color difference with G.
fn interpolate_red_at_blue(cfa: &[f32], width: usize, height: usize, x: usize, y: usize, g: f32) -> f32 {
    let c = |dx: isize, dy: isize| -> f32 {
        let nx = (x as isize + dx).clamp(0, width as isize - 1) as usize;
        let ny = (y as isize + dy).clamp(0, height as isize - 1) as usize;
        cfa[ny * width + nx]
    };

    // R at B: average of (R - G) at 4 diagonal R positions, then add G
    // G at those diagonal positions is the average of 4 G neighbors
    let g_at_diag = |dx: isize, dy: isize| -> f32 {
        let nx = (x as isize + dx).clamp(0, width as isize - 1) as usize;
        let ny = (y as isize + dy).clamp(0, height as isize - 1) as usize;
        let g_h = (cfa[ny * width + nx.saturating_sub(1)] + cfa[ny * width + (nx + 1).min(width - 1)]) * 0.5;
        let g_v = (cfa[ny.saturating_sub(1) * width + nx] + cfa[(ny + 1).min(height - 1) * width + nx]) * 0.5;
        (g_h + g_v) * 0.5
    };

    let diff1 = c(-1, -1) - g_at_diag(-1, -1);
    let diff2 = c(1, -1) - g_at_diag(1, -1);
    let diff3 = c(-1, 1) - g_at_diag(-1, 1);
    let diff4 = c(1, 1) - g_at_diag(1, 1);

    let avg_diff = (diff1 + diff2 + diff3 + diff4) / 4.0;
    (g + avg_diff).max(0.0).min(1.0)
}

/// Interpolate B at an R location (same as above, swapping R/B).
fn interpolate_blue_at_red(cfa: &[f32], width: usize, height: usize, x: usize, y: usize, g: f32) -> f32 {
    interpolate_red_at_blue(cfa, width, height, x, y, g)
}

/// Interpolate R at a green location.
fn interpolate_red_at_green(cfa: &[f32], width: usize, height: usize, x: usize, y: usize) -> f32 {
    let c = |dx: isize, dy: isize| -> f32 {
        let nx = (x as isize + dx).clamp(0, width as isize - 1) as usize;
        let ny = (y as isize + dy).clamp(0, height as isize - 1) as usize;
        cfa[ny * width + nx]
    };
    // At Gr (even row, odd col): R is at (odd, odd) = (y+1, x) and (y-1, x)
    // At Gb (odd row, even col): R is at (odd, odd) = (y, x+1) and (y, x-1)
    // Just use diagonal R-G differences
    let r1 = c(-1, -1); let r2 = c(1, -1);
    let r3 = c(-1, 1); let r4 = c(1, 1);
    let g1 = c(-1, 0); let g2 = c(1, 0);
    let g3 = c(0, -1); let g4 = c(0, 1);
    // R - G from vertical/horizontal neighbors
    let diff1 = r1 - (g3 + g1) / 2.0;
    let diff2 = r2 - (g3 + g2) / 2.0;
    let diff3 = r3 - (g4 + g1) / 2.0;
    let diff4 = r4 - (g4 + g2) / 2.0;
    let avg_diff = (diff1 + diff2 + diff3 + diff4) / 4.0;
    cfa[y * width + x] + avg_diff
}

/// Interpolate B at a green location (same as red interpolation, swapping R/B pattern).
fn interpolate_blue_at_green(cfa: &[f32], width: usize, height: usize, x: usize, y: usize) -> f32 {
    interpolate_red_at_green(cfa, width, height, x, y)
}

/// Set a pixel's RGB values in the output buffer.
fn set_rgb(rgb: &mut [f32], x: usize, y: usize, width: usize, r: f32, g: f32, b: f32) {
    let idx = (y * width + x) * 3;
    rgb[idx] = r.max(0.0).min(1.0);
    rgb[idx + 1] = g.max(0.0).min(1.0);
    rgb[idx + 2] = b.max(0.0).min(1.0);
}

/// Apply 3×3 color correction matrix to each pixel.
fn apply_ccm(rgb: &[f32], matrix: &[f32; 9]) -> Vec<f32> {
    let count = rgb.len() / 3;
    let mut corrected = Vec::with_capacity(rgb.len());

    for i in 0..count {
        let idx = i * 3;
        let r = rgb[idx];
        let g = rgb[idx + 1];
        let b = rgb[idx + 2];

        let out_r = matrix[0]*r + matrix[1]*g + matrix[2]*b;
        let out_g = matrix[3]*r + matrix[4]*g + matrix[5]*b;
        let out_b = matrix[6]*r + matrix[7]*g + matrix[8]*b;

        corrected.push(out_r.max(0.0).min(1.0));
        corrected.push(out_g.max(0.0).min(1.0));
        corrected.push(out_b.max(0.0).min(1.0));
    }
    corrected
}

/// Apply tone curve: gamma + contrast + brightness + saturation + sharpen.
/// Apply tone curve: gamma + contrast + brightness + saturation + unsharp mask.
fn apply_tone(rgb: &[f32], params: &ToneParams, w: usize, h: usize) -> Vec<f32> {
    let gamma_recip = params.gamma_recip;
    let gamma = if gamma_recip > 0.0 { 1.0 / gamma_recip } else { 1.0 };
    let contrast = params.contrast;
    let brightness = params.brightness;
    let saturation = params.saturation;

    // For simplicity, apply basic gamma + contrast
    let count = rgb.len() / 3;
    let mut toned = Vec::with_capacity(rgb.len());

    for i in 0..count {
        let idx = i * 3;
        let mut r = rgb[idx];
        let mut g = rgb[idx + 1];
        let mut b = rgb[idx + 2];

        // Apply gamma
        r = r.powf(gamma);
        g = g.powf(gamma);
        b = b.powf(gamma);

        // Apply contrast
        r = ((r - 0.5) * contrast) + 0.5;
        g = ((g - 0.5) * contrast) + 0.5;
        b = ((b - 0.5) * contrast) + 0.5;

        // Apply brightness
        r += brightness;
        g += brightness;
        b += brightness;

        // Apply saturation (simple)
        let luma = 0.299*r + 0.587*g + 0.114*b;
        r = luma + (r - luma) * saturation;
        g = luma + (g - luma) * saturation;
        b = luma + (b - luma) * saturation;

        toned.push(r.max(0.0).min(1.0));
        toned.push(g.max(0.0).min(1.0));
        toned.push(b.max(0.0).min(1.0));
    }

    // Unsharp mask (3×3 separable, strength proportional to sharpness param)
    if params.sharpness > 0.0 && w >= 3 && h >= 3 {
        let strength = params.sharpness * 0.15;
        apply_unsharp_mask(&mut toned, w, h, strength);
    }
    toned
}

/// Apply unsharp mask to RGB buffer for edge enhancement.
/// Uses a separable 3×3 Laplacian kernel blended with the original.
fn apply_unsharp_mask(rgb: &mut [f32], w: usize, h: usize, strength: f32) {
    let original = rgb.to_vec();
    let get = |x: usize, y: usize, ch: usize| -> f32 {
        original[(y * w + x) * 3 + ch]
    };

    for y in 1..h-1 {
        for x in 1..w-1 {
            for ch in 0..3 {
                let center = original[(y * w + x) * 3 + ch];
                let laplacian =
                    -get(x-1, y, ch) - get(x+1, y, ch)
                    -get(x, y-1, ch) - get(x, y+1, ch)
                    + 4.0 * center;

                // Add detail: sharpened = original + strength * detail
                let detail = center - laplacian; // center - lowpass = highpass
                let val = center + strength * detail;
                rgb[(y * w + x) * 3 + ch] = val.max(0.0).min(1.0);
            }
        }
    }
}

/// Compute channel means (R, G, B average) from an RGB float buffer.
fn compute_channel_means(rgb: &[f32]) -> [f32; 3] {
    let n = rgb.len() / 3;
    if n == 0 { return [0.0; 3]; }
    let mut sum = [0.0; 3];
    for ch in 0..3 {
        for i in 0..n {
            sum[ch] += rgb[i * 3 + ch];
        }
        sum[ch] /= n as f32;
    }
    sum
}

/// Compute tone statistics (mean/min/max luminance) from an RGB float buffer.
fn compute_tone_stats(rgb: &[f32]) -> [f32; 3] {
    let n = rgb.len() / 3;
    if n == 0 { return [0.0, 0.0, 1.0]; }
    let mut mean_lum = 0.0f32;
    let mut min_lum = f32::MAX;
    let mut max_lum = f32::MIN;
    for i in 0..n {
        let r = rgb[i * 3];
        let g = rgb[i * 3 + 1];
        let b = rgb[i * 3 + 2];
        let lum = 0.299 * r + 0.587 * g + 0.114 * b;
        mean_lum += lum;
        if lum < min_lum { min_lum = lum; }
        if lum > max_lum { max_lum = lum; }
    }
    mean_lum /= n as f32;
    [mean_lum, min_lum, max_lum]
}

/// Compute 256-bin luminance histogram from an RGB float buffer.
/// Uses standard BT.601 luma weights.
fn compute_histogram(rgb: &[f32]) -> [f32; 256] {
    let mut hist = [0.0f32; 256];
    let n = rgb.len() / 3;
    if n == 0 { return hist; }
    for i in 0..n {
        let r = rgb[i * 3];
        let g = rgb[i * 3 + 1];
        let b = rgb[i * 3 + 2];
        let lum = 0.299 * r + 0.587 * g + 0.114 * b;
        let bin = (lum * 255.0).round() as usize;
        let bin = bin.min(255);
        hist[bin] += 1.0;
    }
    // Normalize to sum = 1.0
    let total: f32 = hist.iter().sum();
    if total > 0.0 {
        for v in hist.iter_mut() { *v /= total; }
    }
    hist
}

/// Compute zone stats (per-zone RGB means) for multi-illuminant AWB.
/// Divides the image into `rows × cols` zones and computes the average
/// R, G, B for each zone.
/// Returns flat array: [zone0_r, zone0_g, zone0_b, zone1_r, ...] (row-major).
fn compute_zone_stats(rgb: &[f32], width: usize, height: usize, rows: usize, cols: usize) -> Vec<f32> {
    let n = rgb.len() / 3;
    if n == 0 || rows == 0 || cols == 0 {
        return vec![];
    }

    let zone_w = (width / cols).max(1);
    let zone_h = (height / rows).max(1);

    let mut sums = vec![[0.0f64; 3]; rows * cols];
    let mut counts = vec![0u64; rows * cols];

    for y in 0..height {
        for x in 0..width {
            let zi = y / zone_h;
            let zj = x / zone_w;
            if zi >= rows || zj >= cols { continue; }
            let idx = (y * width + x) * 3;
            let zidx = zi * cols + zj;
            sums[zidx][0] += rgb[idx] as f64;
            sums[zidx][1] += rgb[idx + 1] as f64;
            sums[zidx][2] += rgb[idx + 2] as f64;
            counts[zidx] += 1;
        }
    }

    let mut result = Vec::with_capacity(rows * cols * 3);
    for zidx in 0..(rows * cols) {
        let c = counts[zidx].max(1);
        result.push((sums[zidx][0] / c as f64) as f32);
        result.push((sums[zidx][1] / c as f64) as f32);
        result.push((sums[zidx][2] / c as f64) as f32);
    }
    result
}

/// Apply 3×3 Gaussian blur for denoising (raw Bayer domain).
/// Kernel: [1 2 1; 2 4 2; 1 2 1] / 16
/// Only applied when `strength > 0.0`.
fn apply_gaussian_denoise(raw: &[f32], w: usize, h: usize, strength: f32) -> Vec<f32> {
    if strength <= 0.0 || w < 3 || h < 3 {
        return raw.to_vec();
    }
    let k = if strength >= 1.0 { 1.0 } else { strength }; // blend factor
    let mut out = raw.to_vec();
    for y in 1..h-1 {
        for x in 1..w-1 {
            let idx = y * w + x;
            let val = (
                raw[(y-1)*w + (x-1)] * 1.0 + raw[(y-1)*w + x] * 2.0 + raw[(y-1)*w + (x+1)] * 1.0 +
                raw[y*w + (x-1)]     * 2.0 + raw[y*w + x]     * 4.0 + raw[y*w + (x+1)]     * 2.0 +
                raw[(y+1)*w + (x-1)] * 1.0 + raw[(y+1)*w + x] * 2.0 + raw[(y+1)*w + (x+1)] * 1.0
            ) / 16.0;
            out[idx] = raw[idx] + (val - raw[idx]) * k;
        }
    }
    out
}

/// Display output: resize to target width and convert to UINT8 BGRA.
fn display_output(rgb: &[f32], src_w: usize, src_h: usize, target_w: usize) -> Vec<u8> {
    let target_h = (src_h as f32 * target_w as f32 / src_w as f32).round() as usize;
    let mut out = vec![0u8; target_w * target_h * 4];

    for y in 0..target_h {
        for x in 0..target_w {
            // Nearest-neighbor resize
            let sx = (x as f32 * src_w as f32 / target_w as f32) as usize;
            let sy = (y as f32 * src_h as f32 / target_h as f32) as usize;

            let src_idx = (sy * src_w + sx) * 3;
            let r = (rgb[src_idx].max(0.0).min(1.0) * 255.0) as u8;
            let g = (rgb[src_idx + 1].max(0.0).min(1.0) * 255.0) as u8;
            let b = (rgb[src_idx + 2].max(0.0).min(1.0) * 255.0) as u8;

            let dst_idx = (y * target_w + x) * 4;
            // RGBA output
            out[dst_idx] = r;     // R
            out[dst_idx + 1] = g; // G
            out[dst_idx + 2] = b; // B
            out[dst_idx + 3] = 255; // A
        }
    }
    out
}

/// Register the CPU engine into the global registry.
pub fn register_cpu_engine() {
    let factory = EngineFactory {
        name: "CPU",
        priority: 70,
        create_fn: || Box::new(CpuEngine::new()) as Box<dyn IspEngine>,
    };
    register_engine(factory);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::IspEngine;

    #[test]
    fn test_cpu_engine_process() {
        let mut engine = CpuEngine::new();
        assert!(engine.build(Box::new(crate::blocks::RawInputBlock::new()), vec![], None, 21).is_ok());

        // Create a small 16x16 simulated Bayer RAW (INT16 data)
        let w = 16u32;
        let h = 16u32;
        let mut raw_buf = Vec::new();
        for y in 0..h {
            for x in 0..w {
                // BGGR pattern: simulate 12-bit sensor values
                let val: u16 = if y % 2 == 0 {
                    if x % 2 == 0 { 2000 } else { 4000 } // B, Gb
                } else {
                    if x % 2 == 0 { 4000 } else { 6000 } // Gr, R
                };
                raw_buf.extend_from_slice(&val.to_le_bytes());
            }
        }

        let params = cam_types::ToneParams::default();
        let result = engine.process(
            w, h, w, &raw_buf, 65535.0, w,
            None, &params, None, None, 1.0, 0.0, None, None, None,
        );
        assert!(result.is_ok());
        let frame = result.unwrap();
        assert_eq!(frame.width, w);
        assert_eq!(frame.height, h);
        assert!(!frame.data.is_empty());
        // First pixel's red channel should be > 0
        assert!(frame.data[0] > 0);
    }
}
