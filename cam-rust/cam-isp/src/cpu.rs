//! Pure Rust CPU fallback engine.
//!
//! Implements all 9 ISP block operations as software pixel processing,
//! with no dependency on ONNX Runtime or MNN.
//!
//! Pipeline:
//!   RawInput(INT16) → Normalize(FLOAT) → CFA(4ch) → BLC → WB
//!   → Demosaic(RGB) → CCM → Tone → Display(UINT8 BGRA)

use log::info;
use cam_types::ToneParams;

use crate::engine::{IspEngine, EngineFactory, register_engine};
use crate::pipeline::{IspBlock, IspFrame};

// ── Engine registration ──

pub struct CpuEngine {
    loaded: bool,
    _target_width: u32,
}

impl CpuEngine {
    pub fn new() -> Self {
        Self { loaded: false, _target_width: 0 }
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
        tone_params: &ToneParams,
        bayer_gains: Option<&[f32; 4]>,
        _awb_gains: Option<&[f32; 3]>,
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
        let max_val = 65535.0f32;
        let float: Vec<f32> = raw.iter().map(|&v| v as f32 / max_val).collect();

        // ── 3. CFA: extract 4 Bayer planes ──
        // Assume BGGR pattern
        let bayer = extract_bayer(&float, width as usize, height as usize);

        // ── 4. BLC: subtract black level ──
        let blc = if let Some(blc_vals) = blc_values {
            apply_blc(&bayer, blc_vals)
        } else {
            bayer.clone()
        };

        // ── 5. WB: apply white balance gains ──
        let wb_gains = bayer_gains.unwrap_or(&[1.0, 1.0, 1.0, 1.0]);
        let wb = apply_wb(&blc, wb_gains);

        // ── 6. Demosaic: bilinear → RGB ──
        let rgb = demosaic_bilinear(&wb, width as usize, height as usize);

        // ── 7. CCM: 3×3 color matrix ──
        let ccm = ccm_matrix.unwrap_or(&[
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]);
        let corrected = apply_ccm(&rgb, ccm);

        // ── 8. Tone: apply tone curve ──
        let toned = apply_tone(&corrected, tone_params);

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

/// Extract 4 Bayer channels: [R, Gr, Gb, B] as separate planes.
fn extract_bayer(float: &[f32], width: usize, height: usize) -> [Vec<f32>; 4] {
    let mut r_plane = Vec::with_capacity((width * height) / 4);
    let mut gr_plane = Vec::with_capacity((width * height) / 4);
    let mut gb_plane = Vec::with_capacity((width * height) / 4);
    let mut b_plane = Vec::with_capacity((width * height) / 4);

    for y in 0..height {
        for x in 0..width {
            let idx = y * width + x;
            let v = float[idx];
            // BGGR pattern
            if y % 2 == 0 {
                if x % 2 == 0 { b_plane.push(v); } else { gb_plane.push(v); }
            } else {
                if x % 2 == 0 { gr_plane.push(v); } else { r_plane.push(v); }
            }
        }
    }
    [r_plane, gr_plane, gb_plane, b_plane]
}

/// Apply black level correction: subtract blc values from each channel.
fn apply_blc(planes: &[Vec<f32>; 4], blc: &[f32; 4]) -> [Vec<f32>; 4] {
    let mut out: [Vec<f32>; 4] = [
        Vec::with_capacity(planes[0].len()),
        Vec::with_capacity(planes[1].len()),
        Vec::with_capacity(planes[2].len()),
        Vec::with_capacity(planes[3].len()),
    ];
    for (i, plane) in planes.iter().enumerate() {
        for &v in plane.iter() {
            out[i].push((v - blc[i]).max(0.0));
        }
    }
    out
}

/// Apply white balance gains.
fn apply_wb(planes: &[Vec<f32>; 4], gains: &[f32; 4]) -> [Vec<f32>; 4] {
    let mut out: [Vec<f32>; 4] = [
        Vec::with_capacity(planes[0].len()),
        Vec::with_capacity(planes[1].len()),
        Vec::with_capacity(planes[2].len()),
        Vec::with_capacity(planes[3].len()),
    ];
    for (i, plane) in planes.iter().enumerate() {
        let g = gains[i];
        for &v in plane.iter() {
            out[i].push((v * g).max(0.0).min(1.0));
        }
    }
    out
}

/// Bilinear demosaic: reconstruct full RGB from 4 Bayer planes.
fn demosaic_bilinear(planes: &[Vec<f32>; 4], width: usize, height: usize) -> Vec<f32> {
    let grid_w = width / 2;
    let grid_h = height / 2;

    // Convert planes back to grid for interpolation
    let r_grid = mat_from_plane(&planes[0], grid_w, grid_h);
    let gr_grid = mat_from_plane(&planes[1], grid_w, grid_h);
    let gb_grid = mat_from_plane(&planes[2], grid_w, grid_h);
    let b_grid = mat_from_plane(&planes[3], grid_w, grid_h);

    // G grid: average of Gr and Gb
    let g_grid: Vec<Vec<f32>> = (0..grid_h).map(|y| {
        (0..grid_w).map(|x| {
            let gr = gr_grid[y][x];
            let gb = gb_grid[y][x];
            (gr + gb) * 0.5
        }).collect()
    }).collect();

    let mut rgb = vec![0.0f32; width * height * 3];

    for y in 0..height {
        for x in 0..width {
            let gx = x / 2;
            let gy = y / 2;
            let r = bilerp(&r_grid, gx as f32, gy as f32);
            let g = bilerp(&g_grid, gx as f32, gy as f32);
            let b = bilerp(&b_grid, gx as f32, gy as f32);
            let idx = (y * width + x) * 3;
            rgb[idx] = r;
            rgb[idx + 1] = g;
            rgb[idx + 2] = b;
        }
    }
    rgb
}

fn mat_from_plane(plane: &[f32], w: usize, h: usize) -> Vec<Vec<f32>> {
    (0..h).map(|y| {
        (0..w).map(|x| plane[y * w + x]).collect()
    }).collect()
}

/// Bilinear interpolation from a 2D grid at float coordinates.
fn bilerp(grid: &[Vec<f32>], fx: f32, fy: f32) -> f32 {
    let w = grid[0].len() as f32;
    let h = grid.len() as f32;
    let x = fx.clamp(0.0, w - 1.0);
    let y = fy.clamp(0.0, h - 1.0);
    let x0 = x as usize;
    let y0 = y as usize;
    let x1 = (x0 + 1).min(w as usize - 1);
    let y1 = (y0 + 1).min(h as usize - 1);
    let dx = x - x0 as f32;
    let dy = y - y0 as f32;

    let v00 = grid[y0][x0];
    let v10 = grid[y0][x1];
    let v01 = grid[y1][x0];
    let v11 = grid[y1][x1];

    let v0 = v00 + (v10 - v00) * dx;
    let v1 = v01 + (v11 - v01) * dx;
    v0 + (v1 - v0) * dy
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

/// Apply tone curve: power + clip.
fn apply_tone(rgb: &[f32], params: &ToneParams) -> Vec<f32> {
    let gamma = if params.gamma_recip > 0.0 { 1.0 / params.gamma_recip } else { 1.0 };
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
    toned
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
