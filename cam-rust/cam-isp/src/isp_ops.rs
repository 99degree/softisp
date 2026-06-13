//! ISP pixel processing operations — DPC, denoise, BLC/WB, LSC, CCM, tone, etc.
//!
//! All functions operate on flat float arrays. Ported from `cpu.rs`.

use cam_types::ToneParams;

/// Generate simulated raw Bayer data from RGBA test pattern (BGGR).
pub(crate) fn generate_simulated_raw(width: u32, height: u32, rgba: &[u8]) -> Vec<u16> {
    let mut raw = Vec::with_capacity((width * height) as usize);
    for y in 0..height {
        for x in 0..width {
            let idx = (y * width + x) as usize * 4;
            let r = if idx + 3 < rgba.len() { rgba[idx] as u32 } else { 128 };
            let g = if idx + 2 < rgba.len() { rgba[idx + 1] as u32 } else { 128 };
            let b = if idx + 3 < rgba.len() { rgba[idx + 2] as u32 } else { 128 };
            // BGGR pattern
            let raw_val = if y % 2 == 0 {
                if x % 2 == 0 { b } else { g }
            } else {
                if x % 2 == 0 { g } else { r }
            };
            raw.push((raw_val * 257) as u16);
        }
    }
    raw
}

/// Apply defective pixel correction (DPC) — 3x3 median-based hot pixel removal.
pub(crate) fn apply_dpc(cfa: &[f32], w: usize, h: usize, threshold: f32) -> Vec<f32> {
    let mut out = cfa.to_vec();
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let idx = y * w + x;
            let center = cfa[idx];

            // Collect 3x3 neighborhood
            let mut neighbors = Vec::with_capacity(9);
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let nx = (x as isize + dx) as usize;
                    let ny = (y as isize + dy) as usize;
                    neighbors.push(cfa[ny * w + nx]);
                }
            }
            neighbors.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let median = neighbors[4];

            // If center deviates too much from median, replace
            if (center - median).abs() > threshold {
                out[idx] = median;
            }
        }
    }
    out
}

/// Apply Gaussian denoise — 3x3 blur with strength blend.
pub(crate) fn apply_gaussian_denoise(raw: &[f32], w: usize, h: usize, strength: f32) -> Vec<f32> {
    if strength <= 0.0 {
        return raw.to_vec();
    }
    let kernel: [f32; 9] = [1.0, 2.0, 1.0, 2.0, 4.0, 2.0, 1.0, 2.0, 1.0];
    let k_sum: f32 = kernel.iter().sum();
    let mut out = raw.to_vec();

    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let mut sum = 0.0f32;
            for ky in 0..3 {
                for kx in 0..3 {
                    let px = (x as isize + kx as isize - 1) as usize;
                    let py = (y as isize + ky as isize - 1) as usize;
                    sum += raw[py * w + px] * kernel[ky * 3 + kx];
                }
            }
            let blurred = sum / k_sum;
            let idx = y * w + x;
            out[idx] = raw[idx] * (1.0 - strength) + blurred * strength;
        }
    }
    out
}

/// Apply black level correction + white balance on raw CFA.
pub(crate) fn apply_blc_wb_raw(
    float: &[f32],
    width: usize,
    height: usize,
    blc: &[f32; 4],
    gains: &[f32; 4],
) -> Vec<f32> {
    let mut out = float.to_vec();
    for y in 0..height {
        for x in 0..width {
            let idx = y * width + x;
            let ch = match (y % 2, x % 2) {
                (0, 0) => 0, // R
                (0, 1) => 1, // Gr
                (1, 0) => 2, // Gb
                (1, 1) => 3, // B
                _ => 0,
            };
            let val = (float[idx] - blc[ch]) * gains[ch];
            out[idx] = val.max(0.0).min(1.0);
        }
    }
    out
}

/// Apply lens shading correction — radial gain model.
pub(crate) fn apply_lsc(raw: &[f32], w: usize, h: usize, k: f32) -> Vec<f32> {
    if k <= 0.0 {
        return raw.to_vec();
    }
    let cx = (w - 1) as f32 / 2.0;
    let cy = (h - 1) as f32 / 2.0;
    let max_r2 = cx * cx + cy * cy;
    let mut out = raw.to_vec();
    for y in 0..h {
        for x in 0..w {
            let dx = x as f32 - cx;
            let dy = y as f32 - cy;
            let r2 = dx * dx + dy * dy;
            let gain = 1.0 + k * r2 / max_r2;
            let idx = y * w + x;
            out[idx] = (raw[idx] * gain).min(1.0);
        }
    }
    out
}

/// Apply 3x3 color correction matrix to RGB.
#[allow(dead_code)]
pub(crate) fn apply_ccm(rgb: &[f32], matrix: &[f32; 9]) -> Vec<f32> {
    let count = rgb.len() / 3;
    let mut out = Vec::with_capacity(rgb.len());
    for i in 0..count {
        let idx = i * 3;
        let r = rgb[idx];
        let g = rgb[idx + 1];
        let b = rgb[idx + 2];
        let nr = (matrix[0] * r + matrix[1] * g + matrix[2] * b).clamp(0.0, 1.0);
        let ng = (matrix[3] * r + matrix[4] * g + matrix[5] * b).clamp(0.0, 1.0);
        let nb = (matrix[6] * r + matrix[7] * g + matrix[8] * b).clamp(0.0, 1.0);
        out.push(nr);
        out.push(ng);
        out.push(nb);
    }
    out
}

/// Apply tone curve: gamma, contrast, brightness, saturation, unsharp mask.
pub(crate) fn apply_tone(rgb: &[f32], params: &ToneParams, w: usize, h: usize) -> Vec<f32> {
    let gamma_recip = params.gamma_recip;
    let gamma = if gamma_recip > 0.0 { 1.0 / gamma_recip } else { 1.0 };
    let contrast = params.contrast;
    let brightness = params.brightness;
    let saturation = params.saturation;
    let sharpness = params.sharpness;

    let count = rgb.len() / 3;
    let mut toned = Vec::with_capacity(rgb.len());

    for i in 0..count {
        let idx = i * 3;
        let mut r = rgb[idx];
        let mut g = rgb[idx + 1];
        let mut b = rgb[idx + 2];

        r = r.powf(gamma);
        g = g.powf(gamma);
        b = b.powf(gamma);

        r = ((r - 0.5) * contrast) + 0.5;
        g = ((g - 0.5) * contrast) + 0.5;
        b = ((b - 0.5) * contrast) + 0.5;

        r += brightness;
        g += brightness;
        b += brightness;

        let luma = 0.299 * r + 0.587 * g + 0.114 * b;
        r = luma + (r - luma) * saturation;
        g = luma + (g - luma) * saturation;
        b = luma + (b - luma) * saturation;

        toned.push(r.max(0.0).min(1.0));
        toned.push(g.max(0.0).min(1.0));
        toned.push(b.max(0.0).min(1.0));
    }

    // Apply unsharp mask edge enhancement
    if sharpness > 0.0 {
        apply_unsharp_mask(&mut toned, w, h, sharpness);
    }

    toned
}

/// Apply unsharp mask edge enhancement — 3x3 Laplacian.
pub(crate) fn apply_unsharp_mask(rgb: &mut [f32], w: usize, h: usize, strength: f32) {
    let laplacian: [f32; 9] = [0.0, -1.0, 0.0, -1.0, 4.0, -1.0, 0.0, -1.0, 0.0];
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let idx = (y * w + x) * 3;
            for c in 0..3 {
                let mut edge = 0.0f32;
                for ky in 0..3 {
                    for kx in 0..3 {
                        let px = (x + kx).wrapping_sub(1);
                        let py = (y + ky).wrapping_sub(1);
                        let p = (py * w + px) * 3 + c;
                        if p < rgb.len() {
                            edge += rgb[p] * laplacian[ky * 3 + kx];
                        }
                    }
                }
                rgb[idx + c] = (rgb[idx + c] + edge * strength).clamp(0.0, 1.0);
            }
        }
    }
}

/// Apply AE gain to RGB.
#[allow(dead_code)]
pub(crate) fn apply_ae_gain(rgb: &[f32], gain: f32) -> Vec<f32> {
    if (gain - 1.0).abs() < 0.001 {
        return rgb.to_vec();
    }
    rgb.iter().map(|&v| (v * gain).min(1.0)).collect()
}

/// Calculate AE gain targeting 18% gray from RGB stats.
pub(crate) fn calculate_ae_gain(rgb: &[f32]) -> f32 {
    let target_luma = 0.18;
    let count = rgb.len() / 3;
    if count == 0 {
        return 1.0;
    }
    let mut sum_luma = 0.0f64;
    for i in 0..count {
        let idx = i * 3;
        let luma = 0.299 * rgb[idx] as f64 + 0.587 * rgb[idx + 1] as f64 + 0.114 * rgb[idx + 2] as f64;
        sum_luma += luma;
    }
    let mean_luma = sum_luma / count as f64;
    if mean_luma > 0.0 {
        (target_luma / mean_luma).min(4.0) as f32
    } else {
        1.0
    }
}

/// Calculate auto white balance gains using gray world assumption on raw CFA.
#[allow(dead_code)]
pub(crate) fn calculate_awb_gains(cfa: &[f32], width: usize, height: usize) -> [f32; 4] {
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

    let avg_r = sum_r / count_r.max(1) as f32;
    let avg_gr = sum_gr / count_gr.max(1) as f32;
    let avg_gb = sum_gb / count_gb.max(1) as f32;
    let avg_b = sum_b / count_b.max(1) as f32;
    let avg_g = (avg_gr + avg_gb) / 2.0;

    let r_gain = if avg_r > 0.0 { (avg_g / avg_r).clamp(0.25, 4.0) } else { 1.0 };
    let gr_gain = if avg_gr > 0.0 { (avg_g / avg_gr).clamp(0.25, 4.0) } else { 1.0 };
    let gb_gain = if avg_gb > 0.0 { (avg_g / avg_gb).clamp(0.25, 4.0) } else { 1.0 };
    let b_gain = if avg_b > 0.0 { (avg_g / avg_b).clamp(0.25, 4.0) } else { 1.0 };

    [r_gain, gr_gain, gb_gain, b_gain]
}

/// Display output: resize (nearest neighbor) + convert to UINT8 BGRA.
#[allow(dead_code)]
pub(crate) fn display_output(rgb: &[f32], src_w: usize, src_h: usize, target_w: usize) -> Vec<u8> {
    if rgb.len() < 3 {
        return Vec::new();
    }
    let target_h = if target_w < src_w {
        (src_h as f32 * target_w as f32 / src_w as f32) as usize
    } else {
        src_h
    };
    let mut out = vec![0u8; target_w * target_h * 4];
    for y in 0..target_h {
        for x in 0..target_w {
            let sx = x * src_w / target_w;
            let sy = y * src_h / target_h;
            let src_idx = (sy * src_w + sx) * 3;
            let dst_idx = (y * target_w + x) * 4;
            if src_idx + 2 < rgb.len() {
                out[dst_idx] = (rgb[src_idx] * 255.0) as u8;
                out[dst_idx + 1] = (rgb[src_idx + 1] * 255.0) as u8;
                out[dst_idx + 2] = (rgb[src_idx + 2] * 255.0) as u8;
                out[dst_idx + 3] = 255;
            }
        }
    }
    out
}

/// False Color Suppression — edge-aware chroma desaturation.
pub(crate) fn apply_fcs(rgb: &[f32], w: usize, h: usize, strength: f32) -> Vec<f32> {
    if strength <= 0.0 || rgb.len() < 9 {
        return rgb.to_vec();
    }
    let mut out = rgb.to_vec();
    // 3x5 edge detection kernel (openISP style)
    let kernel: [f32; 15] = [
        -0.125, 0.0, -0.125, 0.0, -0.125,
        -0.125, 0.0, 1.0,   0.0, -0.125,
        -0.125, 0.0, -0.125, 0.0, -0.125,
    ];
    let gain = strength * 0.125;
    for y in 0..h {
        for x in 0..w {
            let idx = (y * w + x) * 3;
            if idx + 2 >= rgb.len() { continue; }
            let mut edge_y = 0.0f32;
            for ky in 0..3 {
                for kx in 0..5 {
                    let py = y as isize + ky as isize - 1;
                    let px = x as isize + kx as isize - 2;
                    if py < 0 || py >= h as isize || px < 0 || px >= w as isize { continue; }
                    let p_idx = (py as usize * w + px as usize) * 3;
                    if p_idx + 2 >= rgb.len() { continue; }
                    let lum = 0.299 * rgb[p_idx] + 0.587 * rgb[p_idx + 1] + 0.114 * rgb[p_idx + 2];
                    edge_y += lum * kernel[ky * 5 + kx];
                }
            }
            let edge_strength = (edge_y.abs() * gain).clamp(0.0, 1.0);
            let atten = 1.0 - edge_strength;
            let r = out[idx];
            let g = out[idx + 1];
            let b = out[idx + 2];
            let luma = 0.299 * r + 0.587 * g + 0.114 * b;
            out[idx]     = luma + (r - luma) * atten;
            out[idx + 1] = luma + (g - luma) * atten;
            out[idx + 2] = luma + (b - luma) * atten;
        }
    }
    out
}

/// Local Dynamic Contrast Improvement — tile-based contrast stretch.
pub(crate) fn apply_ldci(rgb: &[f32], w: usize, h: usize, strength: f32) -> Vec<f32> {
    if strength <= 0.0 || rgb.len() < 9 {
        return rgb.to_vec();
    }
    let mut out = vec![0.0f32; rgb.len()];
    let radius: usize = 4; // 9x9 box
    for y in 0..h {
        for x in 0..w {
            let idx = (y * w + x) * 3;
            if idx + 2 >= rgb.len() { continue; }
            let r = rgb[idx];
            let g = rgb[idx + 1];
            let b = rgb[idx + 2];
            let luma = 0.299 * r + 0.587 * g + 0.114 * b;
            let mut sum = 0.0f32;
            let mut count_px = 0u32;
            for dy in -(radius as isize)..=radius as isize {
                for dx in -(radius as isize)..=radius as isize {
                    let py = y as isize + dy;
                    let px = x as isize + dx;
                    if py < 0 || py >= h as isize || px < 0 || px >= w as isize { continue; }
                    let p_idx = (py as usize * w + px as usize) * 3;
                    if p_idx + 2 >= rgb.len() { continue; }
                    let pl = 0.299 * rgb[p_idx] + 0.587 * rgb[p_idx + 1] + 0.114 * rgb[p_idx + 2];
                    sum += pl;
                    count_px += 1;
                }
            }
            let local_mean = sum / count_px.max(1) as f32;
            let local_contrast = luma - local_mean;
            let boost = local_contrast * strength.clamp(0.0, 1.0);
            let enhanced_luma = (luma + boost).clamp(0.0, 1.0);
            let scale = if luma > 0.001 { enhanced_luma / luma } else { 1.0 };
            out[idx]     = (r * scale).clamp(0.0, 1.0);
            out[idx + 1] = (g * scale).clamp(0.0, 1.0);
            out[idx + 2] = (b * scale).clamp(0.0, 1.0);
        }
    }
    out
}
