//! Warp application for deshake — bilinear interpolation, crop, border handling.

/// Convert BGRA interleaved bytes to planar RGB f32.
pub fn bgra_to_planar_rgb_f32(data: &[u8], width: u32, height: u32) -> Vec<f32> {
    let n = (width * height) as usize;
    let mut r = vec![0.0f32; n];
    let mut g = vec![0.0f32; n];
    let mut b = vec![0.0f32; n];
    for i in 0..n {
        let base = i * 4;
        if base + 3 < data.len() {
            b[i] = data[base] as f32 / 255.0;
            g[i] = data[base + 1] as f32 / 255.0;
            r[i] = data[base + 2] as f32 / 255.0;
        }
    }
    let mut out = vec![0.0f32; n * 3];
    for i in 0..n {
        out[i] = r[i];
        out[n + i] = g[i];
        out[2 * n + i] = b[i];
    }
    out
}

/// Apply warp to RGB planar f32 data using bilinear interpolation.
/// Returns warped RGB planar f32.
pub fn apply_warp_rgb_f32(
    rgb: &[f32], width: u32, height: u32,
    dx: f32, dy: f32,
    crop_fraction: f32,
) -> Vec<f32> {
    let n = (width * height) as usize;
    let mut out = vec![0.0f32; n * 3];

    let crop_px_x = (width as f32 * crop_fraction) as i32;
    let crop_px_y = (height as f32 * crop_fraction) as i32;

    for y in crop_px_y..(height as i32 - crop_px_y) {
        for x in crop_px_x..(width as i32 - crop_px_x) {
            // Source position (inverse warp)
            let src_x = x as f32 - dx;
            let src_y = y as f32 - dy;

            // Bilinear interpolation
            let x0 = src_x.floor() as i32;
            let y0 = src_y.floor() as i32;
            let x1 = x0 + 1;
            let y1 = y0 + 1;
            let fx = src_x - x0 as f32;
            let fy = src_y - y0 as f32;

            let sample = |c: usize, sx: i32, sy: i32| -> f32 {
                if sx >= 0 && sx < width as i32 && sy >= 0 && sy < height as i32 {
                    let idx = (sy as u32 * width + sx as u32) as usize + c * n;
                    rgb.get(idx).copied().unwrap_or(0.0)
                } else {
                    0.0
                }
            };

            let dst_idx = (y as u32 * width + x as u32) as usize;
            for c in 0..3 {
                let v00 = sample(c, x0, y0);
                let v10 = sample(c, x1, y0);
                let v01 = sample(c, x0, y1);
                let v11 = sample(c, x1, y1);
                let val = v00 * (1.0 - fx) * (1.0 - fy)
                    + v10 * fx * (1.0 - fy)
                    + v01 * (1.0 - fx) * fy
                    + v11 * fx * fy;
                out[c * n + dst_idx] = val;
            }
        }
    }

    out
}

/// Downscale grayscale image by factor of 2.
pub fn downscale_gray(gray: &[u8], w: u32, h: u32) -> Vec<u8> {
    let nw = w / 2;
    let nh = h / 2;
    let mut out = vec![0u8; (nw * nh) as usize];
    for y in 0..nh {
        for x in 0..nw {
            let sx = x * 2;
            let sy = y * 2;
            let mut sum = 0u32;
            for dy in 0..2u32 {
                for dx in 0..2u32 {
                    let idx = ((sy + dy) * w + (sx + dx)) as usize;
                    sum += gray.get(idx).copied().unwrap_or(0) as u32;
                }
            }
            out[(y * nw + x) as usize] = (sum / 4) as u8;
        }
    }
    out
}

/// Convert RGB or RGBA bytes to grayscale.
pub fn to_grayscale(data: &[u8], width: u32, height: u32) -> Vec<u8> {
    let n = (width * height) as usize;
    let stride = data.len() / n;
    let mut gray = vec![0u8; n];
    for i in 0..n {
        let base = i * stride;
        if base + 2 < data.len() {
            let r = data[base] as f32;
            let g = data[base + 1] as f32;
            let b = data[base + 2] as f32;
            gray[i] = (0.299 * r + 0.587 * g + 0.114 * b) as u8;
        }
    }
    gray
}

/// Compute downscale factor to reach target largest dimension.
pub fn compute_downscale_factor(width: u32, height: u32, target: u32) -> u32 {
    let max_dim = width.max(height);
    if max_dim <= target {
        return 1;
    }
    // Find largest power of 2 that brings max_dim <= target
    let mut factor = 1u32;
    while max_dim / (factor * 2) >= target {
        factor *= 2;
    }
    factor
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_grayscale() {
        let rgb = vec![100u8, 150, 200, 100, 150, 200];
        let gray = to_grayscale(&rgb, 2, 1);
        assert_eq!(gray.len(), 2);
        let expected = (0.299 * 100.0 + 0.587 * 150.0 + 0.114 * 200.0) as u8;
        assert_eq!(gray[0], expected);
    }

    #[test]
    fn test_downscale_gray() {
        let gray = vec![100u8; 16 * 16];
        let small = downscale_gray(&gray, 16, 16);
        assert_eq!(small.len(), 8 * 8);
        assert!(small.iter().all(|&v| v == 100));
    }

    #[test]
    fn test_compute_downscale_factor() {
        assert_eq!(compute_downscale_factor(1920, 1080, 384), 4);
        assert_eq!(compute_downscale_factor(640, 480, 384), 1);
        assert_eq!(compute_downscale_factor(3840, 2160, 384), 8);
    }

    #[test]
    fn test_apply_warp_identity() {
        let rgb = vec![0.5f32; 100 * 100 * 3];
        let warped = apply_warp_rgb_f32(&rgb, 100, 100, 0.0, 0.0, 0.0);
        // Center pixel should be unchanged
        let _n = 10000;
        let center = 50 * 100 + 50;
        assert!((warped[center] - 0.5).abs() < 0.01);
    }
}
