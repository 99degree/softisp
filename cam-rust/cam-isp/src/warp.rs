//! Warp operations — identity grid, radial distortion, bilinear warp sampling.
//!
//! Ported from `WarpBlock.kt` companion object and extracted from `cpu.rs`.

/// Generate an identity warp grid of shape `[H, W, 2]`.
///
/// Each `[y,x]` maps to normalized coordinates `[-1, 1]` where:
///   - grid`[y]``[x]``[0]` = y normalized (row)
///   - grid`[y]``[x]``[1]` = x normalized (col)
pub fn generate_identity_grid(h: usize, w: usize) -> Vec<f32> {
    let mut grid = vec![0.0f32; h * w * 2];
    let mut idx = 0;
    for y in 0..h {
        let ny = 2.0 * y as f32 / (h.max(1) - 1) as f32 - 1.0;
        for x in 0..w {
            let nx = 2.0 * x as f32 / (w.max(1) - 1) as f32 - 1.0;
            grid[idx] = ny;
            grid[idx + 1] = nx;
            idx += 2;
        }
    }
    grid
}

/// Apply radial distortion correction to an identity grid.
///
/// Uses the division model: r' = r / (1 + k1·r² + k2·r⁴ + k3·r⁶)
pub fn apply_radial_distortion(
    identity: &[f32],
    h: usize,
    w: usize,
    k1: f32,
    k2: f32,
    k3: f32,
    cx: f32,
    cy: f32,
) -> Vec<f32> {
    let mut grid = identity.to_vec();
    let mut idx = 0;
    for _y in 0..h {
        for _x in 0..w {
            if idx + 1 >= grid.len() {
                break;
            }
            let ny = grid[idx];
            let nx = grid[idx + 1];
            let dx = nx - cx;
            let dy = ny - cy;
            let r2 = dx * dx + dy * dy;
            let r4 = r2 * r2;
            let r6 = r4 * r2;
            let dist = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
            if dist.abs() > 1e-8 {
                grid[idx] = cy + dy / dist;
                grid[idx + 1] = cx + dx / dist;
            }
            idx += 2;
        }
    }
    grid
}

/// Apply a warp grid to an RGB image via bilinear sampling.
pub fn warp_image(rgb: &[f32], grid: &[f32], h: usize, w: usize) -> Vec<f32> {
    let mut out = vec![0.0f32; rgb.len()];
    for y in 0..h {
        for x in 0..w {
            let g_idx = (y * w + x) * 2;
            if g_idx + 1 >= grid.len() {
                continue;
            }
            let ny = grid[g_idx];
            let nx = grid[g_idx + 1];
            let src_x = (nx + 1.0) * 0.5 * (w - 1) as f32;
            let src_y = (ny + 1.0) * 0.5 * (h - 1) as f32;
            let x0 = src_x.floor() as isize;
            let y0 = src_y.floor() as isize;
            let x1 = (x0 + 1).min(w as isize - 1);
            let y1 = (y0 + 1).min(h as isize - 1);
            let x0 = x0.max(0);
            let y0 = y0.max(0);
            let fx = src_x - x0 as f32;
            let fy = src_y - y0 as f32;
            let p00 = (y0 as usize * w + x0 as usize) * 3;
            let p01 = (y0 as usize * w + x1 as usize) * 3;
            let p10 = (y1 as usize * w + x0 as usize) * 3;
            let p11 = (y1 as usize * w + x1 as usize) * 3;
            let out_idx = (y * w + x) * 3;
            if out_idx + 2 >= out.len() {
                continue;
            }
            for c in 0..3 {
                if p00 + c >= rgb.len()
                    || p01 + c >= rgb.len()
                    || p10 + c >= rgb.len()
                    || p11 + c >= rgb.len()
                {
                    continue;
                }
                let v = (1.0 - fx) * (1.0 - fy) * rgb[p00 + c]
                    + fx * (1.0 - fy) * rgb[p01 + c]
                    + (1.0 - fx) * fy * rgb[p10 + c]
                    + fx * fy * rgb[p11 + c];
                out[out_idx + c] = v.clamp(0.0, 1.0);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_grid_dimensions() {
        let grid = generate_identity_grid(4, 5);
        assert_eq!(grid.len(), 4 * 5 * 2);
        assert!((grid[0] - (-1.0)).abs() < 0.01);
        assert!((grid[1] - (-1.0)).abs() < 0.01);
        let last_idx = (4 * 5 - 1) * 2;
        assert!((grid[last_idx] - 1.0).abs() < 0.01);
        assert!((grid[last_idx + 1] - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_radial_distortion_identity() {
        let h = 8;
        let w = 8;
        let identity = generate_identity_grid(h, w);
        let corrected = apply_radial_distortion(&identity, h, w, 0.0, 0.0, 0.0, 0.0, 0.0);
        for i in 0..identity.len() {
            assert!((corrected[i] - identity[i]).abs() < 0.001);
        }
    }

    #[test]
    fn test_radial_distortion_barrel() {
        let h = 8;
        let w = 8;
        let identity = generate_identity_grid(h, w);
        let corrected = apply_radial_distortion(&identity, h, w, 0.1, 0.0, 0.0, 0.0, 0.0);
        let center_idx = (4 * w + 4) * 2;
        assert!((corrected[center_idx] - identity[center_idx]).abs() < 0.001);
    }

    #[test]
    fn test_warp_image_preserves_size() {
        let h = 10;
        let w = 10;
        let rgb = vec![0.5f32; h * w * 3];
        let identity = generate_identity_grid(h, w);
        let warped = warp_image(&rgb, &identity, h, w);
        assert_eq!(warped.len(), rgb.len());
        for i in 0..rgb.len() {
            assert!((warped[i] - rgb[i]).abs() < 0.001);
        }
    }

    #[test]
    fn test_identity_grid_range() {
        let grid = generate_identity_grid(32, 64);
        assert_eq!(grid.len(), 32 * 64 * 2);
        // Grid uses normalized coords in [-1, 1]
        for i in (0..grid.len()).step_by(2) {
            assert!(
                grid[i] >= -1.0 && grid[i] <= 1.0,
                "x at {} = {}",
                i,
                grid[i]
            );
            assert!(
                grid[i + 1] >= -1.0 && grid[i + 1] <= 1.0,
                "y at {} = {}",
                i,
                grid[i + 1]
            );
        }
    }

    #[test]
    fn test_radial_distortion_pincushion() {
        // Negative k1 = pincushion
        let grid = generate_identity_grid(8, 8);
        let distorted = apply_radial_distortion(&grid, 8, 8, -0.5, 0.0, 0.0, 0.5, 0.5);
        assert_eq!(distorted.len(), grid.len());
        let diff: f32 = grid
            .iter()
            .zip(distorted.iter())
            .map(|(a, b)| (a - b).abs())
            .sum();
        assert!(diff > 0.0, "pincushion should distort the grid");
    }

    #[test]
    fn test_warp_image_with_distortion() {
        let h = 8;
        let w = 8;
        let rgb = vec![0.5f32; h * w * 3];
        let grid = generate_identity_grid(h, w);
        let distorted = apply_radial_distortion(&grid, h, w, 0.1, 0.0, 0.0, 0.5, 0.5);
        let warped = warp_image(&rgb, &distorted, h, w);
        assert_eq!(warped.len(), rgb.len());
    }
}
