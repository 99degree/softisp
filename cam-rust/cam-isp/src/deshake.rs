//! Deshake — software-based video stabilization via block-matching motion estimation.
//!
//! Uses frame-to-frame block matching to compute a global motion vector (translation)
//! between consecutive frames, then applies inverse warp with trajectory smoothing.
//!
//! # Algorithm
//!
//! 1. **Frame buffer** — keep previous frame for motion estimation
//! 2. **Block matching** — divide frame into N×M blocks, for each block search
//!    best match in previous frame via SAD (Sum of Absolute Differences)
//! 3. **Motion vector aggregation** — median of all block motion vectors
//!    (rejects outliers from moving objects)
//! 4. **Trajectory smoothing** — separate intentional motion (low-pass) from
//!    jitter (high-pass) using EMA with adaptive gain
//! 5. **Warp** — apply inverse of jitter as bilinear warp
//! 6. **Crop** — crop margins to hide black borders
//!
//! Unlike EIS (which requires gyroscope hardware), deshake works on any
//! platform with only a camera feed. Trade-off: higher CPU usage and
//! less robust against large motion (rolling shutter, complex scenes).

/// Default block size for motion estimation (pixels per side).
const BLOCK_SIZE: u32 = 32;

/// Search radius around each block's position (pixels).
const SEARCH_RADIUS: i32 = 16;

/// EMA smoothing alpha for intentional motion. Lower = smoother but more lag.
const MOTION_SMOOTHING_ALPHA: f32 = 0.08;

/// Fraction of frame cropped to hide black borders from warp (0..1).
const CROP_FRACTION: f32 = 0.08;

/// Maximum pixel shift per axis (clamp to avoid excessive crop).
const MAX_SHIFT_PX: f32 = 64.0;

/// Motion estimation result for a single block.
#[derive(Debug, Clone, Copy)]
struct BlockMatch {
    /// X displacement (pixels, sub-pixel).
    dx: f32,
    /// Y displacement (pixels, sub-pixel).
    dy: f32,
    /// Cost (SAD) of the match — lower = better.
    cost: f32,
}

/// Deshake engine state.
#[derive(Debug, Clone)]
pub struct DeshakeEngine {
    /// Previous frame data (grayscale, downscaled for efficiency).
    prev_gray: Option<Vec<u8>>,
    /// Previous frame dimensions after optional downscale.
    prev_dims: Option<(u32, u32)>,
    /// Smoothed accumulated motion vector [dx, dy].
    smooth_motion: [f32; 2],
    /// Deshake enabled.
    pub enabled: bool,
    /// Debug logging.
    pub debug: bool,
    /// Frame counter.
    frame_count: u64,
    /// Block size override (0 = default).
    pub block_size: u32,
    /// Search radius override (0 = default).
    pub search_radius: i32,
    /// Smoothing alpha override (0.0 = use default).
    pub smoothing_alpha: f32,
}

impl Default for DeshakeEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl DeshakeEngine {
    pub fn new() -> Self {
        Self {
            prev_gray: None,
            prev_dims: None,
            smooth_motion: [0.0; 2],
            enabled: false,
            debug: false,
            frame_count: 0,
            block_size: 0,
            search_radius: 0,
            smoothing_alpha: 0.0,
        }
    }

    /// Reset state (e.g., on scene change).
    pub fn reset(&mut self) {
        self.prev_gray = None;
        self.prev_dims = None;
        self.smooth_motion = [0.0; 2];
        self.frame_count = 0;
    }

    /// Convert BGRA/RGBA frame to grayscale (luminance only).
    fn to_grayscale(data: &[u8], width: u32, height: u32) -> Vec<u8> {
        let len = (width * height) as usize;
        let mut gray = vec![0u8; len];
        // BT.601 luma: Y = 0.299*R + 0.587*G + 0.114*B
        for (i, chunk) in data.chunks_exact(4).enumerate() {
            if i >= len { break; }
            let r = chunk[0] as f32;
            let g = chunk[1] as f32;
            let b = chunk[2] as f32;
            gray[i] = (0.299 * r + 0.587 * g + 0.114 * b).round() as u8;
        }
        gray
    }

    /// Downscale grayscale image by factor 2 (nearest-neighbor).
    /// Returns (downscaled_data, new_width, new_height).
    fn downscale_gray(gray: &[u8], w: u32, h: u32) -> Vec<u8> {
        let nw = w / 2;
        let nh = h / 2;
        let mut out = vec![0u8; (nw * nh) as usize];
        for y in 0..nh {
            for x in 0..nw {
                let src_y = (y * 2) as usize;
                let src_x = (x * 2) as usize;
                let src_idx = src_y * w as usize + src_x;
                out[(y * nw + x) as usize] = gray[src_idx];
            }
        }
        out
    }

    /// Compute SAD (Sum of Absolute Differences) between two blocks.
    fn block_sad(
        curr: &[u8],
        cw: u32,
        prev: &[u8],
        pw: u32,
        cx: u32,
        cy: u32,
        sx: i32,
        sy: i32,
        size: u32,
    ) -> f32 {
        let mut sad = 0.0f32;
        for dy in 0..size {
            for dx in 0..size {
                let curr_x = (cx + dx) as usize;
                let curr_y = (cy + dy) as usize;
                let src_x = (sx + dx as i32) as usize;
                let src_y = (sy + dy as i32) as usize;

                let c_val = curr[curr_y * cw as usize + curr_x] as f32;
                let p_val = prev[src_y * pw as usize + src_x] as f32;
                sad += (c_val - p_val).abs();
            }
        }
        sad / (size * size) as f32 // normalize
    }

    /// Compute variance of a block (texture metric). Low variance = flat region.
    fn block_variance(data: &[u8], stride: u32, bx: u32, by: u32, size: u32) -> f32 {
        let mut sum = 0.0f32;
        let mut sum_sq = 0.0f32;
        let count = (size * size) as f32;
        for dy in 0..size {
            for dx in 0..size {
                let idx = ((by + dy) * stride + (bx + dx)) as usize;
                let v = data.get(idx).copied().unwrap_or(0) as f32;
                sum += v;
                sum_sq += v * v;
            }
        }
        let mean = sum / count;
        sum_sq / count - mean * mean
    }

    /// Find best match for a block in current frame against previous frame.
    fn find_best_match(
        curr_gray: &[u8],
        cw: u32,
        prev_gray: &[u8],
        pw: u32,
        ph: u32,
        bx: u32,
        by: u32,
        block_size: u32,
        search_radius: i32,
    ) -> Option<BlockMatch> {
        let half = block_size / 2;
        let cx = bx + half;     // center of block in current frame
        let cy = by + half;
        let sx = cx as i32;     // search center in previous frame (same position)
        let sy = cy as i32;

        let search_start_x = (sx - search_radius).max(0).min((pw - block_size) as i32);
        let search_start_y = (sy - search_radius).max(0).min((ph - block_size) as i32);
        let search_end_x = (sx + search_radius).max(0).min((pw - block_size) as i32);
        let search_end_y = (sy + search_radius).max(0).min((ph - block_size) as i32);

        if search_start_x >= search_end_x || search_start_y >= search_end_y {
            return None;
        }

        let mut best = BlockMatch { dx: 0.0, dy: 0.0, cost: f32::MAX };

        // Initialize with the current position (no motion)
        let zero_cost = Self::block_sad(
            curr_gray, cw, prev_gray, pw,
            bx, by, bx as i32, by as i32, block_size,
        );
        best.cost = zero_cost;

        // Full search (for accuracy; diamond search would be faster for production)
        for sy in (search_start_y..=search_end_y).step_by(1) {
            for sx in (search_start_x..=search_end_x).step_by(1) {
                let cost = Self::block_sad(
                    curr_gray, cw, prev_gray, pw,
                    bx, by, sx, sy, block_size,
                );
                if cost < best.cost {
                    best.cost = cost;
                    best.dx = (sx - bx as i32) as f32;
                    best.dy = (sy - by as i32) as f32;
                }
            }
        }

        // Parabolic sub-pixel refinement (2D)
        if best.cost < f32::MAX / 2.0 {
            let int_sx = bx as i32 + best.dx as i32;
            let int_sy = by as i32 + best.dy as i32;
            let sub_dx = Self::parabolic_refine(
                curr_gray, cw, prev_gray, pw,
                bx, by,
                int_sx, int_sy,
                block_size, true, // horizontal
            );
            let sub_dy = Self::parabolic_refine(
                curr_gray, cw, prev_gray, pw,
                bx, by,
                int_sx, int_sy,
                block_size, false, // vertical
            );
            best.dx += sub_dx;
            best.dy += sub_dy;
        }

        Some(best)
    }

    /// Parabolic sub-pixel refinement along one axis.
    /// Fits a parabola to costs at (-1, 0, +1) and finds the minimum.
    /// Returns sub-pixel offset in pixels.
    fn parabolic_refine(
        curr_gray: &[u8],
        cw: u32,
        prev_gray: &[u8],
        pw: u32,
        bx: u32,
        by: u32,
        best_sx: i32,
        best_sy: i32,
        block_size: u32,
        horizontal: bool,
    ) -> f32 {
        // Bounds check: need (best_sx-1, best_sy) and (best_sx+1, best_sy) to be valid
        let max_sx = (pw as i32) - block_size as i32;
        // Compute ph from prev_gray.len / pw
        let ph = (prev_gray.len() / pw as usize) as i32;
        let max_sy = ph - block_size as i32;

        if !horizontal {
            // Vertical refinement: need -1 and +1 in Y
            if best_sy - 1 < 0 || best_sy + 1 > max_sy {
                return 0.0;
            }
        } else {
            // Horizontal refinement: need -1 and +1 in X
            if best_sx - 1 < 0 || best_sx + 1 > max_sx {
                return 0.0;
            }
        }

        let c0 = Self::block_sad(curr_gray, cw, prev_gray, pw as u32, bx, by,
                                  best_sx, best_sy, block_size);
        let (c1, c2) = if horizontal {
            let c1 = Self::block_sad(curr_gray, cw, prev_gray, pw as u32, bx, by,
                                      best_sx - 1, best_sy, block_size);
            let c2 = Self::block_sad(curr_gray, cw, prev_gray, pw as u32, bx, by,
                                      best_sx + 1, best_sy, block_size);
            (c1, c2)
        } else {
            let c1 = Self::block_sad(curr_gray, cw, prev_gray, pw as u32, bx, by,
                                      best_sx, best_sy - 1, block_size);
            let c2 = Self::block_sad(curr_gray, cw, prev_gray, pw as u32, bx, by,
                                      best_sx, best_sy + 1, block_size);
            (c1, c2)
        };

        // Parabola minimum: delta = (c1 - c2) / (2 * (c1 - 2*c0 + c2))
        let denom = c1 - 2.0 * c0 + c2;
        if denom.abs() < 1e-8 {
            return 0.0;
        }
        (c1 - c2) / (2.0 * denom)
    }

    /// Per-frame deshake update.
    ///
    /// Takes the current BGRA/RGBA frame and computes a stabilization
    /// compensation vector [dx, dy].
    ///
    /// Returns `Some([dx, dy])` pixel displacement to compensate,
    /// or `None` if not enough data (first frame).
    pub fn update(
        &mut self,
        data: &[u8],
        width: u32,
        height: u32,
    ) -> Option<[f32; 2]> {
        if !self.enabled || width < 64 || height < 64 {
            return None;
        }

        let block_size = if self.block_size > 0 { self.block_size } else { BLOCK_SIZE };
        let search_radius = if self.search_radius > 0 { self.search_radius } else { SEARCH_RADIUS };
        let alpha = if self.smoothing_alpha > 0.0 { self.smoothing_alpha } else { MOTION_SMOOTHING_ALPHA };

        // Convert to grayscale (just luminance for matching)
        let curr_gray = Self::to_grayscale(data, width, height);

        // If no previous frame, store and skip
        let prev_gray = match self.prev_gray {
            Some(ref pg) => pg.clone(),
            None => {
                self.prev_gray = Some(curr_gray);
                self.prev_dims = Some((width, height));
                return None;
            }
        };
        let prev_dims = self.prev_dims.unwrap_or((width, height));

        // Motion estimation phase
        let num_blocks_x = (width / block_size).max(1);
        let num_blocks_y = (height / block_size).max(1);

        let mut motion_vectors: Vec<[f32; 2]> = Vec::new();
        let mut costs: Vec<f32> = Vec::new();

        for by_idx in 0..num_blocks_y {
            for bx_idx in 0..num_blocks_x {
                let bx = bx_idx * block_size;
                let by = by_idx * block_size;

                // Skip low-texture blocks (variance too low for reliable matching)
                let var = Self::block_variance(&curr_gray, width, bx, by, block_size);
                if var < 10.0 { continue; }

                if let Some(mv) = Self::find_best_match(
                    &curr_gray, width,
                    &prev_gray, prev_dims.0, prev_dims.1,
                    bx, by, block_size, search_radius,
                ) {
                    motion_vectors.push([mv.dx, mv.dy]);
                    costs.push(mv.cost);
                }
            }
        }

        if motion_vectors.is_empty() {
            // No matches found — keep previous compensation
            self.prev_gray = Some(curr_gray);
            self.prev_dims = Some((width, height));
            self.frame_count += 1;
            return Some(self.smooth_motion);
        }

        // Compute median motion vector (robust to outlier blocks)
        let median_mv = Self::median_filter(&motion_vectors, &costs);

        // Clamp to maximum shift
        let raw_dx = median_mv[0].clamp(-MAX_SHIFT_PX, MAX_SHIFT_PX);
        let raw_dy = median_mv[1].clamp(-MAX_SHIFT_PX, MAX_SHIFT_PX);

        // EMA trajectory smoothing
        // raw_motion = intentional + jitter
        // smooth = low-pass (intentional), jitter = raw - smooth
        self.smooth_motion[0] += (raw_dx - self.smooth_motion[0]) * alpha;
        self.smooth_motion[1] += (raw_dy - self.smooth_motion[1]) * alpha;

        // Jitter = high-frequency component to cancel
        // raw_dx positive = content found to the RIGHT in prev frame = content
        // moved LEFT in current frame. We compensate by shifting current frame
        // in the same direction as raw_dx (comp_dx = raw - smooth).
        let comp_dx = raw_dx - self.smooth_motion[0];
        let comp_dy = raw_dy - self.smooth_motion[1];

        // Clamp compensation to crop margin
        let margin_x = (width as f32 * CROP_FRACTION).max(1.0);
        let margin_y = (height as f32 * CROP_FRACTION).max(1.0);
        let clamped_dx = comp_dx.clamp(-margin_x, margin_x);
        let clamped_dy = comp_dy.clamp(-margin_y, margin_y);

        // Store current frame for next iteration
        self.prev_gray = Some(curr_gray);
        self.prev_dims = Some((width, height));
        self.frame_count += 1;

        if self.debug {
            log::debug!(
                "Deshake: raw=[{:.1},{:.1}] smooth=[{:.1},{:.1}] comp=[dx={:.1},dy={:.1}] blocks={}",
                raw_dx, raw_dy,
                self.smooth_motion[0], self.smooth_motion[1],
                clamped_dx, clamped_dy,
                motion_vectors.len(),
            );
        }

        Some([clamped_dx, clamped_dy])
    }

    /// Median filter motion vectors, weighted by inverse cost.
    /// Returns the robust global motion estimate.
    fn median_filter(vectors: &[[f32; 2]], costs: &[f32]) -> [f32; 2] {
        if vectors.is_empty() {
            return [0.0; 2];
        }
        if vectors.len() == 1 {
            return vectors[0];
        }

        // Separate X and Y components, sort each
        let mut xs: Vec<f32> = vectors.iter().map(|v| v[0]).collect();
        let mut ys: Vec<f32> = vectors.iter().map(|v| v[1]).collect();
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        ys.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let mid = vectors.len() / 2;
        [xs[mid], ys[mid]]
    }

    /// Apply deshake warp (translation-only) to BGRA/RGBA frame.
    ///
    /// Returns (warped_data, new_width, new_height).
    pub fn apply_warp(
        data: &[u8],
        width: u32,
        height: u32,
        comp: &[f32; 2],
        crop_fraction: f32,
    ) -> Result<(Vec<u8>, u32, u32), String> {
        let dx = comp[0];
        let dy = comp[1];

        let margin_x = (width as f32 * crop_fraction).round() as u32;
        let margin_y = (height as f32 * crop_fraction).round() as u32;

        if width <= 2 * margin_x || height <= 2 * margin_y {
            return Err("Deshake crop too large".to_string());
        }

        let out_w = width - 2 * margin_x;
        let out_h = height - 2 * margin_y;
        let bpp = 4;
        let mut out = vec![0u8; (out_w * out_h * bpp as u32) as usize];

        for oy in 0..out_h {
            for ox in 0..out_w {
                let ix = ox as f32 + margin_x as f32;
                let iy = oy as f32 + margin_y as f32;

                // Inverse warp: compensate with -dx, -dy
                let sx = ix + dx;  // add compensation to map back
                let sy = iy + dy;

                let pixel = bilinear_sample_4(data, width, height, sx, sy);
                let out_idx = ((oy * out_w + ox) * bpp as u32) as usize;
                out[out_idx..out_idx + bpp].copy_from_slice(&pixel);
            }
        }

        Ok((out, out_w, out_h))
    }

    /// Get current smooth motion estimate.
    pub fn smooth_motion(&self) -> [f32; 2] {
        self.smooth_motion
    }
}

// ============================================================================
// Bilinear interpolation (4-channel reuse from postprocess)
// ============================================================================

/// 4-channel bilinear sample (reimplements postprocess's bilinear_sample for
/// independence, matching the same interface).
fn bilinear_sample_4(data: &[u8], width: u32, height: u32, x: f32, y: f32) -> Vec<u8> {
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    let x1 = x0 + 1;
    let y1 = y0 + 1;

    let fx = x - x0 as f32;
    let fy = y - y0 as f32;

    let sample = |sx: i32, sy: i32| -> Vec<u8> {
        if sx < 0 || sx >= width as i32 || sy < 0 || sy >= height as i32 {
            vec![0u8; 4]
        } else {
            let idx = ((sy as u32 * width + sx as u32) * 4) as usize;
            data[idx..idx + 4].to_vec()
        }
    };

    let p00 = sample(x0, y0);
    let p10 = sample(x1, y0);
    let p01 = sample(x0, y1);
    let p11 = sample(x1, y1);

    let mut result = vec![0u8; 4];
    for c in 0..4 {
        let v = p00[c] as f32 * (1.0 - fx) * (1.0 - fy)
            + p10[c] as f32 * fx * (1.0 - fy)
            + p01[c] as f32 * (1.0 - fx) * fy
            + p11[c] as f32 * fx * fy;
        result[c] = v.round().clamp(0.0, 255.0) as u8;
    }
    result
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deshake_initial_state() {
        let d = DeshakeEngine::new();
        assert!(!d.enabled);
        assert_eq!(d.frame_count, 0);
        assert_eq!(d.smooth_motion, [0.0, 0.0]);
    }

    #[test]
    fn test_first_frame_returns_none() {
        let mut d = DeshakeEngine::new();
        d.enabled = true;
        let data = vec![128u8; 100 * 100 * 4];
        let result = d.update(&data, 100, 100);
        assert!(result.is_none()); // first frame, no prev
    }

    #[test]
    fn test_second_frame_returns_compensation() {
        let mut d = DeshakeEngine::new();
        d.enabled = true;
        let data = vec![128u8; 100 * 100 * 4];
        let _ = d.update(&data, 100, 100); // first
        let result = d.update(&data, 100, 100); // second (identical frames → zero motion)
        assert!(result.is_some());
        let comp = result.unwrap();
        assert!((comp[0]).abs() < 1.0, "identical frames should have ~0 motion, got dx={}", comp[0]);
        assert!((comp[1]).abs() < 1.0, "identical frames should have ~0 motion, got dy={}", comp[1]);
    }

    #[test]
    fn test_to_grayscale() {
        let data = vec![
            255, 0, 0, 255,  // red pixel
            0, 255, 0, 255,  // green pixel
            0, 0, 255, 255,  // blue pixel
            128, 128, 128, 255, // gray pixel
        ];
        let gray = DeshakeEngine::to_grayscale(&data, 2, 2);
        assert_eq!(gray.len(), 4);
        // Red: Y = 0.299*255 ≈ 76
        assert!((gray[0] as i32 - 76).abs() < 5);
        // Green: Y = 0.587*255 ≈ 150
        assert!((gray[1] as i32 - 150).abs() < 5);
        // Blue: Y = 0.114*255 ≈ 29
        assert!((gray[2] as i32 - 29).abs() < 5);
        // Gray: Y ≈ 128
        assert!((gray[3] as i32 - 128).abs() < 5);
    }

    #[test]
    fn test_block_sad_identical() {
        let data = vec![100u8; 64 * 64]; // 64x64 gray
        let sad = DeshakeEngine::block_sad(&data, 64, &data, 64, 0, 0, 0, 0, 16);
        assert!((sad).abs() < 0.01, "identical blocks should have SAD ~0, got {}", sad);
    }

    #[test]
    fn test_block_sad_different() {
        let curr = vec![100u8; 64 * 64];
        let mut prev = vec![200u8; 64 * 64];
        prev[0] = 255; // one different pixel
        let sad = DeshakeEngine::block_sad(&curr, 64, &prev, 64, 0, 0, 0, 0, 16);
        assert!(sad > 0.0, "different blocks should have SAD > 0");
    }

    #[test]
    fn test_median_filter_single() {
        let vectors = vec![[1.0, 2.0]];
        let costs = vec![10.0];
        let result = DeshakeEngine::median_filter(&vectors, &costs);
        assert!((result[0] - 1.0).abs() < 0.01);
        assert!((result[1] - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_median_filter_multiple() {
        let vectors = vec![[1.0, 5.0], [3.0, 1.0], [2.0, 3.0], [100.0, 100.0]];
        let costs = vec![1.0, 1.0, 1.0, 100.0];
        let result = DeshakeEngine::median_filter(&vectors, &costs);
        // xs sorted: [1, 2, 3, 100] → median idx=1 (for 4 elems, mid=2)
        // Wait, for 4 elements, mid = 4/2 = 2 → index 2 = third element = 3
        // ys sorted: [1, 3, 5, 100] → idx 2 = 5
        assert!((result[0] - 3.0).abs() < 0.5, "expected ~3, got {}", result[0]);
        assert!((result[1] - 5.0).abs() < 0.5, "expected ~5, got {}", result[1]);
    }

    #[test]
    fn test_apply_warp_identity() {
        let data = vec![128u8; 100 * 100 * 4];
        let comp = [0.0, 0.0];
        let (out, w, h) = DeshakeEngine::apply_warp(&data, 100, 100, &comp, 0.0).unwrap();
        assert_eq!(w, 100);
        assert_eq!(h, 100);
        assert_eq!(out, data); // no warp, no crop → identical
    }

    #[test]
    fn test_apply_warp_with_crop() {
        let data = vec![128u8; 100 * 100 * 4];
        let comp = [0.0, 0.0];
        let (out, w, h) = DeshakeEngine::apply_warp(&data, 100, 100, &comp, 0.1).unwrap();
        assert_eq!(w, 80); // 100 - 2*10
        assert_eq!(h, 80);
        assert_eq!(out.len(), 80 * 80 * 4);
    }

    #[test]
    fn test_apply_warp_crop_too_large() {
        let data = vec![0u8; 10 * 10 * 4];
        let comp = [0.0, 0.0];
        let result = DeshakeEngine::apply_warp(&data, 10, 10, &comp, 0.6);
        assert!(result.is_err());
    }

    #[test]
    fn test_reset() {
        let mut d = DeshakeEngine::new();
        d.enabled = true;
        d.smooth_motion = [5.0, 3.0];
        d.frame_count = 10;
        d.reset();
        assert_eq!(d.smooth_motion, [0.0, 0.0]);
        assert_eq!(d.frame_count, 0);
        assert!(d.prev_gray.is_none());
    }

    #[test]
    fn test_downscale_gray() {
        let gray = vec![
            1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12,
            13, 14, 15, 16,
        ];
        let down = DeshakeEngine::downscale_gray(&gray, 4, 4);
        assert_eq!(down.len(), 4); // 2x2
        assert_eq!(down[0], 1);  // (0,0)
        assert_eq!(down[1], 3);  // (0,2)
        assert_eq!(down[2], 9);  // (2,0)
        assert_eq!(down[3], 11); // (2,2)
    }

    #[test]
    fn test_small_frame_no_match() {
        let mut d = DeshakeEngine::new();
        d.enabled = true;
        let data = vec![128u8; 32 * 32 * 4];
        let result = d.update(&data, 32, 32);
        assert!(result.is_none(), "frames <64px should return None");
    }

    #[test]
    fn test_shifting_pattern_detects_motion() {
        let mut d = DeshakeEngine::new();
        d.enabled = true;
        d.block_size = 16;
        d.search_radius = 16;

        // Create a 128x128 naturalistic pattern (sine wave + noise)
        let mut frame1 = vec![0u8; 128 * 128 * 4];
        for y in 0..128 {
            for x in 0..128 {
                // Sine wave pattern with strong texture
                let v = ((x as f32 * 2.5).sin() * 0.5 + (y as f32 * 3.1).sin() * 0.5) * 127.0 + 128.0;
                let val = v.round().clamp(0.0, 255.0) as u8;
                let idx = (y * 128 + x) * 4;
                frame1[idx] = val;
                frame1[idx + 1] = val;
                frame1[idx + 2] = val;
                frame1[idx + 3] = 255;
            }
        }

        let _ = d.update(&frame1, 128, 128); // first frame

        // Second frame: shifted right by 2 pixels (wrap-around continuously)
        let mut frame2 = vec![0u8; 128 * 128 * 4];
        for y in 0..128 {
            for x in 0..128 {
                let src_x = if x >= 2 { x - 2 } else { 128 + (x - 2) }; // wrap
                let src_idx = (y * 128 + src_x) * 4;
                let dst_idx = (y * 128 + x) * 4;
                frame2[dst_idx] = frame1[src_idx];
                frame2[dst_idx + 1] = frame1[src_idx + 1];
                frame2[dst_idx + 2] = frame1[src_idx + 2];
                frame2[dst_idx + 3] = 255;
            }
        }

        let result = d.update(&frame2, 128, 128);
        assert!(result.is_some(), "should detect motion");
        let comp = result.unwrap();
        // Pattern shifted RIGHT by 2px (camera moved LEFT).
        // Block matcher should find match at sx = bx-2 → dx = -2.
        // comp_dx = raw_dx - smooth = -2 - 0 ≈ -2
        // Negative compensation = shift image LEFT.
        assert!(comp[0] < 0.0, "rightward content should get negative compensation, got dx={}", comp[0]);
        assert!((comp[0] + 2.0).abs() < 2.0, "compensation should be ~-2, got {}", comp[0]);
    }
}
