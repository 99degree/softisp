//! Deshake — software-based video stabilization via block-matching motion estimation.
//!
//! Uses frame-to-frame block matching to compute a global motion vector (translation)
//! between consecutive frames, then applies inverse warp with trajectory smoothing.
//!
//! # Algorithm
//!
//! 1. **Frame buffer** — keep previous frame for motion estimation
//! 2. **Pyramid downscale** — downsample large frames for efficient matching
//! 3. **Block matching** — divide frame into N×M blocks, for each block search
//!    best match in previous frame via SAD (Sum of Absolute Differences)
//! 4. **Diamond search** — logarithmic 3-step search (33 SADs vs 1089 for full scan)
//! 5. **Sub-pixel refinement** — parabolic interpolation along X/Y axes
//! 6. **Motion vector aggregation** — cost-weighted median of all block vectors
//! 7. **EMA trajectory smoothing** — separate intentional motion from jitter
//! 8. **Warp** — apply inverse compensation with bilinear interpolation + crop
//!
//! Unlike EIS (which requires gyroscope hardware), deshake works on any
//! platform with only a camera feed. Trade-off: higher CPU usage.

/// Default block size for motion estimation (pixels per side at original resolution).
const BLOCK_SIZE: u32 = 32;

/// Search radius around each block's position (pixels at original resolution).
const SEARCH_RADIUS: i32 = 16;

/// EMA smoothing alpha for intentional motion. Lower = smoother but more lag.
const MOTION_SMOOTHING_ALPHA: f32 = 0.08;

/// Fraction of frame cropped to hide black borders from warp (0..1).
const CROP_FRACTION: f32 = 0.08;

/// Maximum pixel shift per axis (clamp to avoid excessive crop).
const MAX_SHIFT_PX: f32 = 64.0;

/// Downscale target: largest dimension after downscale (pixels).
/// e.g. 384 = downscale 4K to 384×216 for matching.
const DOWNSCALE_TARGET: u32 = 384;

/// Variance threshold for texture detection. Blocks below this are skipped.
const TEXTURE_VARIANCE_THRESHOLD: f32 = 10.0;

/// Minimum frame dimension for deshake to operate.
const MIN_FRAME_DIM: u32 = 64;

/// Motion estimation result for a single block.
#[derive(Debug, Clone, Copy)]
struct BlockMatch {
    dx: f32,
    dy: f32,
    cost: f32,
}

/// Deshake engine state.
#[derive(Debug, Clone)]
pub struct DeshakeEngine {
    /// Previous frame data (grayscale, may be downscaled).
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
    /// Downscale target override (0 = auto).
    pub downscale_target: u32,
    /// Use diamond search instead of full scan (default true).
    pub use_diamond_search: bool,
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
            downscale_target: 0,
            use_diamond_search: true,
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
        for (i, chunk) in data.chunks_exact(4).enumerate() {
            if i >= len { break; }
            let r = chunk[0] as f32;
            let g = chunk[1] as f32;
            let b = chunk[2] as f32;
            gray[i] = (0.299 * r + 0.587 * g + 0.114 * b).round() as u8;
        }
        gray
    }

    /// Downscale grayscale by factor 2 (nearest-neighbor).
    fn downscale_gray(gray: &[u8], w: u32, h: u32) -> Vec<u8> {
        let nw = w / 2;
        let nh = h / 2;
        let mut out = vec![0u8; (nw * nh) as usize];
        for y in 0..nh {
            for x in 0..nw {
                let src = (y * 2 * w + x * 2) as usize;
                out[(y * nw + x) as usize] = gray[src];
            }
        }
        out
    }

    /// Compute downscale factor to bring largest dimension ≤ target.
    fn compute_downscale_factor(width: u32, height: u32, target: u32) -> u32 {
        let max_dim = width.max(height);
        if max_dim <= target {
            return 1;
        }
        let mut factor = 1;
        while factor * 2 <= 8 && (max_dim / (factor * 2)) > target {
            factor *= 2;
        }
        factor
    }

    /// Compute SAD between two blocks, normalized by block area.
    fn block_sad(
        curr: &[u8], cw: u32,
        prev: &[u8], pw: u32,
        cx: u32, cy: u32,
        sx: i32, sy: i32,
        size: u32,
    ) -> f32 {
        let mut sad = 0.0f32;
        for dy in 0..size {
            for dx in 0..size {
                let cu = ((cy + dy) * cw + (cx + dx)) as usize;
                let pu = ((sy + dy as i32) * pw as i32 + (sx + dx as i32)) as usize;
                let a = curr.get(cu).copied().unwrap_or(0) as f32;
                let b = prev.get(pu).copied().unwrap_or(0) as f32;
                sad += (a - b).abs();
            }
        }
        sad / (size * size) as f32
    }

    /// Compute variance of a block (texture metric). Low = flat region.
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

    /// 3-step logarithmic diamond search for motion estimation.
    /// Checks at most 1 + 8*log2(search_radius) positions instead of
    /// (2*radius+1)^2 for full scan. Typically 25-33 SAD ops vs 1089.
    fn diamond_search(
        curr_gray: &[u8], cw: u32,
        prev_gray: &[u8], pw: u32, ph: u32,
        bx: u32, by: u32,
        block_size: u32,
        search_radius: i32,
    ) -> Option<BlockMatch> {
        let cx = bx as i32;
        let cy = by as i32;
        let max_x = (pw as i32) - block_size as i32;
        let max_y = (ph as i32) - block_size as i32;

        // Initialize at current position (no motion)
        let zero_cost = Self::block_sad(curr_gray, cw, prev_gray, pw,
                                         bx, by, cx, cy, block_size);
        let mut best_dx = 0i32;
        let mut best_dy = 0i32;
        let mut best_cost = zero_cost;

        // 3-step search: start with biggest step, halve each iteration
        let mut step = (search_radius as u32).next_power_of_two() as i32 / 2;
        while step >= 1 {
            let mut improved = false;
            // Check 4 cardinal directions
            for (dsx, dsy) in &[(step, 0), (-step, 0), (0, step), (0, -step)] {
                let sx = cx + best_dx + dsx;
                let sy = cy + best_dy + dsy;
                if sx < 0 || sy < 0 || sx > max_x || sy > max_y {
                    continue;
                }
                let cost = Self::block_sad(curr_gray, cw, prev_gray, pw,
                                            bx, by, sx, sy, block_size);
                if cost < best_cost {
                    best_cost = cost;
                    best_dx += dsx;
                    best_dy += dsy;
                    improved = true;
                }
            }
            if !improved {
                // Try 4 diagonal directions if cardinal didn't help
                for (dsx, dsy) in &[(step, step), (step, -step), (-step, step), (-step, -step)] {
                    let sx = cx + best_dx + dsx;
                    let sy = cy + best_dy + dsy;
                    if sx < 0 || sy < 0 || sx > max_x || sy > max_y {
                        continue;
                    }
                    let cost = Self::block_sad(curr_gray, cw, prev_gray, pw,
                                                bx, by, sx, sy, block_size);
                    if cost < best_cost {
                        best_cost = cost;
                        best_dx += dsx;
                        best_dy += dsy;
                    }
                }
            }
            step /= 2;
        }

        // Final 1-pixel refinement (check all 8 neighbors)
        for dsy in -1i32..=1 {
            for dsx in -1i32..=1 {
                if dsx == 0 && dsy == 0 { continue; }
                let sx = cx + best_dx + dsx;
                let sy = cy + best_dy + dsy;
                if sx < 0 || sy < 0 || sx > max_x || sy > max_y {
                    continue;
                }
                let cost = Self::block_sad(curr_gray, cw, prev_gray, pw,
                                            bx, by, sx, sy, block_size);
                if cost < best_cost {
                    best_cost = cost;
                    best_dx += dsx;
                    best_dy += dsy;
                }
            }
        }

        Some(BlockMatch {
            dx: best_dx as f32,
            dy: best_dy as f32,
            cost: best_cost,
        })
    }

    /// Full search (exhaustive). Used as fallback or when diamond search disabled.
    fn full_search(
        curr_gray: &[u8], cw: u32,
        prev_gray: &[u8], pw: u32, ph: u32,
        bx: u32, by: u32,
        block_size: u32,
        search_radius: i32,
    ) -> Option<BlockMatch> {
        let cx = bx as i32;
        let cy = by as i32;

        let start_x = (cx - search_radius).max(0).min((pw as i32) - block_size as i32);
        let start_y = (cy - search_radius).max(0).min((ph as i32) - block_size as i32);
        let end_x = (cx + search_radius).max(0).min((pw as i32) - block_size as i32);
        let end_y = (cy + search_radius).max(0).min((ph as i32) - block_size as i32);

        if start_x >= end_x || start_y >= end_y {
            return None;
        }

        let mut best = BlockMatch { dx: 0.0, dy: 0.0, cost: f32::MAX };

        // Initialize at current position (no motion)
        best.cost = Self::block_sad(curr_gray, cw, prev_gray, pw,
                                     bx, by, cx, cy, block_size);

        for sy in start_y..=end_y {
            for sx in start_x..=end_x {
                let cost = Self::block_sad(curr_gray, cw, prev_gray, pw,
                                            bx, by, sx, sy, block_size);
                if cost < best.cost {
                    best.cost = cost;
                    best.dx = (sx - cx) as f32;
                    best.dy = (sy - cy) as f32;
                }
            }
        }
        Some(best)
    }

    /// Find best match for a block using configured search method.
    fn find_best_match(
        curr_gray: &[u8], cw: u32,
        prev_gray: &[u8], pw: u32, ph: u32,
        bx: u32, by: u32,
        block_size: u32,
        search_radius: i32,
        use_diamond: bool,
    ) -> Option<BlockMatch> {
        let mut best = if use_diamond {
            Self::diamond_search(curr_gray, cw, prev_gray, pw, ph,
                                  bx, by, block_size, search_radius)
        } else {
            Self::full_search(curr_gray, cw, prev_gray, pw, ph,
                               bx, by, block_size, search_radius)
        }?;

        // Parabolic sub-pixel refinement (2D)
        if best.cost < f32::MAX / 2.0 {
            let int_sx = bx as i32 + best.dx as i32;
            let int_sy = by as i32 + best.dy as i32;
            let sub_dx = Self::parabolic_refine(curr_gray, cw, prev_gray, pw,
                                                 bx, by, int_sx, int_sy,
                                                 block_size, true);
            let sub_dy = Self::parabolic_refine(curr_gray, cw, prev_gray, pw,
                                                 bx, by, int_sx, int_sy,
                                                 block_size, false);
            best.dx += sub_dx;
            best.dy += sub_dy;
        }

        Some(best)
    }

    /// Parabolic sub-pixel refinement along one axis.
    fn parabolic_refine(
        curr_gray: &[u8], cw: u32,
        prev_gray: &[u8], pw: u32,
        bx: u32, by: u32,
        best_sx: i32, best_sy: i32,
        block_size: u32,
        horizontal: bool,
    ) -> f32 {
        let max_sx = (pw as i32) - block_size as i32;
        let ph = (prev_gray.len() / pw as usize) as i32;
        let max_sy = ph - block_size as i32;

        if horizontal {
            if best_sx - 1 < 0 || best_sx + 1 > max_sx { return 0.0; }
        } else {
            if best_sy - 1 < 0 || best_sy + 1 > max_sy { return 0.0; }
        }

        let c0 = Self::block_sad(curr_gray, cw, prev_gray, pw,
                                  bx, by, best_sx, best_sy, block_size);
        let (c1, c2) = if horizontal {
            (Self::block_sad(curr_gray, cw, prev_gray, pw, bx, by,
                              best_sx - 1, best_sy, block_size),
             Self::block_sad(curr_gray, cw, prev_gray, pw, bx, by,
                              best_sx + 1, best_sy, block_size))
        } else {
            (Self::block_sad(curr_gray, cw, prev_gray, pw, bx, by,
                              best_sx, best_sy - 1, block_size),
             Self::block_sad(curr_gray, cw, prev_gray, pw, bx, by,
                              best_sx, best_sy + 1, block_size))
        };

        let denom = c1 - 2.0 * c0 + c2;
        if denom.abs() < 1e-8 { return 0.0; }
        (c1 - c2) / (2.0 * denom)
    }

    /// Per-frame deshake update with pyramid downscale + diamond search.
    ///
    /// Takes current BGRA/RGBA frame, returns compensation vector [dx, dy].
    pub fn update(
        &mut self,
        data: &[u8],
        width: u32,
        height: u32,
    ) -> Option<[f32; 2]> {
        if !self.enabled || width < MIN_FRAME_DIM || height < MIN_FRAME_DIM {
            return None;
        }

        let block_size = if self.block_size > 0 { self.block_size } else { BLOCK_SIZE };
        let search_radius = if self.search_radius > 0 { self.search_radius } else { SEARCH_RADIUS };
        let alpha = if self.smoothing_alpha > 0.0 { self.smoothing_alpha } else { MOTION_SMOOTHING_ALPHA };
        let ds_target = if self.downscale_target > 0 { self.downscale_target } else { DOWNSCALE_TARGET };
        let use_diamond = self.use_diamond_search;

        // --- Pyramid downscale ---
        // Compute downscale factor and downsample both frames for matching
        let ds_factor = Self::compute_downscale_factor(width, height, ds_target);
        let (curr_gray, ds_width, ds_height) = if ds_factor > 1 {
            let gray = Self::to_grayscale(data, width, height);
            // Apply downscale factor (powers of 2, repeated)
            let mut g = gray;
            let mut w = width;
            let mut h = height;
            for _ in 0..(ds_factor.ilog2()) {
                let prev_w = w;
                g = Self::downscale_gray(&g, prev_w, h);
                w /= 2;
                h /= 2;
            }
            (g, w, h)
        } else {
            (Self::to_grayscale(data, width, height), width, height)
        };

        // Scale block_size and search_radius to downsampled domain
        let ds_block_size = (block_size / ds_factor).max(4);
        let ds_search_radius = (search_radius / ds_factor as i32).max(2);

        // Load previous frame (already at same downscale level)
        let prev_gray = match self.prev_gray {
            Some(ref pg) => pg.clone(),
            None => {
                self.prev_gray = Some(curr_gray);
                self.prev_dims = Some((ds_width, ds_height));
                return None;
            }
        };
        let (prev_w, prev_h) = self.prev_dims.unwrap_or((ds_width, ds_height));

        // Safety: dimensions must match for block matching
        if prev_w != ds_width || prev_h != ds_height {
            // Resolution change — reset
            self.prev_gray = Some(curr_gray);
            self.prev_dims = Some((ds_width, ds_height));
            return None;
        }

        // --- Motion estimation ---
        let num_blocks_x = (ds_width / ds_block_size).max(1);
        let num_blocks_y = (ds_height / ds_block_size).max(1);
        let mut xs = Vec::new();
        let mut ys = Vec::new();
        let mut weights = Vec::new();

        for by_idx in 0..num_blocks_y {
            for bx_idx in 0..num_blocks_x {
                let bx = bx_idx * ds_block_size;
                let by = by_idx * ds_block_size;

                // Skip low-texture blocks
                let var = Self::block_variance(&curr_gray, ds_width, bx, by, ds_block_size);
                if var < TEXTURE_VARIANCE_THRESHOLD { continue; }

                if let Some(mv) = Self::find_best_match(
                    &curr_gray, ds_width,
                    &prev_gray, prev_w, prev_h,
                    bx, by, ds_block_size, ds_search_radius, use_diamond,
                ) {
                    // Scale motion vector back to original resolution
                    xs.push(mv.dx * ds_factor as f32);
                    ys.push(mv.dy * ds_factor as f32);
                    // Weight = inverse of normalized SAD (lower cost = higher weight)
                    weights.push(1.0 / (mv.cost + 0.1));
                }
            }
        }

        if xs.is_empty() {
            self.prev_gray = Some(curr_gray);
            self.prev_dims = Some((ds_width, ds_height));
            self.frame_count += 1;
            return Some(self.smooth_motion);
        }

        // --- Cost-weighted median ---
        let raw_dx = Self::weighted_median(&xs, &weights);
        let raw_dy = Self::weighted_median(&ys, &weights);

        // Clamp to maximum shift
        let raw_dx = raw_dx.clamp(-MAX_SHIFT_PX, MAX_SHIFT_PX);
        let raw_dy = raw_dy.clamp(-MAX_SHIFT_PX, MAX_SHIFT_PX);

        // EMA trajectory smoothing
        self.smooth_motion[0] += (raw_dx - self.smooth_motion[0]) * alpha;
        self.smooth_motion[1] += (raw_dy - self.smooth_motion[1]) * alpha;

        // Jitter = raw - smooth (high-frequency component to cancel)
        let comp_dx = raw_dx - self.smooth_motion[0];
        let comp_dy = raw_dy - self.smooth_motion[1];

        // Clamp compensation to crop margin
        let margin_x = (width as f32 * CROP_FRACTION).max(1.0);
        let margin_y = (height as f32 * CROP_FRACTION).max(1.0);
        let clamped_dx = comp_dx.clamp(-margin_x, margin_x);
        let clamped_dy = comp_dy.clamp(-margin_y, margin_y);

        // Store current (downsampled) frame for next iteration
        self.prev_gray = Some(curr_gray);
        self.prev_dims = Some((ds_width, ds_height));
        self.frame_count += 1;

        if self.debug {
            log::debug!(
                "Deshake: raw=[{:.1},{:.1}] smooth=[{:.1},{:.1}] comp=[{:.1},{:.1}] \
                 blocks={} ds_factor={}",
                raw_dx, raw_dy, self.smooth_motion[0], self.smooth_motion[1],
                clamped_dx, clamped_dy, xs.len(), ds_factor,
            );
        }

        Some([clamped_dx, clamped_dy])
    }

    /// Weighted median of a sorted or sortable array.
    /// Values with higher weight contribute more to the median.
    fn weighted_median(values: &[f32], weights: &[f32]) -> f32 {
        if values.is_empty() {
            return 0.0;
        }
        if values.len() == 1 {
            return values[0];
        }

        // Pair values with weights and sort by value
        let mut pairs: Vec<(f32, f32)> = values.iter().copied()
            .zip(weights.iter().copied())
            .collect();
        pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let total_weight: f32 = pairs.iter().map(|(_, w)| w).sum();
        let half = total_weight / 2.0;

        let mut cum = 0.0f32;
        for (val, w) in &pairs {
            cum += w;
            if cum >= half {
                return *val;
            }
        }
        pairs.last().map(|(v, _)| *v).unwrap_or(0.0)
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

                // Inverse warp: map output pixel to source pixel
                let sx = ix + dx;  // positive dx = shift image LEFT
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
// Bilinear interpolation (4-channel)
// ============================================================================

fn bilinear_sample_4(data: &[u8], width: u32, height: u32, x: f32, y: f32) -> Vec<u8> {
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    let x1 = x0 + 1;
    let y1 = y0 + 1;

    let fx = x - x0 as f32;
    let fy = y - y0 as f32;

    let sample = |sx: i32, sy: i32| -> [u8; 4] {
        if sx < 0 || sx >= width as i32 || sy < 0 || sy >= height as i32 {
            return [0u8; 4];
        }
        let idx = ((sy as u32 * width + sx as u32) * 4) as usize;
        let mut p = [0u8; 4];
        p.copy_from_slice(&data[idx..idx + 4]);
        p
    };

    let p00 = sample(x0, y0);
    let p10 = sample(x1, y0);
    let p01 = sample(x0, y1);
    let p11 = sample(x1, y1);

    let mut result = [0u8; 4];
    for c in 0..4 {
        let v = p00[c] as f32 * (1.0 - fx) * (1.0 - fy)
            + p10[c] as f32 * fx * (1.0 - fy)
            + p01[c] as f32 * (1.0 - fx) * fy
            + p11[c] as f32 * fx * fy;
        result[c] = v.round().clamp(0.0, 255.0) as u8;
    }
    result.to_vec()
}

// ============================================================================
// GPU-accelerated Deshake Pipeline (hybrid GPU/CPU)
// ============================================================================
// Uses GrayscaleBlock + PyramidBlock as a lightweight MNN model that runs
// on Vulkan GPU. The GPU produces a downscaled grayscale image, then the
// CPU block-matching engine runs on the much smaller resolution.
//
// Pipeline: Input[1,3,H,W] → GrayscaleBlock[1,1,H,W] → PyramidBlock[1,1,H/2,W/2]
// IspChainFusion detects R7+R8 fusing into isp.grayscale + isp.pyramid Extra ops.
//
// Requires feature "mnn" for the MNN GPU backend.

#[cfg(feature = "mnn")]
mod gpu_pipeline {
    use crate::pipeline::{IspBlock, GraphComposer};
    use crate::blocks::{GrayscaleBlock, PyramidBlock};
    use crate::mnn_sys::{MnnInterpreterSafe, MnnBackendType, MnnTensorSafe};
    use std::sync::Mutex;

    /// Cached GPU pipeline for grayscale+pyramid.
    /// Lazily built from ONNX at first use.
    pub struct DeshakeGpuPipeline {
        /// Cached MNN model bytes for current resolution.
        mnn_cache: Mutex<Option<(u32, u32, Vec<u8>)>>,
    }

    impl DeshakeGpuPipeline {
        pub fn new() -> Self {
            Self {
                mnn_cache: Mutex::new(None),
            }
        }

        /// Build ONNX for given resolution, convert to MNN, and run inference.
        /// Returns (downscaled_grayscale, ds_width, ds_height).
        pub fn run(
            &self,
            rgb_data: &[f32],  // [1,3,H,W] planar float32
            width: u32,
            height: u32,
        ) -> Result<(Vec<u8>, u32, u32), String> {
            let ds_w = width / 2;
            let ds_h = height / 2;
            let out_len = (ds_w * ds_h) as usize;

            // Check cache
            let mnn_bytes = {
                let mut cache = self.mnn_cache.lock().unwrap();
                if let Some((cw, ch, ref data)) = *cache {
                    if cw == width && ch == height {
                        data.clone()
                    } else {
                        // Resolution changed — rebuild
                        *cache = None;
                        drop(cache);
                        let data = Self::build_mnn_model(width, height)?;
                        let mut cache = self.mnn_cache.lock().unwrap();
                        *cache = Some((width, height, data.clone()));
                        data
                    }
                } else {
                    drop(cache);
                    let data = Self::build_mnn_model(width, height)?;
                    let mut cache = self.mnn_cache.lock().unwrap();
                    *cache = Some((width, height, data.clone()));
                    data
                }
            };

            // Create interpreter and run
            let interpreter = MnnInterpreterSafe::from_buffer(&mnn_bytes)
                .ok_or_else(|| "Failed to create MNN interpreter".to_string())?;
            let session = interpreter.create_session(MnnBackendType::Vulkan, 1)
                .ok_or_else(|| "Failed to create MNN session".to_string())?;

            // Get input tensor
            let input = interpreter.get_first_input(&session)
                .ok_or_else(|| "Failed to get input tensor".to_string())?;
            
            // Write input data via mutable byte slice
            let input_size = (width * height * 3) as usize;
            {
                let mut input_bytes = input.as_bytes_mut()
                    .ok_or_else(|| "Failed to get input tensor data".to_string())?;
                if input_bytes.len() < input_size * 4 {
                    return Err(format!("Input tensor too small: {} bytes, need {}",
                        input_bytes.len(), input_size * 4));
                }
                // Cast byte slice to float slice and copy
                unsafe {
                    let dst = input_bytes.as_mut_ptr() as *mut f32;
                    std::ptr::copy_nonoverlapping(rgb_data.as_ptr(), dst, input_size);
                }
            }

            // Resize and run
            session.resize().map_err(|e| format!("Session resize failed: {}", e))?;
            session.run().map_err(|e| format!("Session run failed: {}", e))?;

            // Get output
            let output = interpreter.get_first_output(&session)
                .ok_or_else(|| "Failed to get output tensor".to_string())?;

            // Read output data as float32 (luminance, single channel)
            let output_size = out_len;  // [1,1,H/2,W/2]
            let mut out_data = vec![0u8; output_size];
            {
                let output_bytes = output.as_bytes()
                    .ok_or_else(|| "Failed to get output tensor data".to_string())?;
                if output_bytes.len() < output_size * 4 {
                    return Err(format!("Output tensor too small: {} bytes, need {}",
                        output_bytes.len(), output_size * 4));
                }
                // Cast byte slice to float slice and convert to u8
                unsafe {
                    let src = output_bytes.as_ptr() as *const f32;
                    for i in 0..output_size {
                        let v = (*src.add(i)) * 255.0;
                        out_data[i] = v.round().clamp(0.0, 255.0) as u8;
                    }
                }
            }

            Ok((out_data, ds_w, ds_h))
        }

        /// Build ONNX model and convert to MNN.
        fn build_mnn_model(width: u32, height: u32) -> Result<Vec<u8>, String> {
            use std::io::Write;
            
            // Build ONNX graph with GrayscaleBlock → PyramidBlock
            let mut gs = GrayscaleBlock::new();
            let mut pyr = PyramidBlock::new();
            let mut blocks: Vec<Box<dyn IspBlock>> = vec![
                Box::new(gs),
                Box::new(pyr),
            ];
            GraphComposer::wire_blocks(&mut blocks);
            let onnx_proto = GraphComposer::compose(
                blocks[0].as_ref(), &[], 21
            )?;

            // Write ONNX to temp file
            let tmp_dir = std::env::temp_dir();
            let onnx_path = tmp_dir.join(format!("deshake_gpu_{}x{}.onnx", width, height));
            let mnn_path = tmp_dir.join(format!("deshake_gpu_{}x{}.mnn", width, height));

            let mut f = std::fs::File::create(&onnx_path)
                .map_err(|e| format!("Failed to create ONNX temp file: {}", e))?;
            f.write_all(&onnx_proto)
                .map_err(|e| format!("Failed to write ONNX: {}", e))?;
            drop(f);

            // Convert ONNX to MNN
            let onnx_cstr = std::ffi::CString::new(onnx_path.to_string_lossy().as_ref())
                .map_err(|_| "CString conversion failed".to_string())?;
            let mnn_cstr = std::ffi::CString::new(mnn_path.to_string_lossy().as_ref())
                .map_err(|_| "CString conversion failed".to_string())?;
            let biz = std::ffi::CString::new("deshake")
                .map_err(|_| "CString conversion failed".to_string())?;

            let mut result = crate::mnn_sys::MnnConvertResult {
                success: 0,
                error_msg: [0u8; 1024],
            };

            unsafe {
                crate::mnn_sys::mnn_convert_onnx_to_mnn(
                    onnx_cstr.as_ptr(),
                    mnn_cstr.as_ptr(),
                    biz.as_ptr(),
                    2,   // optimize level
                    0,   // no weight quant
                    0,   // no fp16
                    0,   // don't preserve input type
                    &mut result,
                );
            }

            if result.success != 0 {
                let err = unsafe {
                    std::ffi::CStr::from_ptr(result.error_msg.as_ptr())
                        .to_string_lossy().into_owned()
                };
                let _ = std::fs::remove_file(&onnx_path);
                return Err(format!("MNN conversion failed: {}", err));
            }

            // Read MNN file
            let mnn_bytes = std::fs::read(&mnn_path)
                .map_err(|e| format!("Failed to read MNN file: {}", e))?;

            // Cleanup temp files
            let _ = std::fs::remove_file(&onnx_path);
            let _ = std::fs::remove_file(&mnn_path);

            Ok(mnn_bytes)
        }
    }
} // mod gpu_pipeline

#[cfg(feature = "mnn")]
pub use gpu_pipeline::DeshakeGpuPipeline;

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
        assert!(result.is_none());
    }

    #[test]
    fn test_second_frame_returns_compensation() {
        let mut d = DeshakeEngine::new();
        d.enabled = true;
        let data = vec![128u8; 100 * 100 * 4];
        let _ = d.update(&data, 100, 100);
        let result = d.update(&data, 100, 100);
        assert!(result.is_some());
        let comp = result.unwrap();
        assert!((comp[0]).abs() < 1.0, "identical frames should have ~0 dx, got {}", comp[0]);
        assert!((comp[1]).abs() < 1.0, "identical frames should have ~0 dy, got {}", comp[1]);
    }

    #[test]
    fn test_to_grayscale() {
        let data = vec![
            255, 0, 0, 255,
            0, 255, 0, 255,
            0, 0, 255, 255,
            128, 128, 128, 255,
        ];
        let gray = DeshakeEngine::to_grayscale(&data, 2, 2);
        assert_eq!(gray.len(), 4);
        assert!((gray[0] as i32 - 76).abs() < 5);   // R
        assert!((gray[1] as i32 - 150).abs() < 5);  // G
        assert!((gray[2] as i32 - 29).abs() < 5);   // B
        assert!((gray[3] as i32 - 128).abs() < 5);  // gray
    }

    #[test]
    fn test_block_sad_identical() {
        let data = vec![100u8; 64 * 64];
        let sad = DeshakeEngine::block_sad(&data, 64, &data, 64, 0, 0, 0, 0, 16);
        assert!((sad).abs() < 0.01, "identical blocks should have SAD ~0, got {}", sad);
    }

    #[test]
    fn test_block_sad_different() {
        let curr = vec![100u8; 64 * 64];
        let mut prev = vec![200u8; 64 * 64];
        prev[0] = 255;
        let sad = DeshakeEngine::block_sad(&curr, 64, &prev, 64, 0, 0, 0, 0, 16);
        assert!(sad > 0.0);
    }

    #[test]
    fn test_weighted_median_single() {
        let vals = vec![42.0];
        let w = vec![1.0];
        let r = DeshakeEngine::weighted_median(&vals, &w);
        assert!((r - 42.0).abs() < 0.01);
    }

    #[test]
    fn test_weighted_median_three() {
        let vals = vec![1.0, 10.0, 100.0];
        let w = vec![1.0, 10.0, 1.0];
        let r = DeshakeEngine::weighted_median(&vals, &w);
        // sorted: (1,1), (10,10), (100,1); total=12, half=6
        // cum after (1,1)=1, (10,10)=11 ≥ 6 → median=10
        assert!((r - 10.0).abs() < 0.5, "expected ~10, got {}", r);
    }

    #[test]
    fn test_weighted_median_outlier_suppressed() {
        let vals = vec![0.0, 0.0, 100.0];
        let w = vec![10.0, 10.0, 0.1];
        let r = DeshakeEngine::weighted_median(&vals, &w);
        // sorted: (0,10), (0,10), (100,0.1); total=20.1, half=10.05
        // cum after first (0,10)=10 < 10.05, after second (0,10)=20.1 ≥ 10.05
        // median=0
        assert!((r - 0.0).abs() < 0.5, "expected ~0, got {}", r);
    }

    #[test]
    fn test_downscale_factor() {
        assert_eq!(DeshakeEngine::compute_downscale_factor(1920, 1080, 384), 4);
        assert_eq!(DeshakeEngine::compute_downscale_factor(3840, 2160, 384), 8);
        assert_eq!(DeshakeEngine::compute_downscale_factor(640, 480, 384), 1);
        assert_eq!(DeshakeEngine::compute_downscale_factor(200, 100, 384), 1);
        assert_eq!(DeshakeEngine::compute_downscale_factor(1280, 720, 384), 2);
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
        assert_eq!(down.len(), 4);
        assert_eq!(down[0], 1);
        assert_eq!(down[1], 3);
        assert_eq!(down[2], 9);
        assert_eq!(down[3], 11);
    }

    #[test]
    fn test_apply_warp_identity() {
        let data = vec![128u8; 100 * 100 * 4];
        let (out, w, h) = DeshakeEngine::apply_warp(&data, 100, 100, &[0.0, 0.0], 0.0).unwrap();
        assert_eq!(w, 100);
        assert_eq!(h, 100);
        assert_eq!(out, data);
    }

    #[test]
    fn test_apply_warp_with_crop() {
        let data = vec![128u8; 100 * 100 * 4];
        let (out, w, h) = DeshakeEngine::apply_warp(&data, 100, 100, &[0.0, 0.0], 0.1).unwrap();
        assert_eq!(w, 80);
        assert_eq!(h, 80);
        assert_eq!(out.len(), 80 * 80 * 4);
    }

    #[test]
    fn test_apply_warp_crop_too_large() {
        let data = vec![0u8; 10 * 10 * 4];
        let result = DeshakeEngine::apply_warp(&data, 10, 10, &[0.0, 0.0], 0.6);
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
    fn test_small_frame_no_match() {
        let mut d = DeshakeEngine::new();
        d.enabled = true;
        let result = d.update(&[0u8; 32 * 32 * 4], 32, 32);
        assert!(result.is_none());
    }

    #[test]
    fn test_shifting_pattern_detects_motion() {
        let mut d = DeshakeEngine::new();
        d.enabled = true;
        d.block_size = 16;
        d.search_radius = 16;
        d.use_diamond_search = true;

        // Create 128×128 sine wave pattern
        let mut frame1 = vec![0u8; 128 * 128 * 4];
        for y in 0..128 {
            for x in 0..128 {
                let v = ((x as f32 * 2.5).sin() * 0.5 + (y as f32 * 3.1).sin() * 0.5) * 127.0 + 128.0;
                let val = v.round().clamp(0.0, 255.0) as u8;
                let idx = (y * 128 + x) * 4;
                frame1[idx] = val;
                frame1[idx + 1] = val;
                frame1[idx + 2] = val;
                frame1[idx + 3] = 255;
            }
        }

        let _ = d.update(&frame1, 128, 128);
        let mut frame2 = frame1.clone();
        // Shift right by 2px (wrap)
        for y in 0..128 {
            for x in (0..128).rev() {
                let src_x = if x >= 2 { x - 2 } else { 126 + x };
                let idx = (y * 128 + x) * 4;
                let src_idx = (y * 128 + src_x) * 4;
                frame2[idx] = frame1[src_idx];
                frame2[idx + 1] = frame1[src_idx + 1];
                frame2[idx + 2] = frame1[src_idx + 2];
                frame2[idx + 3] = 255;
            }
        }

        let result = d.update(&frame2, 128, 128);
        assert!(result.is_some(), "should detect motion");
        let comp = result.unwrap();
        // Rightward shift → negative compensation
        assert!(comp[0] < 0.0, "rightward content -> negative comp, got {}", comp[0]);
        assert!((comp[0] + 2.0).abs() < 2.0, "comp ~-2, got {}", comp[0]);
    }

    #[test]
    fn test_diamond_vs_full_search_accuracy() {
        // Verify diamond search matches full search within tolerance
        let mut frame1 = vec![0u8; 64 * 64 * 4];
        for y in 0..64 {
            for x in 0..64 {
                let v = ((x as f32 * 1.7).sin() * 0.5 + (y as f32 * 2.3).sin() * 0.5) * 127.0 + 128.0;
                let val = v.round().clamp(0.0, 255.0) as u8;
                let idx = (y * 64 + x) * 4;
                frame1[idx] = val;
                frame1[idx + 1] = val;
                frame1[idx + 2] = val;
                frame1[idx + 3] = 255;
            }
        }
        let gray1 = DeshakeEngine::to_grayscale(&frame1, 64, 64);

        // Compare diamond vs full for a few blocks
        for by in [0u32, 16, 32] {
            for bx in [0u32, 16, 32] {
                let diamond = DeshakeEngine::diamond_search(
                    &gray1, 64, &gray1, 64, 64,
                    bx, by, 16, 8,
                ).unwrap();
                let full = DeshakeEngine::full_search(
                    &gray1, 64, &gray1, 64, 64,
                    bx, by, 16, 8,
                ).unwrap();
                assert!((diamond.dx - full.dx).abs() < 2.0,
                        "diamond.dx={} vs full.dx={} at ({},{})", diamond.dx, full.dx, bx, by);
                assert!((diamond.dy - full.dy).abs() < 2.0,
                        "diamond.dy={} vs full.dy={} at ({},{})", diamond.dy, full.dy, bx, by);
            }
        }
    }

    #[test]
    fn test_postprocess_integration() {
        use crate::postprocess::PostProcessPipeline;
        use crate::postprocess::PostProcessConfig;
        use crate::pipeline::IspFrame;

        let mut cfg = PostProcessConfig::default();
        cfg.deshake_enabled = true;
        cfg.deshake_block_size = 32;
        cfg.deshake_search_radius = 8;
        cfg.deshake_crop_fraction = 0.0;

        let mut pipeline = PostProcessPipeline::new(cfg);
        let data = vec![100u8; 192 * 108 * 4];
        let frame = IspFrame {
            data: data.clone(),
            width: 192,
            height: 108,
            format: cam_types::FrameFormat::Rgba8888,
            float_data: None,
            aux: None,
            timestamp_ns: 0,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
        };

        let result = pipeline.process(&frame);
        assert!(result.is_ok(), "pipeline should produce output: {:?}", result.err());
        let out = result.unwrap();
        assert_eq!(out.data.len(), 192 * 108 * 4);
    }
}
