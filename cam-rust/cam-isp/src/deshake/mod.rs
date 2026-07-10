//! # Deshake — Software Video Stabilization
//!
//! ## Overview
//!
//! Software-based video stabilization using block-matching motion estimation.
//! Computes per-frame translation vectors and applies inverse warp with
//! trajectory smoothing to remove camera shake.
//!
//! ## Integration with WarpGridBlock
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    Deshake Pipeline                            │
//! ├─────────────────────────────────────────────────────────────────┤
//! │                                                                 │
 //! │  Frame N-1 ─┐                                                  │
//! │              ├→ DeshakeEngine::estimate_motion()                │
//! │  Frame N ───┘    │                                              │
//! │                   ↓                                              │
//! │            Motion Vector (dx, dy)                               │
//! │                   ↓                                              │
//! │            Motion Smoothing (EMA filter)                        │
//! │                   ↓                                              │
//! │            Grid Generation (motion_to_grid())                   │
//! │                   ↓                                              │
//! │            WarpGridBlock (GPU: GridSample)                      │
//! │                   ↓                                              │
//! │            Stabilized Frame                                     │
//! │                                                                 │
//! └─────────────────────────────────────────────────────────────────┘
//!
//! ## Usage
//!
//! ```rust
//! use cam_isp::deshake::DeshakeEngine;
//! use cam_isp::blocks::WarpGridBlock;
//!
//! // CPU-based deshake
//! let mut deshake = DeshakeEngine::new(3840, 2160);
//! let motion = deshake.estimate_motion(&prev_frame, &curr_frame);
//! let grid = deshake.motion_to_grid(motion, 1920, 1080);
//! let warp = WarpGridBlock::new(1920, 1080).with_grid(Some(grid));
//!
//! // GPU-accelerated deshake (entire pipeline on GPU)
//! let gpu_deshake = DeshakeGpuPipeline::new();
//! ```
//!
//! ## Modules
//!
//! - `motion` — Block matching, diamond search, weighted median
//! - `warp` — Bilinear interpolation, crop, border handling
//! - `gpu_pipeline` — MNN-based GPU acceleration for entire pipeline

pub mod gpu_pipeline;
pub mod motion;
pub mod warp;

pub use gpu_pipeline::DeshakeGpuPipeline;
pub use motion::{BlockMatch, block_sad, block_variance, diamond_search, full_search, parabolic_refine, weighted_median};
pub use warp::{apply_warp_rgb_f32, bgra_to_planar_rgb_f32, compute_downscale_factor, downscale_gray, to_grayscale};

/// Default block size for motion estimation (pixels per side at original resolution).
const BLOCK_SIZE: u32 = 32;

/// Search radius around each block's position (pixels at original resolution).
const SEARCH_RADIUS: i32 = 16;

/// EMA smoothing alpha for intentional motion. Lower = smoother but more lag.
const MOTION_SMOOTHING_ALPHA: f32 = 0.08;

/// Fraction of frame cropped to hide black borders from warp (0..1).
const CROP_FRACTION: f32 = 0.08;

#[allow(dead_code)]
/// Maximum pixel shift per axis (clamp to avoid excessive crop).
const MAX_SHIFT_PX: f32 = 64.0;

/// Downscale target: largest dimension after downscale (pixels).
/// e.g. 384 = downscale 4K to 384×216 for matching.
const DOWNSCALE_TARGET: u32 = 384;

/// Variance threshold for texture detection. Blocks below this are skipped.
const TEXTURE_VARIANCE_THRESHOLD: f32 = 10.0;

/// Minimum frame dimension for deshake to operate.
const MIN_FRAME_DIM: u32 = 64;

/// Deshake engine state.
#[derive(Debug, Clone)]
pub struct DeshakeEngine {
    /// Previous frame data (grayscale, may be downscaled).
    prev_gray: Option<Vec<u8>>,
    /// Previous frame dimensions after optional downscale.
    prev_dims: Option<(u32, u32)>,
    /// Smoothed accumulated motion vector `[dx, dy]`.
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
    /// Optional GPU pipeline for grayscale+pyramid.
    /// When Some, grayscale conversion and downscale run on GPU (Vulkan).
    pub gpu_pipeline: Option<DeshakeGpuPipeline>,
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
            gpu_pipeline: None,
        }
    }

    pub fn reset(&mut self) {
        self.prev_gray = None;
        self.prev_dims = None;
        self.smooth_motion = [0.0; 2];
        self.frame_count = 0;
    }

    pub fn with_gpu_pipeline(&mut self, use_gpu: bool) -> &mut Self {
        if use_gpu {
            self.gpu_pipeline = Some(DeshakeGpuPipeline::new());
        } else {
            self.gpu_pipeline = None;
        }
        self
    }

    fn effective_block_size(&self) -> u32 {
        if self.block_size > 0 { self.block_size } else { BLOCK_SIZE }
    }

    fn effective_search_radius(&self) -> i32 {
        if self.search_radius > 0 { self.search_radius } else { SEARCH_RADIUS }
    }

    fn effective_smoothing_alpha(&self) -> f32 {
        if self.smoothing_alpha > 0.0 { self.smoothing_alpha } else { MOTION_SMOOTHING_ALPHA }
    }

    fn effective_downscale_target(&self) -> u32 {
        if self.downscale_target > 0 { self.downscale_target } else { DOWNSCALE_TARGET }
    }

    /// Process a frame and return compensated output.
    ///
    /// `data` — BGRA or RGBA interleaved bytes.
    /// Returns `None` on first frame (no reference), `Some((output, w, h))` thereafter.
    pub fn update(&mut self, data: &[u8], width: u32, height: u32) -> Option<(Vec<u8>, u32, u32)> {
        if !self.enabled || width < MIN_FRAME_DIM || height < MIN_FRAME_DIM {
            return None;
        }

        // Convert to grayscale
        let ds_factor = self.effective_downscale_target();
        let (gray, gw, gh) = if let Some(ref gpu) = self.gpu_pipeline {
            // GPU path: use GPU for grayscale + pyramid
            let planar = bgra_to_planar_rgb_f32(data, width, height);
            if let Ok((small, sw, sh)) = gpu.run(&planar, width, height) {
                (small, sw, sh)
            } else {
                // Fallback to CPU
                let g = to_grayscale(data, width, height);
                let ds = compute_downscale_factor(width, height, ds_factor);
                if ds > 1 {
                    let mut small = g;
                    for _ in 0..ds.ilog2() {
                        let prev_w = small.len() as u32 / height;
                        small = downscale_gray(&small, prev_w, height);
                    }
                    (small, width / ds, height / ds)
                } else {
                    (g, width, height)
                }
            }
        } else {
            // CPU path
            let g = to_grayscale(data, width, height);
            let ds = compute_downscale_factor(width, height, ds_factor);
            if ds > 1 {
                let mut small = g;
                let mut w = width;
                let mut h = height;
                for _ in 0..ds.ilog2() {
                    let prev_w = w;
                    small = downscale_gray(&small, prev_w, h);
                    w /= 2;
                    h /= 2;
                }
                (small, w, h)
            } else {
                (g, width, height)
            }
        };

        // If we have a previous frame, compute motion
        let has_prev = self.prev_gray.is_some();
        let (global_dx, global_dy) = if let (Some(ref prev), Some((pw, ph))) = (&self.prev_gray, self.prev_dims) {
            self.compute_motion(prev, pw, ph, &gray, gw, gh)
        } else {
            (0.0, 0.0)
        };

        // Store current frame as reference
        self.prev_gray = Some(gray);
        self.prev_dims = Some((gw, gh));
        self.frame_count += 1;

        // Return None on first frame (no motion to compensate)
        if !has_prev {
            return None;
        }

        // Smooth motion with EMA
        let alpha = self.effective_smoothing_alpha();
        let raw = [global_dx, global_dy];
        let smooth = &mut self.smooth_motion;
        smooth[0] += (raw[0] - smooth[0]) * alpha;
        smooth[1] += (raw[1] - smooth[1]) * alpha;

        // Apply warp
        let planar = bgra_to_planar_rgb_f32(data, width, height);
        let warped = apply_warp_rgb_f32(&planar, width, height, smooth[0], smooth[1], CROP_FRACTION);

        // Convert back to BGRA
        let n = (width * height) as usize;
        let mut output = vec![0u8; n * 4];
        for i in 0..n {
            output[i * 4] = (warped[2 * n + i] * 255.0).clamp(0.0, 255.0) as u8;
            output[i * 4 + 1] = (warped[n + i] * 255.0).clamp(0.0, 255.0) as u8;
            output[i * 4 + 2] = (warped[i] * 255.0).clamp(0.0, 255.0) as u8;
            output[i * 4 + 3] = 255;
        }

        Some((output, width, height))
    }

    fn compute_motion(
        &self,
        prev_gray: &[u8], pw: u32, ph: u32,
        curr_gray: &[u8], cw: u32, _ch: u32,
    ) -> (f32, f32) {
        let bs = self.effective_block_size();
        let sr = self.effective_search_radius();
        let mut dxs = Vec::new();
        let mut dys = Vec::new();
        let mut costs = Vec::new();

        for by in (0..ph).step_by(bs as usize) {
            for bx in (0..pw).step_by(bs as usize) {
                // Skip flat blocks
                let var = block_variance(prev_gray, pw, bx, by, bs);
                if var < TEXTURE_VARIANCE_THRESHOLD {
                    continue;
                }

                let match_result = if self.use_diamond_search {
                    diamond_search(
                        curr_gray, cw,
                        prev_gray, pw, ph,
                        bx, by, bs, sr,
                    )
                } else {
                    full_search(
                        curr_gray, cw,
                        prev_gray, pw, ph,
                        bx, by, bs, sr,
                    )
                };

                if let Some(m) = match_result {
                    if m.cost < 50.0 {
                        dxs.push(m.dx);
                        dys.push(m.dy);
                        costs.push(m.cost);
                    }
                }
            }
        }

        if dxs.is_empty() {
            return (0.0, 0.0);
        }

        let dx = weighted_median(&dxs, &costs);
        let dy = weighted_median(&dys, &costs);

        if self.debug {
            log::debug!(
                "Deshake: {} blocks, dx={:.1}, dy={:.1}",
                dxs.len(), dx, dy
            );
        }

        (dx, dy)
    }

    /// Get smoothed motion vector.
    pub fn smooth_motion(&self) -> [f32; 2] {
        self.smooth_motion
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deshake_initial_state() {
        let e = DeshakeEngine::new();
        assert!(!e.enabled);
        assert_eq!(e.frame_count, 0);
        assert_eq!(e.smooth_motion, [0.0, 0.0]);
    }

    #[test]
    fn test_first_frame_returns_none() {
        let mut e = DeshakeEngine::new();
        e.enabled = true;
        let data = vec![128u8; 100 * 100 * 4];
        let result = e.update(&data, 100, 100);
        assert!(result.is_none());
    }

    #[test]
    fn test_second_frame_returns_compensation() {
        let mut e = DeshakeEngine::new();
        e.enabled = true;
        let data = vec![128u8; 100 * 100 * 4];
        e.update(&data, 100, 100);
        let result = e.update(&data, 100, 100);
        assert!(result.is_some());
        assert_eq!(e.frame_count, 2);
    }

    #[test]
    fn test_reset() {
        let mut e = DeshakeEngine::new();
        e.enabled = true;
        let data = vec![128u8; 100 * 100 * 4];
        e.update(&data, 100, 100);
        e.update(&data, 100, 100);
        e.reset();
        assert_eq!(e.frame_count, 0);
        assert!(e.prev_gray.is_none());
    }

    #[test]
    fn test_small_frame_no_match() {
        let mut e = DeshakeEngine::new();
        e.enabled = true;
        let data = vec![128u8; 32 * 32 * 4];
        e.update(&data, 32, 32);
        let result = e.update(&data, 32, 32);
        // Frame too small, should return None
        assert!(result.is_none());
    }

    #[test]
    fn test_shifting_pattern_detects_motion() {
        let mut e = DeshakeEngine::new();
        e.enabled = true;
        e.block_size = 16;
        e.search_radius = 8;

        // Frame 1: pattern at x=10
        let mut f1 = vec![0u8; 64 * 64 * 4];
        for y in 10..30 {
            for x in 10..30 {
                let idx = (y * 64 + x) * 4;
                f1[idx] = 200;
                f1[idx + 1] = 200;
                f1[idx + 2] = 200;
                f1[idx + 3] = 255;
            }
        }
        e.update(&f1, 64, 64);

        // Frame 2: same pattern shifted right by 4px
        let mut f2 = vec![0u8; 64 * 64 * 4];
        for y in 10..30 {
            for x in 14..34 {
                let idx = (y * 64 + x) * 4;
                f2[idx] = 200;
                f2[idx + 1] = 200;
                f2[idx + 2] = 200;
                f2[idx + 3] = 255;
            }
        }
        e.update(&f2, 64, 64);

        let motion = e.smooth_motion();
        // Should detect rightward shift (motion vector points left to compensate)
        assert!(motion[0].abs() > 0.0, "Expected non-zero dx, got {}", motion[0]);
    }

    #[test]
    fn test_diamond_vs_full_search_accuracy() {
        // Both should find similar results for the same input
        // Create prev with a bright block at (16,16)
        let mut prev = vec![0u8; 64 * 64];
        for y in 16..24 {
            for x in 16..24 {
                prev[y * 64 + x] = 200;
            }
        }
        // Create curr with same block shifted right by 2px
        let mut curr = vec![0u8; 64 * 64];
        for y in 16..24 {
            for x in 18..26 {
                curr[y * 64 + x] = 200;
            }
        }

        let dm = diamond_search(&curr, 64, &prev, 64, 64, 16, 16, 8, 16);
        let fs = full_search(&curr, 64, &prev, 64, 64, 16, 16, 8, 16);

        assert!(dm.is_some());
        assert!(fs.is_some());
        let dm = dm.unwrap();
        let fs = fs.unwrap();
        // Both should find non-zero motion
        assert!(dm.dx.abs() > 0.0 || dm.dy.abs() > 0.0, "Diamond search should find motion");
        assert!(fs.dx.abs() > 0.0 || fs.dy.abs() > 0.0, "Full search should find motion");
        // Both should find motion in similar direction
        let dm_dir = if dm.dx > 0.0 { 1 } else if dm.dx < 0.0 { -1 } else { 0 };
        let fs_dir = if fs.dx > 0.0 { 1 } else if fs.dx < 0.0 { -1 } else { 0 };
        assert_eq!(dm_dir, fs_dir, "Both should find motion in same direction");
    }
}
