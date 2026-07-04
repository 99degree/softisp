//! EIS (Electronic Image Stabilization) engine.
//!
//! Ported from `com.camcore.isp.pipeline.controller.GyroEngine` (Java).
//!
//! Pure math — no Android/HAL dependency. Takes gyroscope samples
//! (timestamped angular velocities) and computes pixel-space compensation
//! for rolling-shutter jitter removal.
//!
//! # Flow
//! 1. Push gyro samples via `push_sample()`
//! 2. Each frame, call `update()` with frame timestamp + focal length
//! 3. Apply resulting `[dx, dy, roll_deg]` as inverse transform

use std::f64::consts::PI;

/// Size of the gyro ring buffer (holds ~0.5s of samples at 200 Hz).
const GYRO_BUFFER_CAPACITY: usize = 256;

/// EMA alpha for motion smoothing. Lower = smoother but more lag.
const EIS_SMOOTHING_ALPHA: f32 = 0.15;

/// Crop fraction reserved for stabilization borders (0..1). 0.1 = 10% each side.
const EIS_CROP_FRACTION: f32 = 0.10;

/// Maximum angular compensation in degrees (clamp to avoid excessive crop).
const EIS_MAX_ANGLE_DEG: f32 = 3.0;

/// A single gyroscope sample: angular velocity around X/Y/Z at a moment in time.
#[derive(Debug, Clone, Copy)]
pub struct GyroSample {
    /// Timestamp in nanoseconds (CLOCK_MONOTONIC or equivalent).
    pub timestamp_ns: i64,
    /// Angular velocity around X axis (rad/s) — pitch.
    pub x: f32,
    /// Angular velocity around Y axis (rad/s) — yaw.
    pub y: f32,
    /// Angular velocity around Z axis (rad/s) — roll.
    pub z: f32,
}

/// EIS state machine.
#[derive(Debug, Clone)]
pub struct EisEngine {
    /// Ring buffer of recent gyro samples.
    buffer: Vec<GyroSample>,
    /// Timestamp of the last processed frame (ns).
    last_frame_timestamp_ns: i64,
    /// Smoothed angular displacement (EMA) — low-frequency intentional motion.
    smooth_pitch: f32,
    smooth_yaw: f32,
    smooth_roll: f32,
    /// EIS enabled flag.
    pub enabled: bool,
    /// Debug logging flag.
    pub debug: bool,
}

impl Default for EisEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl EisEngine {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(GYRO_BUFFER_CAPACITY),
            last_frame_timestamp_ns: 0,
            smooth_pitch: 0.0,
            smooth_yaw: 0.0,
            smooth_roll: 0.0,
            enabled: false,
            debug: false,
        }
    }

    /// Push a gyro sample into the ring buffer.
    pub fn push_sample(&mut self, sample: GyroSample) {
        if self.buffer.len() >= GYRO_BUFFER_CAPACITY {
            self.buffer.remove(0);
        }
        self.buffer.push(sample);
    }

    /// Reset all EIS state.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.last_frame_timestamp_ns = 0;
        self.smooth_pitch = 0.0;
        self.smooth_yaw = 0.0;
        self.smooth_roll = 0.0;
    }

    /// Integrate gyro angular velocities between two frame timestamps.
    ///
    /// Sums all gyro samples whose timestamps fall within `[from_ns, to_ns]`.
    /// Each sample contributes ω·Δt to the angular displacement.
    ///
    /// Returns `[pitch_deg, yaw_deg, roll_deg]` displacement in degrees.
    pub fn integrate_gyro(&self, from_ns: i64, to_ns: i64) -> [f32; 3] {
        if from_ns >= to_ns || self.buffer.is_empty() {
            return [0.0, 0.0, 0.0];
        }

        let mut pitch = 0.0f64;
        let mut yaw = 0.0f64;
        let mut roll = 0.0f64;
        let mut prev_ts = from_ns;

        for sample in &self.buffer {
            if sample.timestamp_ns < from_ns {
                continue;
            }
            if sample.timestamp_ns > to_ns {
                break;
            }

            let dt = (sample.timestamp_ns - prev_ts).max(0) as f64 / 1_000_000_000.0;
            pitch += sample.x as f64 * dt;
            yaw += sample.y as f64 * dt;
            roll += sample.z as f64 * dt;
            prev_ts = sample.timestamp_ns;
        }

        // Partial interval after last sample up to to_ns
        let tail = (to_ns - prev_ts).max(0) as f64 / 1_000_000_000.0;
        if tail > 0.0 {
            if let Some(last) = self.buffer.last() {
                pitch += last.x as f64 * tail;
                yaw += last.y as f64 * tail;
                roll += last.z as f64 * tail;
            }
        }

        // Convert radians → degrees
        [
            (pitch * 180.0 / PI) as f32,
            (yaw * 180.0 / PI) as f32,
            (roll * 180.0 / PI) as f32,
        ]
    }

    /// Per-frame EIS update.
    ///
    /// 1. Integrate gyro between previous and current frame timestamps
    /// 2. EMA smoothing to separate jitter from intentional motion
    /// 3. Compute pixel-space compensation `[dx_px, dy_px, roll_deg]`
    ///
    /// The compensation should be applied as an inverse transform:
    ///   - Shift by (-dx, -dy) pixels
    ///   - Rotate by -roll degrees around center
    ///   - Crop by `EIS_CROP_FRACTION` on each edge to hide borders
    ///
    /// Returns `Some(`[dx, dy, roll_deg]`)` or `None` if EIS disabled or no data.
    pub fn update(
        &mut self,
        frame_timestamp_ns: i64,
        focal_length_px: f32,
        display_width: u32,
        display_height: u32,
    ) -> Option<[f32; 3]> {
        if !self.enabled {
            return None;
        }

        let prev_ts = self.last_frame_timestamp_ns;
        if prev_ts <= 0 {
            // First frame — just record timestamp, no compensation yet
            self.last_frame_timestamp_ns = frame_timestamp_ns;
            return None;
        }

        // 1) Integrate raw angular displacement between frames
        let raw_deg = self.integrate_gyro(prev_ts, frame_timestamp_ns);

        // 2) EMA smoothing
        let alpha = EIS_SMOOTHING_ALPHA;
        self.smooth_pitch += (raw_deg[0] - self.smooth_pitch) * alpha;
        self.smooth_yaw += (raw_deg[1] - self.smooth_yaw) * alpha;
        self.smooth_roll += (raw_deg[2] - self.smooth_roll) * alpha;

        // 3) Jitter = raw - smoothed (high-frequency component only)
        let jitter_pitch = raw_deg[0] - self.smooth_pitch;
        let jitter_yaw = raw_deg[1] - self.smooth_yaw;
        let jitter_roll = raw_deg[2] - self.smooth_roll;

        // Clamp to max angle
        let comp_pitch = jitter_pitch.clamp(-EIS_MAX_ANGLE_DEG, EIS_MAX_ANGLE_DEG);
        let comp_yaw = jitter_yaw.clamp(-EIS_MAX_ANGLE_DEG, EIS_MAX_ANGLE_DEG);
        let comp_roll = jitter_roll.clamp(-EIS_MAX_ANGLE_DEG, EIS_MAX_ANGLE_DEG);

        self.last_frame_timestamp_ns = frame_timestamp_ns;

        // 4) Convert angular displacement to pixel shift
        // For small angles: shift_px = focal_length * tan(angle_rad) ≈ focal_length * angle_rad
        let pitch_rad = comp_pitch * PI as f32 / 180.0;
        let yaw_rad = comp_yaw * PI as f32 / 180.0;

        let dx = -(focal_length_px * yaw_rad);   // yaw → horizontal shift
        let dy = -(focal_length_px * pitch_rad); // pitch → vertical shift

        // Clamp pixel shifts to crop margin
        let margin_x = (display_width as f32 * EIS_CROP_FRACTION).max(1.0);
        let margin_y = (display_height as f32 * EIS_CROP_FRACTION).max(1.0);
        let clamped_dx = dx.clamp(-margin_x, margin_x);
        let clamped_dy = dy.clamp(-margin_y, margin_y);

        let compensation = [clamped_dx, clamped_dy, comp_roll];

        if self.debug {
            log::debug!(
                "EIS: raw=[{:.2},{:.2},{:.2}] jit=[{:.2},{:.2},{:.2}] comp=[dx={:.1},dy={:.1},roll={:.2}]",
                raw_deg[0], raw_deg[1], raw_deg[2],
                jitter_pitch, jitter_yaw, jitter_roll,
                clamped_dx, clamped_dy, comp_roll,
            );
        }

        Some(compensation)
    }

    /// Compute the sampling grid for ONNX GridSample from EIS compensation.
    ///
    /// Each output pixel (x, y) is mapped to a source pixel (sx, sy) via the
    /// inverse of `[dx, dy, roll]` so that the rendered frame appears stabilized.
    ///
    /// Grid format: `[H][W][2]` where `[h,w,0]` = normalized X (column)
    /// and `[h,w,1]` = normalized Y (row), both in `[-1, 1]`.
    pub fn compute_warp_grid(
        w: u32,
        h: u32,
        comp: &[f32; 3],
    ) -> Vec<f32> {
        let dx = comp[0];
        let dy = comp[1];
        let roll_rad = comp[2] * PI as f32 / 180.0;
        let cos_r = roll_rad.cos();
        let sin_r = roll_rad.sin();
        let cx = w as f32 / 2.0;
        let cy = h as f32 / 2.0;

        let mut grid = Vec::with_capacity((w * h * 2) as usize);
        for y in 0..h {
            for x in 0..w {
                let xc = x as f32 - cx;
                let yc = y as f32 - cy;
                // Apply inverse of compensation affine:
                // Source coords = R(roll)^-1 * (output_coords - center) + center + translation
                let sx = cos_r * xc - sin_r * yc + cx + dx;
                let sy = sin_r * xc + cos_r * yc + cy + dy;
                // Normalize to [-1, 1] for GridSample
                grid.push(2.0 * sx / w as f32 - 1.0);
                grid.push(2.0 * sy / h as f32 - 1.0);
            }
        }
        grid
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eis_initial_state() {
        let eis = EisEngine::new();
        assert!(!eis.enabled);
        assert!(eis.buffer.is_empty());
        assert_eq!(eis.last_frame_timestamp_ns, 0);
    }

    #[test]
    fn test_push_sample() {
        let mut eis = EisEngine::new();
        eis.push_sample(GyroSample {
            timestamp_ns: 1000,
            x: 0.1,
            y: 0.2,
            z: 0.0,
        });
        assert_eq!(eis.buffer.len(), 1);
        assert!((eis.buffer[0].x - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_integrate_gyro_basic() {
        let mut eis = EisEngine::new();
        // Constant pitch rotation at 0.1 rad/s for 1 full second
        eis.push_sample(GyroSample {
            timestamp_ns: 1_000_000_000, // 1s
            x: 0.1,
            y: 0.0,
            z: 0.0,
        });

        // Integrate from 1s to 2s: the sample at 1s has rate 0.1 rad/s
        // dt = (sample.timestamp - from_ns) = 0 for the first sample.
        // Then tail = (to_ns - sample.timestamp) / 1e9 = (2e9 - 1e9) / 1e9 = 1.0
        // pitch = 0.1 * 0 (dt for sample itself) + 0.1 * 1.0 (tail) = 0.1 rad ≈ 5.73 deg
        let result = eis.integrate_gyro(1_000_000_000, 2_000_000_000);
        assert!((result[0] - 5.73).abs() < 0.1, "Pitch should be ~5.73°, got {}", result[0]);
        assert!((result[1]).abs() < 0.01, "Yaw should be ~0, got {}", result[1]);
    }

    #[test]
    fn test_eis_update_no_first_frame() {
        let mut eis = EisEngine::new();
        eis.enabled = true;
        // First call should return None (no previous timestamp)
        let result = eis.update(1_000_000_000, 500.0, 640, 480);
        assert!(result.is_none());
        // Second call should produce compensation
        let result = eis.update(2_000_000_000, 500.0, 640, 480);
        assert!(result.is_some());
    }

    #[test]
    fn test_compute_warp_grid() {
        let comp = [1.0, 2.0, 0.0]; // dx=1, dy=2, no rotation
        let grid = EisEngine::compute_warp_grid(4, 4, &comp);
        assert_eq!(grid.len(), 4 * 4 * 2); // 32 elements
        // Center pixel (2,2) should map to (2+1, 2+2) = (3, 4)
        // normalized: x=2*3/4-1=0.5, y=2*4/4-1=1.0
        let center_idx = (2 * 4 + 2) * 2; // row=2, col=2
        assert!((grid[center_idx] - 0.5).abs() < 0.01, "Center X should be 0.5, got {}", grid[center_idx]);
        assert!((grid[center_idx + 1] - 1.0).abs() < 0.01, "Center Y should be 1.0, got {}", grid[center_idx + 1]);
    }

    #[test]
    fn test_eis_reset() {
        let mut eis = EisEngine::new();
        eis.push_sample(GyroSample { timestamp_ns: 1000, x: 0.1, y: 0.0, z: 0.0 });
        eis.smooth_pitch = 5.0;
        eis.reset();
        assert!(eis.buffer.is_empty());
        assert_eq!(eis.smooth_pitch, 0.0);
        assert_eq!(eis.last_frame_timestamp_ns, 0);
    }

    #[test]
    fn test_eis_disabled() {
        let mut eis = EisEngine::new();
        eis.enabled = false;
        let result = eis.update(1_000_000_000, 500.0, 640, 480);
        assert!(result.is_none());
    }

    #[test]
    fn test_warp_grid_no_displacement() {
        let comp = [0.0, 0.0, 0.0];
        let grid = EisEngine::compute_warp_grid(4, 4, &comp);
        // No displacement → output pixel maps to itself
        let center_idx = (2 * 4 + 2) * 2;
        // For (2,2): normalized x = 2*2/4-1 = 0, y = 2*2/4-1 = 0
        assert!((grid[center_idx]).abs() < 0.01, "Center X should be 0, got {}", grid[center_idx]);
        assert!((grid[center_idx + 1]).abs() < 0.01, "Center Y should be 0, got {}", grid[center_idx + 1]);
    }
}
