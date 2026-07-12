//! Motion processing — EIS stabilizer, IMU data types, gyro-based stabilization.
//!
//! Ported from `com.camcore.motion` (Kotlin):
//! - `EisStabilizer.kt` — motion-adaptive frame stabilization via gyro data
//! - `ImuManager.kt` — IMU data types (Android sensor binding separate)

/// Raw IMU sensor data from gyroscope + accelerometer.
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    /// Gyroscope X (rad/s)
    pub gyro_x: f32,
    /// Gyroscope Y (rad/s)
    pub gyro_y: f32,
    /// Gyroscope Z (rad/s)
    pub gyro_z: f32,
    /// Accelerometer X (m/s²)
    pub accel_x: f32,
    /// Accelerometer Y (m/s²)
    pub accel_y: f32,
    /// Accelerometer Z (m/s²)
    pub accel_z: f32,
    /// Timestamp in nanoseconds.
    pub timestamp_ns: u64,
}

impl ImuData {
    /// Magnitude of gyroscope vector.
    pub fn gyro_magnitude(&self) -> f32 {
        (self.gyro_x * self.gyro_x + self.gyro_y * self.gyro_y + self.gyro_z * self.gyro_z).sqrt()
    }
}

impl Default for ImuData {
    fn default() -> Self {
        Self {
            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,
            accel_x: 0.0,
            accel_y: 0.0,
            accel_z: 0.0,
            timestamp_ns: 0,
        }
    }
}

/// Motion correction output from the stabilizer.
#[derive(Debug, Clone, Copy)]
pub struct MotionCorrection {
    /// Pixel displacement in X.
    pub delta_x: f32,
    /// Pixel displacement in Y.
    pub delta_y: f32,
    /// Rotation angle in degrees.
    pub delta_angle: f32,
    /// Confidence (0..1) — lower when motion is large/unstable.
    pub confidence: f32,
    /// Whether stabilization is active.
    pub stabilized: bool,
}

impl Default for MotionCorrection {
    fn default() -> Self {
        Self {
            delta_x: 0.0,
            delta_y: 0.0,
            delta_angle: 0.0,
            confidence: 1.0,
            stabilized: false,
        }
    }
}

/// EIS stabilizer — gyro-based electronic image stabilization.
///
/// Algorithm:
/// 1. Buffer last N raw corrections from gyro data
/// 2. Average buffer for low-pass filtering
/// 3. EMA smooth against previous correction
/// 4. Compute confidence inversely proportional to motion magnitude
pub struct EisStabilizer {
    /// Previous correction for EMA smoothing.
    prev_correction: MotionCorrection,
    /// EMA smooth factor (0..1). Higher = smoother but more lag.
    smooth_factor: f32,
    /// Circular buffer of recent corrections.
    buffer: Vec<MotionCorrection>,
    /// Maximum buffer size.
    buffer_size: usize,
}

impl EisStabilizer {
    pub fn new() -> Self {
        Self {
            prev_correction: MotionCorrection::default(),
            smooth_factor: 0.6,
            buffer: Vec::with_capacity(10),
            buffer_size: 10,
        }
    }

    /// Create with custom smoothing factor.
    pub fn with_smooth(smooth_factor: f32) -> Self {
        Self {
            prev_correction: MotionCorrection::default(),
            smooth_factor: smooth_factor.clamp(0.0, 1.0),
            buffer: Vec::with_capacity(10),
            buffer_size: 10,
        }
    }

    /// Compute the correction for the given IMU sample.
    ///
    /// `focal_length_px` is the approximate focal length in pixels (e.g. `width * 1.2`).
    /// `sensitivity` scales gyro → pixel displacement (default 0.02).
    pub fn compute_correction(
        &mut self,
        imu: &ImuData,
        focal_length_px: f32,
        sensitivity: f32,
    ) -> MotionCorrection {
        // Convert gyro (rad/s) to pixel displacement
        let raw_dx = imu.gyro_y * sensitivity * focal_length_px;
        let raw_dy = imu.gyro_x * sensitivity * focal_length_px;
        let raw_angle = imu.gyro_z * 0.001; // rad → ° (approx)

        // Add to buffer
        self.buffer.push(MotionCorrection {
            delta_x: raw_dx,
            delta_y: raw_dy,
            delta_angle: raw_angle,
            ..Default::default()
        });
        if self.buffer.len() > self.buffer_size {
            self.buffer.remove(0);
        }

        // Average buffer for low-pass
        let avg = if self.buffer.is_empty() {
            MotionCorrection::default()
        } else {
            let n = self.buffer.len() as f32;
            let sum_x: f32 = self.buffer.iter().map(|c| c.delta_x).sum();
            let sum_y: f32 = self.buffer.iter().map(|c| c.delta_y).sum();
            let sum_a: f32 = self.buffer.iter().map(|c| c.delta_angle).sum();
            MotionCorrection {
                delta_x: sum_x / n,
                delta_y: sum_y / n,
                delta_angle: sum_a / n,
                ..Default::default()
            }
        };

        // EMA smoothing
        let smooth = MotionCorrection {
            delta_x: self.prev_correction.delta_x
                + (avg.delta_x - self.prev_correction.delta_x) * self.smooth_factor,
            delta_y: self.prev_correction.delta_y
                + (avg.delta_y - self.prev_correction.delta_y) * self.smooth_factor,
            delta_angle: self.prev_correction.delta_angle
                + (avg.delta_angle - self.prev_correction.delta_angle) * self.smooth_factor,
            confidence: 1.0 / (1.0 + imu.gyro_magnitude()),
            stabilized: true,
        };

        self.prev_correction = smooth;
        smooth
    }

    /// Reset stabilizer state.
    pub fn reset(&mut self) {
        self.prev_correction = MotionCorrection::default();
        self.buffer.clear();
    }

    /// Get the last computed correction (without computing a new one).
    pub fn last_correction(&self) -> &MotionCorrection {
        &self.prev_correction
    }
}

impl Default for EisStabilizer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_data_default() {
        let imu = ImuData::default();
        assert_eq!(imu.gyro_x, 0.0);
        assert_eq!(imu.timestamp_ns, 0);
    }

    #[test]
    fn test_gyro_magnitude() {
        let imu = ImuData {
            gyro_x: 3.0,
            gyro_y: 4.0,
            ..Default::default()
        };
        assert!((imu.gyro_magnitude() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_motion_correction_default() {
        let mc = MotionCorrection::default();
        assert_eq!(mc.delta_x, 0.0);
        assert_eq!(mc.confidence, 1.0);
        assert!(!mc.stabilized);
    }

    #[test]
    fn test_stabilizer_initial_correction() {
        let mut stab = EisStabilizer::new();
        let imu = ImuData {
            gyro_x: 0.1,
            gyro_y: 0.05,
            ..Default::default()
        };
        let corr = stab.compute_correction(&imu, 100.0, 0.02);
        assert!(corr.stabilized);
        assert!(corr.confidence > 0.0 && corr.confidence <= 1.0);
    }

    #[test]
    fn test_stabilizer_convergence() {
        let mut stab = EisStabilizer::with_smooth(0.8);
        let imu = ImuData {
            gyro_x: 0.2,
            gyro_y: 0.0,
            ..Default::default()
        };
        // After multiple frames with same input, correction should converge to steady state
        let mut _last_dx = 0.0;
        for _ in 0..20 {
            let corr = stab.compute_correction(&imu, 100.0, 0.02);
            _last_dx = corr.delta_x;
        }
        // gyro_x=0.2, focal_length=100, sensitivity=0.02 → dy ≈ 0.2*0.02*100 = 0.4 px
        // With buffer averaging and EMA smoothing, converges to ~0.4
        let corr = stab.compute_correction(&imu, 100.0, 0.02);
        assert!(corr.delta_y.abs() > 0.1);
        assert!(corr.stabilized);
        assert!(corr.confidence > 0.0);
    }

    #[test]
    fn test_stabilizer_reset() {
        let mut stab = EisStabilizer::new();
        let imu = ImuData {
            gyro_x: 1.0,
            ..Default::default()
        };
        stab.compute_correction(&imu, 100.0, 0.02);
        stab.reset();
        let corr = stab.compute_correction(&ImuData::default(), 100.0, 0.02);
        assert!(corr.delta_x.abs() < 1e-5);
    }
}
