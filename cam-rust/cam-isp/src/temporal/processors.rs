//! Temporal processors — apply coefficients to frames.

use super::coefficients::FrameCoefficients;
use super::frame::TemporalFrame;

/// Trait for temporal processing using coefficients.
pub trait TemporalProcessor: Send + Sync {
    /// Process a frame using extracted coefficients.
    fn process(&self, frame: &TemporalFrame, coefficients: &FrameCoefficients) -> TemporalFrame;
    /// Get the name of this processor.
    fn name(&self) -> &str;
}

/// Temporal noise reduction using historical coefficients.
pub struct TemporalDenoiseProcessor {
    pub strength: f32,
}

impl TemporalDenoiseProcessor {
    pub fn new(strength: f32) -> Self {
        Self { strength }
    }
}

impl Default for TemporalDenoiseProcessor {
    fn default() -> Self {
        Self::new(0.5)
    }
}

impl TemporalProcessor for TemporalDenoiseProcessor {
    fn process(&self, frame: &TemporalFrame, coefficients: &FrameCoefficients) -> TemporalFrame {
        let w = frame.width as usize;
        let h = frame.height as usize;
        let mut output = frame.clone();

        for y in 0..h {
            for x in 0..w {
                let idx = y * w + x;
                let noise = coefficients.noise_map.get(idx).copied().unwrap_or(0.0);
                let factor = 1.0 - (noise * self.strength * 0.01).min(0.9);

                for c in 0..3 {
                    output.set_pixel(
                        x as u32,
                        y as u32,
                        c,
                        frame.get_pixel(x as u32, y as u32, c) * factor,
                    );
                }
            }
        }

        output
    }

    fn name(&self) -> &str {
        "TemporalDenoiseProcessor"
    }
}

/// Motion-adaptive processing.
pub struct MotionAdaptiveProcessor {
    pub motion_strength: f32,
    pub static_strength: f32,
}

impl MotionAdaptiveProcessor {
    pub fn new(motion_strength: f32, static_strength: f32) -> Self {
        Self {
            motion_strength,
            static_strength,
        }
    }
}

impl Default for MotionAdaptiveProcessor {
    fn default() -> Self {
        Self::new(0.3, 0.8)
    }
}

impl TemporalProcessor for MotionAdaptiveProcessor {
    fn process(&self, frame: &TemporalFrame, coefficients: &FrameCoefficients) -> TemporalFrame {
        let w = frame.width as usize;
        let h = frame.height as usize;
        let mut output = frame.clone();

        for y in 0..h {
            for x in 0..w {
                let idx = y * w + x;
                let motion = coefficients.motion_map.get(idx).copied().unwrap_or(0.0);
                let strength = self.static_strength
                    + (self.motion_strength - self.static_strength) * motion.min(1.0);
                let gain = coefficients.gain_map.get(idx).copied().unwrap_or(1.0);

                for c in 0..3 {
                    let val = frame.get_pixel(x as u32, y as u32, c);
                    output.set_pixel(
                        x as u32,
                        y as u32,
                        c,
                        val * (1.0 + (gain - 1.0) * strength),
                    );
                }
            }
        }

        output
    }

    fn name(&self) -> &str {
        "MotionAdaptiveProcessor"
    }
}

/// Gain/offset calibration processor.
pub struct GainCalibrationProcessor;

impl TemporalProcessor for GainCalibrationProcessor {
    fn process(&self, frame: &TemporalFrame, coefficients: &FrameCoefficients) -> TemporalFrame {
        let w = frame.width as usize;
        let h = frame.height as usize;
        let mut output = frame.clone();

        for y in 0..h {
            for x in 0..w {
                let idx = y * w + x;
                let gain = coefficients.gain_map.get(idx).copied().unwrap_or(1.0);
                let offset = coefficients.offset_map.get(idx).copied().unwrap_or(0.0);

                for c in 0..3 {
                    output.set_pixel(
                        x as u32,
                        y as u32,
                        c,
                        frame.get_pixel(x as u32, y as u32, c) * gain + offset,
                    );
                }
            }
        }

        output
    }

    fn name(&self) -> &str {
        "GainCalibrationProcessor"
    }
}

/// Color balance processor using AWB statistics.
pub struct ColorBalanceProcessor {
    pub target_rg: f32,
    pub target_bg: f32,
}

impl ColorBalanceProcessor {
    pub fn new(target_rg: f32, target_bg: f32) -> Self {
        Self { target_rg, target_bg }
    }
}

impl Default for ColorBalanceProcessor {
    fn default() -> Self {
        Self::new(1.0, 1.0)
    }
}

impl TemporalProcessor for ColorBalanceProcessor {
    fn process(&self, frame: &TemporalFrame, coefficients: &FrameCoefficients) -> TemporalFrame {
        let w = frame.width as usize;
        let h = frame.height as usize;

        let (r_m, g_m, b_m) = (
            coefficients.color_stats[0],
            coefficients.color_stats[1],
            coefficients.color_stats[2],
        );

        let r_gain = self.target_rg / (r_m / g_m.max(1.0)).max(0.1);
        let b_gain = self.target_bg / (b_m / g_m.max(1.0)).max(0.1);

        let mut output = frame.clone();
        for y in 0..h {
            for x in 0..w {
                output.set_pixel(x as u32, y as u32, 0, frame.get_pixel(x as u32, y as u32, 0) * r_gain);
                output.set_pixel(x as u32, y as u32, 1, frame.get_pixel(x as u32, y as u32, 1));
                output.set_pixel(x as u32, y as u32, 2, frame.get_pixel(x as u32, y as u32, 2) * b_gain);
            }
        }

        output
    }

    fn name(&self) -> &str {
        "ColorBalanceProcessor"
    }
}
