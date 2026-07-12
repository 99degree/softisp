//! Coefficient extractors — extract temporal statistics from frames.

use super::coefficients::FrameCoefficients;
use super::frame::TemporalFrame;

/// Trait for coefficient extraction from frames.
pub trait CoeffExtractor: Send + Sync {
    /// Extract coefficients from a single frame.
    fn extract(
        &self,
        frame: &TemporalFrame,
        prev_coefficients: Option<&FrameCoefficients>,
    ) -> FrameCoefficients;
    /// Get the name of this extractor.
    fn name(&self) -> &str;
}

/// Extracts noise model from frame history.
pub struct NoiseExtractor {
    pub smoothing: f32,
}

impl NoiseExtractor {
    pub fn new(smoothing: f32) -> Self {
        Self { smoothing }
    }
}

impl Default for NoiseExtractor {
    fn default() -> Self {
        Self::new(0.1)
    }
}

impl CoeffExtractor for NoiseExtractor {
    fn extract(
        &self,
        frame: &TemporalFrame,
        prev: Option<&FrameCoefficients>,
    ) -> FrameCoefficients {
        let w = frame.width as usize;
        let h = frame.height as usize;
        let size = w * h;
        let mut noise_map = vec![0.0f32; size];

        for y in 0..h {
            for x in 0..w {
                let r = frame.get_pixel(x as u32, y as u32, 0);
                let g = frame.get_pixel(x as u32, y as u32, 1);
                let b = frame.get_pixel(x as u32, y as u32, 2);
                noise_map[y * w + x] = (0.299 * r + 0.587 * g + 0.114 * b).powi(2);
            }
        }

        let mut coeffs = FrameCoefficients::new(
            frame.width,
            frame.height,
            1,
            noise_map,
            vec![0.0; size],
            vec![1.0; size],
            vec![0.0; size],
        );

        if let Some(prev) = prev {
            coeffs.smooth_with(prev, self.smoothing);
            coeffs.history_size = prev.history_size + 1;
        }

        coeffs
    }

    fn name(&self) -> &str {
        "NoiseExtractor"
    }
}

/// Extracts motion between frames.
pub struct MotionExtractor {
    pub block_size: u32,
}

impl MotionExtractor {
    pub fn new(block_size: u32) -> Self {
        Self { block_size }
    }
}

impl Default for MotionExtractor {
    fn default() -> Self {
        Self::new(16)
    }
}

impl CoeffExtractor for MotionExtractor {
    fn extract(
        &self,
        frame: &TemporalFrame,
        prev: Option<&FrameCoefficients>,
    ) -> FrameCoefficients {
        let w = frame.width as usize;
        let h = frame.height as usize;
        let size = w * h;
        let mut motion_map = vec![0.0f32; size];

        // Smooth with previous motion map
        if let Some(prev) = prev {
            for i in 0..size {
                motion_map[i] = prev.motion_map.get(i).copied().unwrap_or(0.0) * 0.9;
            }
        }

        // Block-based motion estimation
        let bs = self.block_size as usize;
        for by in (0..h).step_by(bs) {
            for bx in (0..w).step_by(bs) {
                let mut sum = 0.0f32;
                let mut sum_sq = 0.0f32;
                let mut count = 0u32;

                for y in by..(by + bs).min(h) {
                    for x in bx..(bx + bs).min(w) {
                        let luma = 0.299 * frame.get_pixel(x as u32, y as u32, 0)
                            + 0.587 * frame.get_pixel(x as u32, y as u32, 1)
                            + 0.114 * frame.get_pixel(x as u32, y as u32, 2);
                        sum += luma;
                        sum_sq += luma * luma;
                        count += 1;
                    }
                }

                let mean = sum / count as f32;
                let variance = sum_sq / count as f32 - mean * mean;

                for y in by..(by + bs).min(h) {
                    for x in bx..(bx + bs).min(w) {
                        motion_map[y * w + x] = variance.sqrt();
                    }
                }
            }
        }

        FrameCoefficients::new(
            frame.width,
            frame.height,
            1,
            vec![0.0; size],
            motion_map,
            vec![1.0; size],
            vec![0.0; size],
        )
    }

    fn name(&self) -> &str {
        "MotionExtractor"
    }
}

/// Extracts gain/offset calibration from frames.
pub struct GainExtractor {
    pub target_gray: f32,
}

impl GainExtractor {
    pub fn new(target_gray: f32) -> Self {
        Self { target_gray }
    }
}

impl Default for GainExtractor {
    fn default() -> Self {
        Self::new(128.0)
    }
}

impl CoeffExtractor for GainExtractor {
    fn extract(
        &self,
        frame: &TemporalFrame,
        prev: Option<&FrameCoefficients>,
    ) -> FrameCoefficients {
        let w = frame.width as usize;
        let h = frame.height as usize;
        let size = w * h;

        let (mut r_sum, mut g_sum, mut b_sum, mut count) = (0.0f32, 0.0f32, 0.0f32, 0u32);
        for y in 0..h {
            for x in 0..w {
                r_sum += frame.get_pixel(x as u32, y as u32, 0);
                g_sum += frame.get_pixel(x as u32, y as u32, 1);
                b_sum += frame.get_pixel(x as u32, y as u32, 2);
                count += 1;
            }
        }

        let r_gain = self.target_gray / (r_sum / count as f32).max(1.0);
        let g_gain = self.target_gray / (g_sum / count as f32).max(1.0);
        let b_gain = self.target_gray / (b_sum / count as f32).max(1.0);

        let mut gain_map = vec![1.0f32; size];
        for y in 0..h {
            for x in 0..w {
                let ch = x % 3;
                gain_map[y * w + x] = match ch {
                    0 => r_gain,
                    1 => g_gain,
                    _ => b_gain,
                };
            }
        }

        let mut coeffs = FrameCoefficients::new(
            frame.width,
            frame.height,
            1,
            vec![0.0; size],
            vec![0.0; size],
            gain_map,
            vec![0.0; size],
        );
        coeffs.color_stats = [
            r_sum / count as f32,
            g_sum / count as f32,
            b_sum / count as f32,
            0.0,
            0.0,
            0.0,
        ];

        if let Some(prev) = prev {
            coeffs.smooth_with(prev, 0.2);
            coeffs.history_size = prev.history_size + 1;
        }

        coeffs
    }

    fn name(&self) -> &str {
        "GainExtractor"
    }
}

/// Extracts color statistics for AWB.
pub struct ColorExtractor {
    pub smoothing: f32,
}

impl ColorExtractor {
    pub fn new(smoothing: f32) -> Self {
        Self { smoothing }
    }
}

impl Default for ColorExtractor {
    fn default() -> Self {
        Self::new(0.1)
    }
}

impl CoeffExtractor for ColorExtractor {
    fn extract(
        &self,
        frame: &TemporalFrame,
        prev: Option<&FrameCoefficients>,
    ) -> FrameCoefficients {
        let w = frame.width as usize;
        let h = frame.height as usize;

        let (mut r_sum, mut g_sum, mut b_sum, mut count) = (0.0f32, 0.0f32, 0.0f32, 0u32);
        for y in 0..h {
            for x in 0..w {
                r_sum += frame.get_pixel(x as u32, y as u32, 0);
                g_sum += frame.get_pixel(x as u32, y as u32, 1);
                b_sum += frame.get_pixel(x as u32, y as u32, 2);
                count += 1;
            }
        }

        let (r_m, g_m, b_m) = (
            r_sum / count as f32,
            g_sum / count as f32,
            b_sum / count as f32,
        );

        let mut coeffs = FrameCoefficients::empty(frame.width, frame.height);
        coeffs.color_stats = [r_m, g_m, b_m, 0.0, 0.0, 0.0];
        coeffs.history_size = 1;

        if let Some(prev) = prev {
            coeffs.smooth_with(prev, self.smoothing);
            coeffs.history_size = prev.history_size + 1;
        }

        coeffs
    }

    fn name(&self) -> &str {
        "ColorExtractor"
    }
}
