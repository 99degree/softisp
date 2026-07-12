//! ISP Controller — analyzes frames and drives pipeline parameters.
//!
//! The controller makes all decisions about image processing,
//! while the pipeline blocks remain pure parameterized functions.

use crate::isp_params::*;
use crate::pipeline::IspFrame;

/// ISP Controller — analyzes frames and generates parameters.
pub struct IspController {
    /// Current parameters.
    params: IspParams,
    /// Target brightness (0.0-1.0).
    target_brightness: f32,
    /// Target color temperature (K).
    target_color_temp: f32,
    /// Smoothing factor for parameter updates.
    smoothing: f32,
    /// Previous frame stats.
    prev_stats: Option<FrameStats>,
}

impl IspController {
    /// Create a new controller.
    pub fn new() -> Self {
        Self {
            params: IspParams::identity(),
            target_brightness: 0.5,
            target_color_temp: 5500.0,
            smoothing: 0.1,
            prev_stats: None,
        }
    }

    /// Create with custom targets.
    pub fn with_targets(target_brightness: f32, target_color_temp: f32) -> Self {
        Self {
            target_brightness,
            target_color_temp,
            ..Self::new()
        }
    }

    /// Analyze frame and update parameters.
    pub fn analyze_and_update(&mut self, frame: &IspFrame) -> &IspParams {
        // 1. Compute frame statistics
        let stats = self.compute_stats(frame);

        // 2. Update auto-exposure (brightness)
        self.update_exposure(&stats);

        // 3. Update auto-white-balance
        self.update_white_balance(&stats);

        // 4. Update denoise based on noise level
        self.update_denoise(&stats);

        // 5. Update sharpening based on scene
        self.update_sharpen(&stats);

        // 6. Update tone mapping based on histogram
        self.update_tone(&stats);

        // 7. Update saturation based on scene
        self.update_saturation(&stats);

        // Store stats for next frame
        self.prev_stats = Some(stats);

        &self.params
    }

    /// Compute statistics from frame.
    fn compute_stats(&self, frame: &IspFrame) -> FrameStats {
        let mut stats = FrameStats::default();

        if frame.data.is_empty() {
            return stats;
        }

        // Compute average luminance
        let mut sum = 0.0f32;
        let mut count = 0u32;

        // Sample every 4th pixel for speed
        for (i, &val) in frame.data.iter().enumerate() {
            if i % 4 == 0 {
                sum += val as f32;
                count += 1;
            }
        }

        if count > 0 {
            stats.avg_luminance = sum / count as f32 / 255.0;
        }

        // Compute histogram
        for &val in frame.data.iter().step_by(4) {
            let bin = (val as u32).min(255);
            stats.histogram[bin as usize] += 1;
        }

        // Estimate noise level (simplified)
        stats.noise_level = self.estimate_noise(&frame.data);

        stats
    }

    /// Estimate noise level.
    fn estimate_noise(&self, data: &[u8]) -> f32 {
        // Simplified noise estimation using median absolute deviation
        if data.len() < 100 {
            return 0.0;
        }

        // Sample some pixels
        let sample: Vec<f32> = data.iter().step_by(16).map(|&v| v as f32).collect();

        let mean = sample.iter().sum::<f32>() / sample.len() as f32;
        let variance =
            sample.iter().map(|&v| (v - mean).powi(2)).sum::<f32>() / sample.len() as f32;

        variance.sqrt() / 255.0
    }

    /// Update exposure based on brightness.
    fn update_exposure(&mut self, stats: &FrameStats) {
        let diff = self.target_brightness - stats.avg_luminance;

        // Adjust brightness
        self.params.tone.brightness += diff * self.smoothing;
        self.params.tone.brightness = self.params.tone.brightness.clamp(-0.5, 0.5);
    }

    /// Update white balance based on color temperature.
    fn update_white_balance(&mut self, stats: &FrameStats) {
        // Simple color temperature to WB gains mapping
        let temp_ratio = stats.color_temp / self.target_color_temp;

        // Warm light (low temp) needs more blue, cool light (high temp) needs more red
        self.params.wb.r = 1.0 / temp_ratio;
        self.params.wb.b = temp_ratio;
        self.params.wb.g = 1.0;

        // Clamp to reasonable range
        self.params.wb.r = self.params.wb.r.clamp(0.5, 2.0);
        self.params.wb.b = self.params.wb.b.clamp(0.5, 2.0);
    }

    /// Update denoise based on noise level.
    fn update_denoise(&mut self, stats: &FrameStats) {
        if stats.noise_level > 0.1 {
            // High noise - increase denoise
            self.params.denoise.spatial_strength = (stats.noise_level * 2.0).min(1.0);
            self.params.denoise.temporal_strength = 0.3;
        } else if stats.noise_level > 0.05 {
            // Medium noise
            self.params.denoise.spatial_strength = stats.noise_level;
            self.params.denoise.temporal_strength = 0.1;
        } else {
            // Low noise - light denoise
            self.params.denoise.spatial_strength = 0.0;
            self.params.denoise.temporal_strength = 0.0;
        }
    }

    /// Update sharpening based on scene.
    fn update_sharpen(&mut self, stats: &FrameStats) {
        // Less sharpening in low light (amplifies noise)
        if stats.noise_level > 0.1 {
            self.params.sharpen.amount = 0.2;
        } else {
            self.params.sharpen.amount = 0.5;
        }
    }

    /// Update tone mapping based on histogram.
    fn update_tone(&mut self, stats: &FrameStats) {
        // Check if histogram is clipped
        let highlights = stats.histogram[240..256].iter().sum::<u32>();
        let shadows = stats.histogram[0..16].iter().sum::<u32>();
        let total = stats.histogram.iter().sum::<u32>();

        if total == 0 {
            return;
        }

        let highlight_ratio = highlights as f32 / total as f32;
        let shadow_ratio = shadows as f32 / total as f32;

        // Adjust contrast based on dynamic range
        if highlight_ratio > 0.1 || shadow_ratio > 0.1 {
            // High dynamic range - reduce contrast
            self.params.tone.contrast = 0.8;
        } else {
            self.params.tone.contrast = 1.0;
        }
    }

    /// Update saturation based on scene.
    fn update_saturation(&mut self, stats: &FrameStats) {
        // Adjust saturation based on scene type
        match stats.scene {
            SceneType::Portrait => {
                self.params.saturation.factor = 1.0;
                self.params.saturation.vibrance = 0.2;
            }
            SceneType::Landscape => {
                self.params.saturation.factor = 1.2;
                self.params.saturation.vibrance = 0.3;
            }
            SceneType::LowLight => {
                self.params.saturation.factor = 0.8;
                self.params.saturation.vibrance = 0.0;
            }
            _ => {
                self.params.saturation.factor = 1.0;
                self.params.saturation.vibrance = 0.0;
            }
        }
    }

    /// Get current parameters.
    pub fn params(&self) -> &IspParams {
        &self.params
    }

    /// Set parameters directly.
    pub fn set_params(&mut self, params: IspParams) {
        self.params = params;
    }

    /// Set target brightness.
    pub fn set_target_brightness(&mut self, brightness: f32) {
        self.target_brightness = brightness.clamp(0.0, 1.0);
    }

    /// Set target color temperature.
    pub fn set_target_color_temp(&mut self, temp: f32) {
        self.target_color_temp = temp.clamp(2000.0, 10000.0);
    }

    /// Set smoothing factor.
    pub fn set_smoothing(&mut self, smoothing: f32) {
        self.smoothing = smoothing.clamp(0.0, 1.0);
    }

    /// Get last frame stats.
    pub fn last_stats(&self) -> Option<&FrameStats> {
        self.prev_stats.as_ref()
    }
}

impl Default for IspController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_controller_creation() {
        let controller = IspController::new();
        assert_eq!(controller.target_brightness, 0.5);
    }

    #[test]
    fn test_controller_with_targets() {
        let controller = IspController::with_targets(0.6, 5000.0);
        assert_eq!(controller.target_brightness, 0.6);
        assert_eq!(controller.target_color_temp, 5000.0);
    }

    #[test]
    fn test_controller_params() {
        let controller = IspController::new();
        let params = controller.params();
        assert_eq!(params.wb.r, 1.0);
    }

    #[test]
    fn test_controller_set_brightness() {
        let mut controller = IspController::new();
        controller.set_target_brightness(0.7);
        assert_eq!(controller.target_brightness, 0.7);
    }

    #[test]
    fn test_controller_analyze_empty_frame() {
        let mut controller = IspController::new();
        let frame = IspFrame {
            params: IspParams::default(),
            width: 100,
            height: 100,
            data: vec![],
            format: cam_types::FrameFormat::Rgb888,
            float_data: None,
            aux: None,
            timestamp_ns: 0,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
        };
        let params = controller.analyze_and_update(&frame);
        assert!(params.tone.brightness.abs() < 0.01);
    }
}

impl crate::controller_api::ControllerApi for IspController {
    fn analyze_and_update(&mut self, frame: &IspFrame) -> IspParams {
        self.analyze_and_update(frame).clone()
    }

    fn last_params(&self) -> Option<&IspParams> {
        Some(self.params())
    }
}
