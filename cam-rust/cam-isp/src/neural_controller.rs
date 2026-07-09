//! Neural Controller — ISP parameter prediction via distilled model.
//!
//! Wraps `isp_rectifier::OptimizedInference` with automatic fallback
//! to the rule-based `IspController` when the model is unavailable or fails.

use log::{info, warn};

use crate::isp_params::*;
use crate::isp_controller::IspController;
use crate::error::IspResult;
use crate::pipeline::IspFrame;

/// Neural controller with fallback to rule-based controller.
pub struct NeuralController {
    /// Primary: distilled neural network controller
    #[cfg(feature = "rectifier")]
    rectifier: Option<isp_rectifier::OptimizedInference>,
    
    /// Fallback: rule-based controller
    fallback: IspController,
    
    /// Last good parameters (for temporal smoothing)
    last_params: Option<IspParams>,
}

impl NeuralController {
    /// Create controller without model (pure fallback).
    pub fn new() -> Self {
        Self {
            #[cfg(feature = "rectifier")]
            rectifier: None,
            fallback: IspController::new(),
            last_params: None,
        }
    }
    
    /// Create controller with ONNX model path.
    #[cfg(feature = "rectifier")]
    pub fn with_model(model_path: &str) -> Self {
        let rectifier = match isp_rectifier::OptimizedInference::new(model_path, true) {
            Ok(r) => {
                info!("NeuralController: loaded model from {}", model_path);
                Some(r)
            }
            Err(e) => {
                warn!("NeuralController: failed to load model: {}, using fallback", e);
                None
            }
        };
        
        Self {
            rectifier,
            fallback: IspController::new(),
            last_params: None,
        }
    }
    
    /// Check if neural model is available.
    pub fn has_model(&self) -> bool {
        #[cfg(feature = "rectifier")]
        { self.rectifier.is_some() }
        #[cfg(not(feature = "rectifier"))]
        { false }
    }
    
    /// Analyze frame and produce ISP parameters.
    pub fn analyze_and_update(&mut self, frame: &IspFrame) -> IspParams {
        let params = self.analyze_inner(frame);
        
        let smoothed = if let Some(ref last) = self.last_params {
            Self::smooth_params(last, &params, 0.3)
        } else {
            params.clone()
        };
        
        self.last_params = Some(smoothed.clone());
        smoothed
    }
    
    #[cfg(feature = "rectifier")]
    fn analyze_inner(&mut self, frame: &IspFrame) -> IspParams {
        if let Some(ref mut rectifier) = self.rectifier {
            match self.extract_metadata(frame) {
                Ok(metadata) => {
                    match rectifier.optimize(&metadata) {
                        Ok(optimized) => {
                            info!("NeuralController: neural model succeeded");
                            self.params_from_optimized(&optimized)
                        }
                        Err(e) => {
                            warn!("NeuralController: model inference failed: {}, using fallback", e);
                            self.fallback.analyze_and_update(frame).clone()
                        }
                    }
                }
                Err(e) => {
                    warn!("NeuralController: metadata extraction failed: {}, using fallback", e);
                    self.fallback.analyze_and_update(frame).clone()
                }
            }
        } else {
            self.fallback.analyze_and_update(frame).clone()
        }
    }
    
    #[cfg(not(feature = "rectifier"))]
    fn analyze_inner(&mut self, frame: &IspFrame) -> IspParams {
        self.fallback.analyze_and_update(frame).clone()
    }
    
    #[cfg(feature = "rectifier")]
    fn extract_metadata(&self, frame: &IspFrame) -> IspResult<isp_rectifier::FrameMetadata> {
        let histogram = self.compute_histogram(frame)?;
        
        Ok(isp_rectifier::FrameMetadata {
            histogram,
            cct: 5500.0,
            wb_gains: [1.0, 1.0, 1.0],
            ae: isp_rectifier::AutoExposure {
                exposure_time: 0.033,
                iso_gain: 1.0,
                target_brightness: 0.5,
            },
            af: isp_rectifier::AutoFocus {
                position: 0.5,
                sharpness: 0.5,
            },
            awb: isp_rectifier::AutoWhiteBalance {
                gains: [1.0, 1.0, 1.0],
                confidence: 0.5,
            },
            brightness: 0.5,
            contrast: 0.5,
            noise_level: 0.1,
            timestamp: frame.timestamp_ns,
        })
    }
    
    fn compute_histogram(&self, frame: &IspFrame) -> IspResult<Vec<u32>> {
        let mut histogram = vec![0u32; 256];
        
        if frame.data.is_empty() {
            return Ok(histogram);
        }
        
        let bytes_per_pixel = match frame.format {
            cam_types::FrameFormat::RawSensor => 2,
            cam_types::FrameFormat::Raw10 => 1,
            cam_types::FrameFormat::Raw12 => 1,
            _ => 4,
        };
        
        let step = bytes_per_pixel.max(1);
        for i in (0..frame.data.len()).step_by(step) {
            let val = if bytes_per_pixel == 2 && i + 1 < frame.data.len() {
                let raw = (frame.data[i] as u16) | ((frame.data[i + 1] as u16) << 8);
                (raw >> 8) as u8
            } else {
                frame.data[i]
            };
            histogram[val as usize] += 1;
        }
        
        Ok(histogram)
    }
    
    #[cfg(feature = "rectifier")]
    fn params_from_optimized(&self, optimized: &isp_rectifier::ISPOptimizedParams) -> IspParams {
        IspParams {
            blc: BlcParams::default(),
            wb: WbParams {
                r: optimized.wb_r_gain,
                g: optimized.wb_g_gain,
                b: optimized.wb_b_gain,
            },
            ccm: CcmParams {
                matrix: {
                    let mut m = [0.0f32; 9];
                    for i in 0..3 {
                        for j in 0..3 {
                            m[i * 3 + j] = optimized.ccm[i][j];
                        }
                    }
                    m
                },
            },
            tone: ToneParams {
                contrast: 1.0,
                brightness: 0.0,
                gamma: 1.0,
                curve_lut: optimized.tone_curve_lut.clone(),
            },
            saturation: SaturationParams::default(),
            sharpen: SharpenParams::default(),
            denoise: DenoiseParams::default(),
            lens: LensParams::default(),
            display: DisplayParams::default(),
            custom: std::collections::HashMap::new(),
        }
    }
    
    fn smooth_params(old: &IspParams, new: &IspParams, alpha: f32) -> IspParams {
        let lerp = |a: f32, b: f32| a * (1.0 - alpha) + b * alpha;
        
        IspParams {
            blc: new.blc.clone(),
            wb: WbParams {
                r: lerp(old.wb.r, new.wb.r),
                g: lerp(old.wb.g, new.wb.g),
                b: lerp(old.wb.b, new.wb.b),
            },
            ccm: CcmParams {
                matrix: {
                    let mut m = [0.0f32; 9];
                    for i in 0..9 {
                        m[i] = lerp(old.ccm.matrix[i], new.ccm.matrix[i]);
                    }
                    m
                },
            },
            tone: new.tone.clone(),
            saturation: SaturationParams {
                factor: lerp(old.saturation.factor, new.saturation.factor),
                vibrance: lerp(old.saturation.vibrance, new.saturation.vibrance),
            },
            sharpen: SharpenParams {
                amount: lerp(old.sharpen.amount, new.sharpen.amount),
                radius: lerp(old.sharpen.radius, new.sharpen.radius),
                threshold: lerp(old.sharpen.threshold, new.sharpen.threshold),
            },
            denoise: DenoiseParams {
                spatial_strength: lerp(old.denoise.spatial_strength, new.denoise.spatial_strength),
                temporal_strength: lerp(old.denoise.temporal_strength, new.denoise.temporal_strength),
                edge_preserve: lerp(old.denoise.edge_preserve, new.denoise.edge_preserve),
                bilateral_sigma: lerp(old.denoise.bilateral_sigma, new.denoise.bilateral_sigma),
            },
            lens: new.lens.clone(),
            display: new.display.clone(),
            custom: new.custom.clone(),
        }
    }
}

impl Default for NeuralController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_neural_controller_fallback() {
        let mut ctrl = NeuralController::new();
        assert!(!ctrl.has_model());
        
        let frame = IspFrame {
            width: 64,
            height: 48,
            data: vec![128; 64 * 48 * 2],
            format: cam_types::FrameFormat::RawSensor,
            float_data: None,
            aux: None,
            timestamp_ns: 0,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
        };
        
        let params = ctrl.analyze_and_update(&frame);
        assert!(params.wb.r > 0.0);
    }
    
    #[test]
    fn test_temporal_smoothing() {
        let old = IspParams::default();
        let mut new = IspParams::default();
        new.wb.r = 2.0;
        new.wb.b = 0.5;
        
        let smoothed = NeuralController::smooth_params(&old, &new, 0.3);
        
        assert!((smoothed.wb.r - 1.3).abs() < 0.01);
        assert!((smoothed.wb.b - 0.85).abs() < 0.01);
    }
}

impl crate::controller_api::ControllerApi for NeuralController {
    fn analyze_and_update(&mut self, frame: &IspFrame) -> IspParams {
        self.analyze_and_update(frame)
    }
    
    fn has_model(&self) -> bool {
        self.has_model()
    }
    
    fn last_params(&self) -> Option<&IspParams> {
        self.last_params.as_ref()
    }
}

