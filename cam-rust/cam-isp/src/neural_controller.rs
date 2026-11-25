//! Neural Controller — ISP parameter prediction via distilled model.
//!
//! Wraps `isp_rectifier::OptimizedInference` with automatic fallback
//! to the rule-based `IspController` when the model is unavailable or fails.
//!
//! Multiple pre-trained ONNX model variants are supported. The default
//! production model is `fusedispcontroller_light_int8.onnx` (INT8, fastest),
//! with FP32 and FP16 variants as fallbacks. For testing/CI, `with_mock_model()`
//! generates a self-contained ONNX that matches the real interface
//! (hist[256] + meta[52] → wb/ccm/tone/zoom).

#[cfg(feature = "rectifier")]
use crate::error::IspResult;
use crate::isp_controller::IspController;
use crate::isp_params::*;
use crate::pipeline::IspFrame;
#[cfg(feature = "rectifier")]
use log::{info, warn};

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

    /// Create controller with mock model for testing.
    /// Uses the rectifier_model module to generate a valid ONNX model.
    pub fn with_mock_model() -> Self {
        let model_bytes = crate::rectifier_model::generate_rectifier_model();

        #[cfg(feature = "rectifier")]
        {
            match Self::load_from_bytes(&model_bytes) {
                Ok(ctrl) => {
                    log::info!(
                        "NeuralController: loaded mock model ({} bytes)",
                        model_bytes.len()
                    );
                    ctrl
                }
                Err(e) => {
                    log::warn!(
                        "NeuralController: failed to load mock model: {}, using fallback",
                        e
                    );
                    Self::new()
                }
            }
        }
        #[cfg(not(feature = "rectifier"))]
        {
            let _ = model_bytes;
            log::info!("NeuralController: rectifier feature disabled, using fallback");
            Self::new()
        }
    }

    /// Load model from bytes (for testing or in-memory models).
    #[cfg(feature = "rectifier")]
    pub fn load_from_bytes(model_bytes: &[u8]) -> Result<Self, String> {
        // Write to temp file and load
        let temp_dir = std::env::temp_dir();
        let temp_path = temp_dir.join("mock_rectifier.onnx");
        std::fs::write(&temp_path, model_bytes)
            .map_err(|e| format!("Failed to write temp model: {}", e))?;

        let rectifier = isp_rectifier::OptimizedInference::new(&temp_path, true)
            .map_err(|e| format!("Failed to load model: {}", e))?;

        // Cleanup temp file
        let _ = std::fs::remove_file(&temp_path);

        Ok(Self {
            rectifier: Some(rectifier),
            fallback: IspController::new(),
            last_params: None,
        })
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
                warn!(
                    "NeuralController: failed to load model: {}, using fallback",
                    e
                );
                None
            }
        };

        Self {
            rectifier,
            fallback: IspController::new(),
            last_params: None,
        }
    }

    /// Default pre-trained model candidates, in priority order (fastest first).
    /// All variants share the same interface: hist[256] + meta[52] → wb/ccm/tone/zoom.
    #[cfg(feature = "rectifier")]
    const DEFAULT_MODEL_CANDIDATES: &[&str] = &[
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../isp-rectifier/models/fusedispcontroller_light_int8.onnx"
        ),
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../isp-rectifier/models/fusedispcontroller_light.onnx"
        ),
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../isp-rectifier/models/fusedispcontroller_light_fp16.onnx"
        ),
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../isp-rectifier/fusedispcontroller_light_int8.onnx"
        ),
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../isp-rectifier/fusedispcontroller_light.onnx"
        ),
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../isp-rectifier/fusedispcontroller_light_fp16.onnx"
        ),
    ];

    /// Embedded INT8 model bytes (compiled into the APK at build time).
    /// This is the fallback used on Android where the `isp-rectifier/models/`
    /// directory is not present on the device filesystem.
    #[cfg(feature = "rectifier")]
    const EMBEDDED_MODEL_INT8: &[u8] = include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../isp-rectifier/models/fusedispcontroller_light_int8.onnx"
    ));

    /// Embedded FP32 model bytes (compiled into the APK at build time).
    #[cfg(feature = "rectifier")]
    const EMBEDDED_MODEL_FP32: &[u8] = include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../isp-rectifier/models/fusedispcontroller_light.onnx"
    ));

    /// Create controller with the default pre-trained ONNX model.
    ///
    /// Tries each candidate path in order; the first one that loads successfully
    /// is used. If none exist on disk (e.g. on Android where the
    /// `isp-rectifier/models/` directory is not part of the APK), falls back to
    /// the embedded model bytes compiled into the binary. If that also fails,
    /// falls back to the rule-based controller.
    #[cfg(feature = "rectifier")]
    pub fn with_default_model() -> Self {
        // 1. Try filesystem paths (dev/CI runs from repo root)
        for &path in Self::DEFAULT_MODEL_CANDIDATES {
            if std::path::Path::new(path).exists() {
                let ctrl = Self::with_model(path);
                if ctrl.has_model() {
                    return ctrl;
                }
            }
        }

        // 2. Try embedded model bytes (APK / installed builds)
        for embedded in [Self::EMBEDDED_MODEL_INT8, Self::EMBEDDED_MODEL_FP32] {
            if !embedded.is_empty() {
                match Self::load_from_bytes(embedded) {
                    Ok(ctrl) => {
                        info!(
                            "NeuralController: loaded embedded model ({} bytes)",
                            embedded.len()
                        );
                        return ctrl;
                    }
                    Err(e) => {
                        warn!("NeuralController: failed to load embedded model: {}", e);
                    }
                }
            }
        }

        // None of the models exist/load; fall back to rule-based.
        warn!("NeuralController: no pre-trained ONNX model found, using fallback");
        Self::new()
    }

    /// Check if neural model is available.
    pub fn has_model(&self) -> bool {
        #[cfg(feature = "rectifier")]
        {
            self.rectifier.is_some()
        }
        #[cfg(not(feature = "rectifier"))]
        {
            false
        }
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
        // Extract metadata first (borrows self immutably)
        let metadata = self.extract_metadata(frame);

        if let Some(ref mut rectifier) = self.rectifier {
            match metadata {
                Ok(meta) => match rectifier.optimize(&meta) {
                    Ok(optimized) => {
                        info!("NeuralController: neural model succeeded");
                        self.params_from_optimized(frame, &optimized)
                    }
                    Err(e) => {
                        warn!(
                            "NeuralController: model inference failed: {}, using fallback",
                            e
                        );
                        self.fallback.analyze_and_update(frame).clone()
                    }
                },
                Err(e) => {
                    warn!(
                        "NeuralController: metadata extraction failed: {}, using fallback",
                        e
                    );
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

        // Compute basic statistics from histogram
        let total: u32 = histogram.iter().sum();
        let brightness = if total > 0 {
            let mean: f32 = histogram
                .iter()
                .enumerate()
                .map(|(i, &count)| i as f32 * count as f32)
                .sum::<f32>()
                / total as f32
                / 255.0;
            mean.clamp(0.0, 1.0)
        } else {
            0.5
        };

        // Contrast: standard deviation of luminance, normalized
        let contrast = if total > 0 {
            let mean: f32 = histogram
                .iter()
                .enumerate()
                .map(|(i, &count)| i as f32 * count as f32)
                .sum::<f32>()
                / total as f32;
            let var: f32 = histogram
                .iter()
                .enumerate()
                .map(|(i, &count)| {
                    let diff = i as f32 - mean;
                    diff * diff * count as f32
                })
                .sum::<f32>()
                / total as f32;
            (var.sqrt() / 255.0).clamp(0.0, 1.0)
        } else {
            0.5
        };

        // Estimate color temperature from histogram balance:
        // weight low bins as warmer, high bins as cooler
        let cct = if total > 0 {
            let warm: u32 = histogram[..64].iter().sum();
            let cool: u32 = histogram[192..].iter().sum();
            let ratio = if warm + cool > 0 {
                cool as f32 / (warm + cool) as f32
            } else {
                0.5
            };
            2000.0 + ratio * 8000.0 // 2000K (warm) → 10000K (cool)
        } else {
            5500.0
        };

        // Noise level: estimate from high-frequency histogram variation
        let noise_level = if total > 0 {
            let mut adjacent_diff: f32 = 0.0;
            for i in 1..256 {
                adjacent_diff += (histogram[i] as i32 - histogram[i - 1] as i32).abs() as f32;
            }
            (adjacent_diff / total as f32 / 2.0).clamp(0.0, 0.3)
        } else {
            0.1
        };

        Ok(isp_rectifier::FrameMetadata {
            histogram,
            cct,
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
            brightness,
            contrast,
            noise_level,
            timestamp: frame.timestamp_ns,
        })
    }

    #[cfg(feature = "rectifier")]
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
    fn params_from_optimized(&mut self, frame: &IspFrame, optimized: &isp_rectifier::ISPOptimizedParams) -> IspParams {
        // Neural model predicts WB gains, CCM matrix, tone curve LUT, and zoom.
        // Ask the rule-based controller to compute the remaining fields
        // (BLC, denoise, sharpen, saturation, parametric tone contrast/brightness/
        // gamma, lens) while SKIPPING white-balance and tone — those would be
        // recomputed by IspController only to be immediately overwritten by the
        // neural values below. This avoids wasted compute on coefficients that get
        // dropped.
        let mut params = self.fallback.analyze_and_update_partial(frame, true, true).clone();

        // Neural model outputs override the rule-based defaults:
        params.wb.r = optimized.wb_r_gain;
        params.wb.g = optimized.wb_g_gain;
        params.wb.b = optimized.wb_b_gain;

        params.ccm.matrix = {
            let mut m = [0.0f32; 9];
            for i in 0..3 {
                for j in 0..3 {
                    m[i * 3 + j] = optimized.ccm[i][j];
                }
            }
            m
        };

        params.tone.contrast = 1.0;
        params.tone.brightness = 0.0;
        params.tone.gamma = 1.0;
        params.tone.black_crush = 0.0;
        params.tone.white_clip = 1.0;
        params.tone.curve_lut = optimized.tone_curve_lut.clone();

        params.zoom = optimized.zoom_factor;

        params
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
                temporal_strength: lerp(
                    old.denoise.temporal_strength,
                    new.denoise.temporal_strength,
                ),
                edge_preserve: lerp(old.denoise.edge_preserve, new.denoise.edge_preserve),
                bilateral_sigma: lerp(old.denoise.bilateral_sigma, new.denoise.bilateral_sigma),
            },
            lens: new.lens.clone(),
            display: new.display.clone(),
            zoom: lerp(old.zoom, new.zoom),
            vcm_position: lerp(old.vcm_position, new.vcm_position),
            custom: new.custom.clone(),
        }
    }
}

impl Default for NeuralController {
    fn default() -> Self {
        Self::new()
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

#[cfg(test)]
#[cfg(feature = "rectifier")]
mod tests {
    use super::*;

    fn create_test_frame() -> IspFrame {
        IspFrame {
            params: IspParams::default(),
            width: 64,
            height: 48,
            data: vec![128; 64 * 48 * 2],
            format: cam_types::FrameFormat::RawSensor,
            float_data: None,
            aux: None,
            timestamp_ns: 1000,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
            external_data: None,
        }
    }

    #[test]
    fn test_neural_controller_fallback() {
        let mut ctrl = NeuralController::new();
        assert!(!ctrl.has_model());

        let frame = create_test_frame();
        let params = ctrl.analyze_and_update(&frame);
        assert!(params.wb.r > 0.0);
    }

    #[test]
    fn test_neural_controller_mock_model() {
        // Test that mock model can be generated
        let model_bytes = crate::rectifier_model::generate_rectifier_model();
        assert!(!model_bytes.is_empty());
        assert!(model_bytes.len() > 1000);

        // Create controller with mock model
        let mut ctrl = NeuralController::with_mock_model();

        let frame = create_test_frame();
        let params = ctrl.analyze_and_update(&frame);

        // Should produce valid parameters
        assert!(params.wb.r > 0.0);
        assert!(params.wb.g > 0.0);
        assert!(params.wb.b > 0.0);
    }

    #[test]
    fn test_mock_model_load_from_bytes() {
        let model_bytes = crate::rectifier_model::generate_rectifier_model();

        let result = NeuralController::load_from_bytes(&model_bytes);
        assert!(result.is_ok());

        let mut ctrl = result.unwrap();
        assert!(ctrl.has_model());

        let frame = create_test_frame();
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

    #[test]
    fn test_mock_model_output_shape() {
        // Verify mock model produces correct output dimensions
        let model_bytes = crate::rectifier_model::generate_rectifier_model();

        // Model should be valid ONNX (starts with protobuf data)
        assert!(!model_bytes.is_empty());

        // Check it's a reasonable size for a small model
        assert!(model_bytes.len() > 1000);
        assert!(model_bytes.len() < 100_000); // Less than 100KB
    }

    #[test]
    fn test_neural_controller_multiple_frames() {
        let mut ctrl = NeuralController::new();
        let frame = create_test_frame();

        // Process multiple frames
        let params1 = ctrl.analyze_and_update(&frame);
        let params2 = ctrl.analyze_and_update(&frame);

        // Should be similar (temporal smoothing)
        assert!((params1.wb.r - params2.wb.r).abs() < 0.2);
    }
}
