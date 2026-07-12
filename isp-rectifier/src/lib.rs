//! ISP Rectifier - Neural ISP Parameter Controller
//!
//! A distilled neural network that predicts optimal ISP parameters
//! (WB gains, CCM, tone curve, zoom) from frame metadata.
//!
//! ## Usage
//!
//! ```rust
//! use isp_rectifier::{FrameMetadata, OptimizedInference};
//!
//! // Initialize
//! let mut optimizer = OptimizedInference::new("fusedispcontroller_int8.onnx", true)?;
//!
//! // Process frame metadata
//! let metadata = FrameMetadata { ... };
//! let params = optimizer.optimize(&metadata)?;
//!
//! // Apply to ISP registers
//! apply_to_registers(&params);
//! ```

#[cfg(feature = "onnx-runtime")]
mod inference;

mod register_injector;

mod types;

// Re-export main types
pub use types::*;

// Public exports
#[cfg(feature = "onnx-runtime")]
pub use inference::{OptimizedInference, TractInference};
pub use types::ISPRegisters;

#[cfg(feature = "onnx-runtime")]
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_metadata_feature_vector() {
        let metadata = FrameMetadata {
            histogram: vec![100; 256],
            cct: 6500.0,
            wb_gains: [1.2, 1.0, 0.9],
            ae: AutoExposure {
                exposure_time: 0.033,
                iso_gain: 2.0,
                target_brightness: 0.5,
            },
            af: AutoFocus {
                position: 0.5,
                sharpness: 0.8,
            },
            awb: AutoWhiteBalance {
                gains: [1.2, 1.0, 0.9],
                confidence: 0.9,
            },
            brightness: 0.6,
            contrast: 0.7,
            noise_level: 0.1,
            timestamp: 12345,
        };

        let (hist, meta) = metadata.to_feature_vector();
        assert_eq!(hist.len(), 256);
        assert_eq!(meta.len(), 11);
    }

    #[test]
    fn test_isp_params_clamp() {
        let mut params = ISPOptimizedParams {
            wb_r_gain: 15.0,
            wb_g_gain: -0.5,
            wb_b_gain: 2.0,
            ccm: [[3.0, -3.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            tone_curve_lut: vec![-0.5, 1.5, 0.5, 0.0, 0.5, 0.5, 0.5],
            zoom_factor: 10.0,
        };

        params.clamp(&RegisterLimits::default());

        assert_eq!(params.wb_r_gain, 10.0);
        assert_eq!(params.wb_g_gain, 0.1);
        assert_eq!(params.ccm[0][0], 2.0);
        assert_eq!(params.ccm[0][1], -2.0);
        assert_eq!(params.tone_curve_lut[0], 0.0);
        assert_eq!(params.tone_curve_lut[1], 1.0);
        assert_eq!(params.zoom_factor, 4.0);
    }
}
