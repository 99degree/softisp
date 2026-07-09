//! Controller API — unified interface for ISP parameter controllers.
//!
//! Both `IspController` (rule-based) and `NeuralController` (neural + fallback)
//! implement this trait, allowing UnifiedPipeline to use either interchangeably.

use crate::isp_params::IspParams;
use crate::pipeline::IspFrame;

/// Unified controller API for ISP parameter prediction.
pub trait ControllerApi {
    /// Analyze frame and produce ISP parameters.
    ///
    /// This is the main entry point. Implementations should:
    /// 1. Analyze frame statistics
    /// 2. Compute optimal parameters
    /// 3. Apply temporal smoothing (if applicable)
    fn analyze_and_update(&mut self, frame: &IspFrame) -> IspParams;
    
    /// Check if a neural model is loaded (for monitoring/debugging).
    fn has_model(&self) -> bool {
        false
    }
    
    /// Load a neural model (optional, not all controllers support this).
    fn load_model(&mut self, _model_path: &str) -> bool {
        false
    }
    
    /// Get last computed parameters without re-analyzing.
    fn last_params(&self) -> Option<&IspParams> {
        None
    }
}

/// Wrapper enum for concrete controller types.
pub enum Controller {
    /// Rule-based controller (always available).
    RuleBased(Box<crate::isp_controller::IspController>),
    /// Neural controller with fallback (requires `rectifier` feature).
    Neural(Box<crate::neural_controller::NeuralController>),
}

impl Controller {
    /// Create rule-based controller.
    pub fn rule_based() -> Self {
        Self::RuleBased(Box::default())
    }
    
    /// Create neural controller (without model).
    pub fn neural() -> Self {
        Self::Neural(Box::default())
    }
    
    /// Create neural controller with model.
    pub fn neural_with_model(_model_path: &str) -> Self {
        #[cfg(feature = "rectifier")]
        { Self::Neural(Box::new(crate::neural_controller::NeuralController::with_model(model_path))) }
        #[cfg(not(feature = "rectifier"))]
        { Self::Neural(Box::default()) }
    }
}

impl ControllerApi for Controller {
    fn analyze_and_update(&mut self, frame: &IspFrame) -> IspParams {
        match self {
            Self::RuleBased(c) => c.analyze_and_update(frame).clone(),
            Self::Neural(c) => c.analyze_and_update(frame),
        }
    }
    
    fn has_model(&self) -> bool {
        match self {
            Self::RuleBased(_) => false,
            Self::Neural(c) => c.has_model(),
        }
    }
    
    fn load_model(&mut self, model_path: &str) -> bool {
        match self {
            Self::RuleBased(_) => false,
            Self::Neural(_c) => {
                #[cfg(feature = "rectifier")]
                {
                    **_c = crate::neural_controller::NeuralController::with_model(model_path);
                    _c.has_model()
                }
                #[cfg(not(feature = "rectifier"))]
                {
                    let _ = model_path;
                    false
                }
            }
        }
    }
    
    fn last_params(&self) -> Option<&IspParams> {
        match self {
            Self::RuleBased(c) => c.last_params(),
            Self::Neural(c) => c.last_params(),
        }
    }
}

impl Default for Controller {
    fn default() -> Self {
        Self::rule_based()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::IspFrame;
    use cam_types::FrameFormat;
    
    fn create_test_frame() -> IspFrame {
        IspFrame {
            params: IspParams::default(),
            width: 64,
            height: 48,
            data: vec![128; 64 * 48 * 2],
            format: FrameFormat::RawSensor,
            float_data: None,
            aux: None,
            timestamp_ns: 1000,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
        }
    }
    
    #[test]
    fn test_controller_rule_based() {
        let mut controller = Controller::rule_based();
        assert!(!controller.has_model());
        
        let frame = create_test_frame();
        let params = controller.analyze_and_update(&frame);
        
        assert!(params.wb.r > 0.0);
        assert!(params.wb.g > 0.0);
        assert!(params.wb.b > 0.0);
    }
    
    #[test]
    fn test_controller_neural() {
        let mut controller = Controller::neural();
        assert!(!controller.has_model()); // No model loaded
        
        let frame = create_test_frame();
        let params = controller.analyze_and_update(&frame);
        
        // Should fallback to rule-based
        assert!(params.wb.r > 0.0);
    }
    
    #[test]
    fn test_controller_default() {
        let mut controller = Controller::default();
        
        let frame = create_test_frame();
        let params = controller.analyze_and_update(&frame);
        
        assert!(params.wb.r > 0.0);
    }
    
    #[test]
    fn test_controller_load_model() {
        let mut controller = Controller::neural();
        
        // Try to load non-existent model (should fail gracefully)
        let result = controller.load_model("nonexistent.onnx");
        assert!(!result);
        assert!(!controller.has_model());
    }
    
    #[test]
    fn test_controller_last_params() {
        let mut controller = Controller::rule_based();
        
        // IspController has default params from init
        assert!(controller.last_params().is_some());
        
        let frame = create_test_frame();
        let params1 = controller.analyze_and_update(&frame);
        
        // After analysis, params should be updated
        let params2 = controller.last_params().unwrap();
        assert_eq!(params1.wb.r, params2.wb.r);
    }
    
    #[test]
    fn test_controller_multiple_frames() {
        let mut controller = Controller::rule_based();
        let frame = create_test_frame();
        
        // Process multiple frames
        let params1 = controller.analyze_and_update(&frame);
        let params2 = controller.analyze_and_update(&frame);
        
        // Params should be similar (same input)
        assert!((params1.wb.r - params2.wb.r).abs() < 0.1);
    }
    
    #[test]
    fn test_controller_api_trait() {
        fn process_frame<C: ControllerApi>(controller: &mut C, frame: &IspFrame) -> IspParams {
            controller.analyze_and_update(frame)
        }
        
        let mut controller = Controller::rule_based();
        let frame = create_test_frame();
        
        let params = process_frame(&mut controller, &frame);
        assert!(params.wb.r > 0.0);
    }
}
