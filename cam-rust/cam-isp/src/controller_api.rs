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
                    **c = crate::neural_controller::NeuralController::with_model(model_path);
                    c.has_model()
                }
                #[cfg(not(feature = "rectifier"))]
                {
                    let _ = model_path;
                    false
                }
            }
        }
    }
}

impl Default for Controller {
    fn default() -> Self {
        Self::rule_based()
    }
}
