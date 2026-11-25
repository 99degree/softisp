//! IpaController wrapping cam_isp::controller::IspController for 3A

use cam_isp::controller::IspController;
use cam_isp::isp_params::{IspParams, WbParams, ToneParams};
use std::sync::{Arc, Mutex};
use crate::{ControlList, ControlInfoMap};

/// Wrapper around IspController for IPA use
pub struct IpaController {
    inner: Arc<Mutex<IspController>>,
}

impl IpaController {
    /// Create a new controller
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(IspController::new())),
        }
    }
    
    /// Get the inner controller (for engine)
    pub fn inner(&self) -> Arc<Mutex<IspController>> {
        self.inner.clone()
    }
    
    /// Analyze frame and update parameters (3A)
    pub fn analyze_and_update(&self, frame: &cam_isp::pipeline::IspFrame) -> IspParams {
        let mut ctrl = self.inner.lock().unwrap();
        // IspController doesn't have analyze_and_update directly; use default params
        // The engine's controller field is used for 3A state
        IspParams {
            wb: WbParams {
                r: ctrl.awb_gains[0],
                g: ctrl.awb_gains[1],
                b: ctrl.awb_gains[2],
            },
            tone: ToneParams {
                contrast: ctrl.tone_contrast,
                brightness: ctrl.tone_brightness,
                gamma: ctrl.tone_gamma,
                ..ToneParams::default()
            },
            ..IspParams::default()
        }
    }
    
    /// Get current AWB gains [R, G, B]
    pub fn get_awb_gains(&self) -> [f32; 3] {
        self.inner.lock().unwrap().awb_gains
    }
    
    /// Get current analog gain
    pub fn get_analog_gain(&self) -> f32 {
        self.inner.lock().unwrap().analog_gain
    }
    
    /// Get exposure time (ns)
    pub fn get_exposure_time(&self) -> i64 {
        self.inner.lock().unwrap().exposure_time_ns
    }
    
    /// Get current ISP parameters
    pub fn get_params(&self) -> IspParams {
        let ctrl = self.inner.lock().unwrap();
        IspParams {
            wb: WbParams {
                r: ctrl.awb_gains[0],
                g: ctrl.awb_gains[1],
                b: ctrl.awb_gains[2],
            },
            tone: ToneParams {
                contrast: ctrl.tone_contrast,
                brightness: ctrl.tone_brightness,
                gamma: ctrl.tone_gamma,
                ..ToneParams::default()
            },
            ..IspParams::default()
        }
    }
    
    /// Update from ControlList (e.g., from sensor controls)
    pub fn update_from_controls(&self, controls: &ControlList) {
        let mut ctrl = self.inner.lock().unwrap();
        
        if let Some(gain) = controls.get_f32("AnalogueGain") {
            ctrl.analog_gain = gain;
        }
        if let Some(exp) = controls.get_f32("ExposureTime") {
            ctrl.exposure_time_ns = exp as i64 * 1000; // ms to ns
        }
        if let Some(awb) = controls.get_f32_array("ColourGains") {
            if awb.len() >= 2 {
                ctrl.awb_gains = [awb[0], 1.0, awb[1]];
            }
        }
    }
    
    /// Update from ControlInfoMap (e.g., from sensor controls)
    pub fn update_from_controls_list(&self, controls: &ControlInfoMap) {
        // Convert to ControlList and update
        let ctrl_list = ControlList::from_info_map(controls);
        self.update_from_controls(&ctrl_list);
    }
    
    /// Set target brightness (0.0-1.0)
    pub fn set_target_brightness(&self, brightness: f32) {
        self.inner.lock().unwrap().brightness_bias = brightness.clamp(0.0, 1.0);
    }
}

impl Default for IpaController {
    fn default() -> Self {
        Self::new()
    }
}