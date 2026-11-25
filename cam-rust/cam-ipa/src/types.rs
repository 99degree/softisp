//! Public types for cam-ipa stable API

use std::path::PathBuf;
use serde::{Deserialize, Serialize};
use std::os::fd::RawFd;
use cam_isp::profile::PipelineProfile;

/// Configuration for IPA context
#[derive(Debug, Clone)]
pub struct IpaConfig {
    /// Directory containing model files (algo.mnn, applier.mnn)
    pub model_dir: Option<PathBuf>,
    /// Input image width
    pub image_width: u32,
    /// Input image height
    pub image_height: u32,
    /// Output format
    pub format: IpaFormat,
    /// Number of output buffers to allocate
    pub num_output_buffers: u32,
    /// Backend engine to use
    pub backend: EngineBackend,
    /// Enable dual-GPU ping-pong inference
    pub enable_dual_gpu: bool,
    /// Stats inference interval (1 = every frame)
    pub stats_interval: u32,
    /// Pipeline profile preset (LITE, MED, FUSED, HEAVY, PRO, TEST, UNIFIED, HDR, YUV)
    pub profile: PipelineProfile,
    /// Enable NeuralController (isp-rectifier) instead of rule-based IspController
    pub neural_enabled: bool,
}

impl Default for IpaConfig {
    fn default() -> Self {
        Self {
            model_dir: None,
            image_width: 1920,
            image_height: 1080,
            format: IpaFormat::BGRA,
            num_output_buffers: 2,
            backend: EngineBackend::MnnVulkan,
            enable_dual_gpu: false,
            stats_interval: 1,
            profile: PipelineProfile::FUSED,
            neural_enabled: false,
        }
    }
}

/// Output format for processed frames
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IpaFormat {
    RGB,
    RGBA,
    BGRA,
    ARGB,
    ABGR,
    YUV,
    NV12,
    P010,
}

/// Backend engine type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EngineBackend {
    MnnVulkan,
    Cpu,
    OnnxRuntime,
}

/// Settings passed during initialization
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpaSettings {
    /// Optional configuration file path
    pub config_file: Option<String>,
    /// Sensor model name
    pub sensor_model: String,
}

/// Camera sensor information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpaSensorInfo {
    pub model: String,
    pub bits_per_pixel: u32,
    pub cfa_pattern: u32,
    pub active_area_size: Size,
    pub analog_crop: Rectangle,
    pub output_size: Size,
    pub pixel_rate: u64,
    pub min_line_length: u32,
    pub max_line_length: u32,
    pub min_frame_length: u32,
    pub max_frame_length: u32,
}

/// Configuration info for configure()
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpaConfigureInfo {
    pub sensor_controls: ControlInfoMap,
}

/// Initialization result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpaInitResult {
    pub ipa_controls: ControlInfoMap,
    pub ccm_enabled: bool,
}

/// Size (width/height)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Size {
    pub width: u32,
    pub height: u32,
}

/// Rectangle (x, y, width, height)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Rectangle {
    pub x: i32,
    pub y: i32,
    pub width: u32,
    pub height: u32,
}

/// Control list - simplified serializable version
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ControlList {
    // For MVP, use JSON-serializable map
    // Real implementation would use proper control IDs
    pub data: std::collections::HashMap<String, serde_json::Value>,
}

impl ControlList {
    pub fn new() -> Self {
        Self { data: std::collections::HashMap::new() }
    }
    
    pub fn set_f32(&mut self, key: &str, value: f32) {
        self.data.insert(key.to_string(), serde_json::Value::Number(serde_json::Number::from_f64(value as f64).unwrap()));
    }
    
    pub fn set_i32(&mut self, key: &str, value: i32) {
        self.data.insert(key.to_string(), serde_json::Value::Number(value.into()));
    }
    
    pub fn set_f32_array(&mut self, key: &str, values: &[f32]) {
        self.data.insert(key.to_string(), serde_json::Value::Array(values.iter().map(|v| serde_json::Value::Number(serde_json::Number::from_f64(*v as f64).unwrap())).collect()));
    }
    
    pub fn get_f32(&self, key: &str) -> Option<f32> {
        self.data.get(key).and_then(|v| v.as_f64()).map(|v| v as f32)
    }
    
    pub fn get_f32_array(&self, key: &str) -> Option<Vec<f32>> {
        self.data.get(key).and_then(|v| v.as_array()).map(|arr| arr.iter().filter_map(|v| v.as_f64().map(|f| f as f32)).collect())
    }
    
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Create ControlList from ControlInfoMap
    pub fn from_info_map(info: &ControlInfoMap) -> Self {
        let mut data = std::collections::HashMap::new();
        for (k, v) in &info.data {
            data.insert(k.clone(), v.clone());
        }
        Self { data }
    }
}

/// Control info map - metadata about available controls
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ControlInfoMap {
    pub data: std::collections::HashMap<String, serde_json::Value>,
}

/// Buffer trait for zero-copy abstraction
pub trait Buffer: Send + Sync {
    fn as_ptr(&self) -> *const u8;
    fn as_mut_ptr(&mut self) -> *mut u8;
    fn len(&self) -> usize;
    fn fd(&self) -> Option<RawFd>;
    fn is_ahb(&self) -> bool { false }
    fn ahb_ptr(&self) -> Option<*mut std::ffi::c_void> { None }
}

/// Callbacks for async events
pub struct IpaCallbacks {
    pub metadata_ready: Option<Box<dyn Fn(u32, &ControlList) + Send + Sync>>,
    pub frame_done: Option<Box<dyn Fn(u32, u32) + Send + Sync>>,
    pub set_sensor_controls: Option<Box<dyn Fn(&ControlList) + Send + Sync>>,
    pub set_isp_params: Option<Box<dyn Fn() + Send + Sync>>,
}

impl Default for IpaCallbacks {
    fn default() -> Self {
        Self {
            metadata_ready: None,
            frame_done: None,
            set_sensor_controls: None,
            set_isp_params: None,
        }
    }
}