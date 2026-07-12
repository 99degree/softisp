//! NPU Offload Layer
//!
//! Bridges SoftISP to SoC-specific NPU accelerators:
//!
//! - Qualcomm HTA (via SNPE or QNN)
//! - MediaTek APU (via Neuropilot 2.0)
//! - Samsung NPU
//! - HiSilicon NPU
//!
//! Provides:
//! ```text
//!  ┌─────────────┐     ┌─────────────────┐
//!  │ SoftISP ISP│────▶│ Quantized INT8 │
//!  │ pipeline    │     │ CNN model       │
//!  └─────────────┘     └─────────────────┘
//!        ↑                ↓
//!  ┌───────────┐   ┌───────────┐
//!  │ V4L2     │   │ NPU      │
//!  │           │───│ offload  │
//!  └───────────┘   └───────────┘
//! ```
//!
//! Key: residual blocks and thumbnail-scale tasks benefit most.

use log::info;
use std::collections::HashMap;
use std::sync::Arc;

/// NPU accelerator vendor
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NpuVendor {
    /// Qualcomm Hexagon HTA
    Qualcomm,
    /// MediaTek APU (Neuropilot)
    MediaTek,
    /// Samsung NPU
    Samsung,
    /// HiSilicon NPU (Ascend)
    HiSilicon,
    /// Huawei NPU
    Huawei,
    /// Google EdgeTPU
    Google,
}

impl std::fmt::Display for NpuVendor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Qualcomm => write!(f, "Qualcomm HTA"),
            Self::MediaTek => write!(f, "MediaTek APU"),
            Self::Samsung => write!(f, "Samsung NPU"),
            Self::HiSilicon => write!(f, "HiSilicon NPU"),
            Self::Huawei => write!(f, "Huawei NPU"),
            Self::Google => write!(f, "Google EdgeTPU"),
        }
    }
}

impl NpuVendor {
    pub fn from_soc(soc: &str) -> Option<Self> {
        let soc_lower = soc.to_lowercase();
        match soc_lower.as_str() {
            s if s.contains("sm") || s.contains("sdm") => Some(Self::Qualcomm),
            s if s.contains("dimensity") || s.contains("mediatek") => Some(Self::MediaTek),
            s if s.contains("exynos") || s.contains("samsung") => Some(Self::Samsung),
            s if s.contains("kirin") || s.contains("hisilicon") => Some(Self::HiSilicon),
            s if s.contains("hi") || s.contains("huawei") => Some(Self::Huawei),
            s if s.contains("google") || s.contains("edge") => Some(Self::Google),
            _ => None,
        }
    }
}

/// NPU engine (FFI handle)
#[derive(Debug)]
pub enum NpuEngine {
    /// Qualcomm QNN (via qnn.h)
    Qnn(*mut std::ffi::c_void),
    /// MediaTek Neuropilot (via mediatek_npu.h)
    Neuropilot(*mut std::ffi::c_void),
    /// Samsung NPU SDK
    Samsung(*mut std::ffi::c_void),
    /// HiSilicon Ascend SDK
    HiSilicon(*mut std::ffi::c_void),
}

/// INT8 quantized model
#[derive(Debug)]
pub struct NpuModel {
    /// Vendor-specific model pointer
    _model_ptr: *mut std::ffi::c_void,
    /// Vendor that created this model
    vendor: NpuVendor,
    /// Input shape (NCHW)
    input_shape: Vec<u32>,
    /// Input channel order (BGR, RGB, BGRA)
    _input_layout: String,
    /// Output shape (NCHW)
    _output_shape: Vec<u32>,
    /// Scale factor for dequantization
    _output_scale: f32,
    /// Zero point for dequantization
    _output_zero: i8,
    /// Model FLOPS
    flops: f64,
    /// Model size (MB)
    _model_size: f32,
}

/// Offload task descriptor
pub struct NpuJob {
    /// NPU engine to use
    engine: Arc<NpuEngine>,
    /// Quantized model
    model: Arc<NpuModel>,
    /// Input tensor ptr
    input_ptr: *mut u8,
    /// Input quantized to INT8
    _input_scale: f32,
    /// Input quant zero point
    _input_zero: i8,
    /// Output tensor ptr
    output_ptr: *mut u8,
    /// On completion callback
    callback: Box<dyn FnOnce(Result<(), NpuError>) + Send>,
    /// Hang timeout ms
    _timeout_secs: u32,
}

/// NPU error category
#[derive(Debug, Clone)]
pub enum NpuError {
    /// Model compilation failed
    CompileError(String),
    /// Runtime execution timeout
    Timeout(String),
    /// Memory allocation failure
    MemoryError(String),
    /// Unsupported model
    Unsupported(String),
    /// API call failed
    ApiError(String),
    /// Integration mismatch
    IntegrationError(String),
    /// Quantization error
    QuantizationError(String),
    /// Generic backend error
    BackendError(String),
}

impl std::fmt::Display for NpuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CompileError(s) => write!(f, "NPU compile error: {}", s),
            Self::Timeout(s) => write!(f, "NPU timeout: {}", s),
            Self::MemoryError(s) => write!(f, "NPU memory error: {}", s),
            Self::Unsupported(s) => write!(f, "NPU unsupported: {}", s),
            Self::ApiError(s) => write!(f, "NPU API error: {}", s),
            Self::IntegrationError(s) => write!(f, "NPU integration error: {}", s),
            Self::QuantizationError(s) => write!(f, "NPU quantization error: {}", s),
            Self::BackendError(s) => write!(f, "NPU backend error: {}", s),
        }
    }
}

impl std::error::Error for NpuError {}
/// NPU Offload Manager
pub struct NpuManager {
    /// Active NPU engines (indexed by vendor)
    _engines: HashMap<NpuVendor, Vec<Arc<NpuEngine>>>,
    /// Supported quantized models (id → instruction)
    models: HashMap<String, Arc<NpuModel>>,
    /// Device-specific configuration
    _config: NpuConfig,
    /// Total offload FLOPS executed
    offload_ops: std::sync::atomic::AtomicU64,
}

/// Configuration for NPU
pub struct NpuConfig {
    /// Maximum concurrent jobs
    pub max_jobs: usize,
    /// Hang timeout (seconds)
    pub timeout_secs: u32,
    /// Enable INT8 quantization
    pub enable_int8: bool,
}

impl Default for NpuConfig {
    fn default() -> Self {
        Self {
            max_jobs: 4,
            timeout_secs: 3,
            enable_int8: true,
        }
    }
}

impl NpuManager {
    /// Create new manager
    pub fn new(config: NpuConfig) -> Self {
        Self {
            _engines: HashMap::new(),
            models: HashMap::new(),
            _config: config,
            offload_ops: std::sync::atomic::AtomicU64::new(0),
        }
    }

    /// Detect available NPU hardware
    pub fn detect_hardware(&mut self) -> Vec<NpuVendor> {
        let mut vendors = Vec::new();

        // Query /proc/cpuinfo or device-tree
        if let Ok(cpuinfo) = std::fs::read_to_string("/proc/cpuinfo") {
            let soc_line = cpuinfo.lines().find(|l| l.contains("Hardware"));
            if let Some(soc_line) = soc_line {
                if let Some(soc) = soc_line.split(':').nth(1) {
                    if let Some(vendor) = NpuVendor::from_soc(soc.trim()) {
                        vendors.push(vendor);
                    }
                }
            }
        }

        // Query proprietary APIs
        if is_qualcomm() {
            vendors.push(NpuVendor::Qualcomm);
        }
        if is_mediatek() {
            vendors.push(NpuVendor::MediaTek);
        }
        if is_samsung() {
            vendors.push(NpuVendor::Samsung);
        }

        info!("NPU hardware detected: {:?}", vendors);
        vendors
    }

    /// Load quantified model
    pub fn load_model(
        &mut self,
        vendor: NpuVendor,
        model_id: &str,
        _model_data: &[u8],
        _compile_opts: Option<HashMap<String, String>>,
    ) -> Result<Arc<NpuModel>, NpuError> {
        // In production: SAFETY-free FFI wrapper using bindgen generated bindings
        #[allow(unused)]
        match vendor {
            NpuVendor::Qualcomm => { /* qnn_compile_model */ }
            NpuVendor::MediaTek => { /* mediatek_npu_compile */ }
            NpuVendor::Samsung => { /* samsung_npu_compile */ }
            _ => {
                return Err(NpuError::Unsupported(format!(
                    "Model import not supported for vendor {}",
                    vendor
                )))
            }
        }

        // For now: simulate successful model
        let model = Arc::new(NpuModel {
            _model_ptr: std::ptr::null_mut(),
            vendor,
            input_shape: vec![1, 384, 384, 3], // HWC layout
            _input_layout: "RGB".into(),
            _output_shape: vec![1, 256, 256, 16],
            _output_scale: 0.028f32,
            _output_zero: -128,
            flops: 4.5e9,
            _model_size: 1.8,
        });
        self.models.insert(model_id.into(), model.clone());
        Ok(model)
    }

    /// Execute offloaded inference
    pub fn offload_inference(&self, job: NpuJob) -> Result<(), NpuError> {
        let start_us = std::time::Instant::now();
        let engine = &*job.engine;
        let model = job.model;
        let input_ptr = job.input_ptr;
        let output_ptr = job.output_ptr;
        let callback = job.callback;

        info!(
            "NPU offload: {} {:?} input={:?}",
            model.vendor, model.input_shape, input_ptr
        );

        // SAFETY: hw-boundary only. Real production uses safe binding-generated API.
        match engine {
            NpuEngine::Qnn(_) => unsafe { simulate_qnn_execution(&model, input_ptr, output_ptr) },
            NpuEngine::Neuropilot(_) => unsafe {
                simulate_mtk_execution(&model, input_ptr, output_ptr)
            },
            NpuEngine::Samsung(_) => unsafe {
                simulate_samsung_execution(&model, input_ptr, output_ptr)
            },
            NpuEngine::HiSilicon(_) => Err(NpuError::Unsupported(
                "HiSilicon NPU not yet supported".into(),
            )),
        }?;

        let elapsed_us = start_us.elapsed().as_micros();
        self.offload_ops
            .fetch_add(model.flops as u64, std::sync::atomic::Ordering::Relaxed);
        info!(
            "NPU offload complete: {} us, {:.1} GFLOPS, {:.1} GFLOPS/sec",
            elapsed_us,
            model.flops / 1e9,
            model.flops / elapsed_us as f64
        );

        (callback)(Ok(()));
        Ok(())
    }

    /// Get statistics
    pub fn stats(&self) -> NpuStats {
        NpuStats {
            total_ops: self.offload_ops.load(std::sync::atomic::Ordering::Relaxed),
        }
    }
}

/// FFI simulation stub
unsafe fn simulate_qnn_execution(
    _model: &NpuModel,
    _input_ptr: *mut u8,
    _output_ptr: *mut u8,
) -> Result<(), NpuError> {
    // Simulate: transfer input to DSP, run QNN model, transfer output
    Ok(())
}

#[allow(dead_code)]
unsafe fn simulate_mtk_execution(
    _model: &NpuModel,
    _input_ptr: *mut u8,
    _output_ptr: *mut u8,
) -> Result<(), NpuError> {
    Ok(()) // stub
}

#[allow(dead_code)]
unsafe fn simulate_samsung_execution(
    _model: &NpuModel,
    _input_ptr: *mut u8,
    _output_ptr: *mut u8,
) -> Result<(), NpuError> {
    Ok(()) // stub
}

/// Detect Qualcomm
fn is_qualcomm() -> bool {
    cfg!(target_arch = "aarch64") && cfg!(target_os = "android")
}

/// Detect MediaTek
fn is_mediatek() -> bool {
    if let Ok(cpuinfo) = std::fs::read_to_string("/proc/cpuinfo") {
        cpuinfo.lines().any(|l| l.contains("MT6"))
    } else {
        false
    }
}

/// Detect Samsung
fn is_samsung() -> bool {
    if let Ok(cpuinfo) = std::fs::read_to_string("/proc/cpuinfo") {
        cpuinfo.lines().any(|l| l.contains("exynos"))
    } else {
        false
    }
}

/// Statistics
#[derive(Debug, Clone)]
pub struct NpuStats {
    /// Total GFLOPS executed
    pub total_ops: u64,
}

/// Quantization manager
pub struct QuantizationManager {
    managers: HashMap<String, NpuManager>,
}

impl Default for QuantizationManager {
    fn default() -> Self {
        Self::new()
    }
}

impl QuantizationManager {
    pub fn new() -> Self {
        Self {
            managers: HashMap::new(),
        }
    }

    pub fn manager(&mut self, soc_name: &str) -> &mut NpuManager {
        let config = NpuConfig {
            max_jobs: 1,
            timeout_secs: 2,
            enable_int8: true,
        };

        self.managers.entry(soc_name.into()).or_insert_with(|| {
            let mut manager = NpuManager::new(config);
            let _ = manager.detect_hardware();
            manager
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_npu_vendor_from_soc() {
        assert_eq!(NpuVendor::from_soc("SDM865"), Some(NpuVendor::Qualcomm));
        assert_eq!(
            NpuVendor::from_soc("dimensity_9000"),
            Some(NpuVendor::MediaTek)
        );
        assert_eq!(NpuVendor::from_soc("exynos_5123"), Some(NpuVendor::Samsung));
        assert_eq!(NpuVendor::from_soc("unknown"), None);
    }

    #[test]
    fn test_npu_config_default() {
        let _cfg = NpuConfig::default();
        assert_eq!(NpuConfig::default().max_jobs, 4);
        assert_eq!(NpuConfig::default().timeout_secs, 3);
        assert!(NpuConfig::default().enable_int8);
    }

    #[test]
    fn test_npu_manager_initial_state() {
        let _mgr = NpuManager::new(NpuConfig::default());
    }

    #[test]
    fn test_load_model_adds_entry() {
        let mut mgr = NpuManager::new(NpuConfig::default());
        let vendor = NpuVendor::Qualcomm;
        let result = mgr.load_model(vendor, "test", &[], None);
        assert!(result.is_ok());
    }

    #[test]
    fn test_load_model_unsupported_vendor() {
        let mut mgr = NpuManager::new(NpuConfig::default());
        let result = mgr.load_model(NpuVendor::Huawei, "test", &[], None);
        assert!(result.is_err());
    }

    #[test]
    fn test_offload_ops_counter() {
        let mgr = NpuManager::new(NpuConfig::default());
        assert_eq!(mgr.stats().total_ops, 0);
        // Would increment in real offload
    }

    #[test]
    fn test_quantization_manager() {
        let mut qm = QuantizationManager::new();
        let _mgr = qm.manager("test_soc");
    }

    #[test]
    fn test_npu_vendor_display() {
        assert_eq!(format!("{}", NpuVendor::Qualcomm), "Qualcomm HTA");
        assert_eq!(format!("{}", NpuVendor::MediaTek), "MediaTek APU");
        assert_eq!(format!("{}", NpuVendor::Samsung), "Samsung NPU");
    }
}
