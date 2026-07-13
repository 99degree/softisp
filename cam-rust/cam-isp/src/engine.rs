//! Backend-agnostic inference engine for the ISP pipeline.
//! Ported from com.camcore.isp.engine.IspEngine

use cam_types::ToneParams;
use log::info;
use std::collections::BTreeMap;
use std::fmt;
use std::sync::Mutex;

use crate::controller::IspController;
use crate::pipeline::{IspBlock, IspFrame};

/// Output pixel format selector.
///
/// Controls whether the engine converts the MNN output to a specific byte
/// format (BGRA, RGBA, ARGB, etc.) or returns raw bytes with no conversion.
///
/// # Format Table
///
/// | Format       | ONNX shape     | Data type  | B/px | ONNX subgraph                    |
/// |--------------|----------------|------------|------|----------------------------------|
/// | `FloatRgb`   | `[1,3,H,W]`    | f32×3      | 12   | identity Mul(1.0)                |
/// | `FloatBgra`  | `[1,4,H,W]`    | f32×4      | 16   | Conv(1×1): B←R, G←G, R←B, A←255 |
/// | `PackedRgb`  | `[1,1,H,W/2]`  | INT32      |  2   | adjacent-pixel RGBA pack           |
/// | `Bgra`       | `[1,4,H,W]`    | f32×4      |  4   | Conv(1×1): B←R, G←G, R←B, A←255 |
/// | `Rgba`       | `[1,4,H,W]`    | f32×4      |  4   | Conv(1×1): R←R, G←G, B←B, A←255 |
/// | `Argb`       | `[1,4,H,W]`    | f32×4      |  4   | Conv(1×1): A←255, R←R, G←G, B←B |
/// | `Abgr`       | `[1,4,H,W]`    | f32×4      |  4   | Conv(1×1): A←255, B←B, G←G, R←R |
/// | `Rgb`        | `[1,3,H,W]`    | f32×3      |  3   | Conv(1×1): R←R, G←G, B←B        |
/// | `Bgr`        | `[1,3,H,W]`    | f32×3      |  3   | Conv(1×1): B←B, G←G, R←R        |
///
/// Notes:
/// - `Float*` and `PackedRgb` are model-native — ONNX graph outputs data in
///   exactly this layout, engine memcpys the raw bytes, zero copy.
/// - `Bgra`/`Rgba`/etc. (no `Float` prefix) also use the same Conv(1×1) ONNX
///   subgraph, producing `f32 [0,255]` values. Consumer truncates to u8.
/// - All Conv-based formats also multiply by 255 (weights include the scale).
/// - `PackedRgb` encodes two RGBA pixels per INT32: lower 16 bits are pixel0.R|G,
///   upper 16 bits are pixel1.B|A.
/// - Per default (`PackedRgb`), every frame is `[1,1,H,W/2]` INT32.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OutputFormat {
    /// Float RGB `[0,1]` — return raw f32×3 planar bytes.
    FloatRgb,
    /// Float BGRA `[0,255]` — return raw f32×4 planar bytes (bg4a).
    FloatBgra,
    /// INT32 packed — return raw packed bytes (R<<16|G<<8|B, 4B/pixel).
    PackedRgb,
    /// BGRA u8 (4 bytes/pixel). Always converted.
    #[default]
    Bgra,
    /// RGBA u8 (4 bytes/pixel). Always converted.
    Rgba,
    /// ARGB u8 (4 bytes/pixel). Always converted.
    Argb,
    /// ABGR u8 (4 bytes/pixel). Always converted.
    Abgr,
    /// RGB u8 (3 bytes/pixel). Always converted.
    Rgb,
    /// BGR u8 (3 bytes/pixel). Always converted.
    Bgr,
    /// Float16 RGB `[0,255]` — return raw f16×3 planar bytes. Halves GPU→CPU bandwidth.
    Float16Rgb,
    /// Float16 BGRA `[0,255]` — return raw f16×4 planar bytes. Halves GPU→CPU bandwidth.
    Float16Bgra,
}

impl OutputFormat {
    /// Model output channel count for this format.
    pub fn channel_count(self) -> usize {
        match self {
            Self::PackedRgb => 1,
            Self::FloatRgb | Self::Float16Rgb | Self::Rgb | Self::Bgr => 3,
            _ => 4,
        }
    }
    /// Bytes per pixel in the final output.
    pub fn bytes_per_pixel(self) -> usize {
        match self {
            Self::FloatRgb => 12,   // f32×3
            Self::FloatBgra => 16,  // f32×4
            Self::Float16Rgb => 6,  // f16×3
            Self::Float16Bgra => 8, // f16×4
            Self::PackedRgb => 2,   // INT32 per two pixels
            Self::Rgb | Self::Bgr => 3,
            _ => 4,
        }
    }
    /// Whether this format outputs float16 tensors (halves GPU→CPU bandwidth).
    pub fn is_fp16(self) -> bool {
        matches!(self, Self::Float16Rgb | Self::Float16Bgra)
    }
    /// ONNX elem_type for the output tensor: 10=FLOAT16, 1=FLOAT.
    pub fn onnx_elem_type(self) -> i32 {
        if self.is_fp16() {
            10
        } else {
            1
        }
    }
}

/// Default tone parameters for the ISP pipeline.
pub fn default_tone_params() -> ToneParams {
    ToneParams {
        contrast: 1.2,
        brightness: 0.05,
        gamma_recip: 2.2,
        saturation: 1.3,
        ..Default::default()
    }
}

/// Unified parameter bundle for ISP pipeline processing.
/// Instead of passing 15 positional args to process(), construct this once.
#[derive(Clone)]
pub struct ProcessParams<'a> {
    pub width: u32,
    pub height: u32,
    pub stride_width: u32,
    pub buf: &'a [u8],
    pub sensor_max: f32,
    pub target_width: u32,
    /// Output frame height (default: same as height). For pipelines with
    /// fused downscale, this is the actual output height (e.g., H/2 for stride=2).
    pub target_height: u32,
    pub ccm_matrix: Option<[f32; 9]>,
    pub tone_params: ToneParams,
    pub bayer_gains: Option<[f32; 4]>,
    pub awb_gains: Option<[f32; 3]>,
    pub bayer_pattern: i32,
    pub analog_gain: f32,
    pub scene_change: f32,
    pub lsc_gains: Option<&'a [f32]>,
    pub blc_values: Option<[f32; 4]>,
    pub warp_grid: Option<&'a [f32]>,
    /// Output format: PackedInt32 (raw) or Bgra (converted).
    pub output_format: OutputFormat,
    /// Capture timestamp in nanoseconds (from HAL/framework). Passed through to IspFrame.
    pub timestamp_ns: u64,
    /// Full ISP parameters from controller (WB, CCM, tone, etc.)
    pub isp_params: Option<crate::isp_params::IspParams>,
}

impl<'a> ProcessParams<'a> {
    /// Create params with minimal required fields; rest get defaults.
    pub fn new(width: u32, height: u32, buf: &'a [u8]) -> Self {
        Self {
            width,
            height,
            stride_width: width,
            buf,
            isp_params: None,
            sensor_max: 1023.0,
            target_width: width,
            target_height: height,
            ccm_matrix: None,
            tone_params: default_tone_params(),
            bayer_gains: None,
            awb_gains: None,
            bayer_pattern: 0,
            analog_gain: 1.0,
            scene_change: 0.0,
            output_format: OutputFormat::default(),
            timestamp_ns: 0,
            lsc_gains: None,
            blc_values: None,
            warp_grid: None,
        }
    }

    /// Fill ccm/bayer/awb from a controller reference.
    pub fn with_controller(mut self, ctrl: &IspController) -> Self {
        self.ccm_matrix = Some(ctrl.get_ccm());
        self.awb_gains = Some(ctrl.get_awb_gains());
        self.tone_params = ctrl.get_tone_params();
        self
    }
}

/// Runtime capabilities of a backend engine.
/// Queried before building pipelines to select optimal input mode.
#[derive(Debug, Clone)]
pub struct BackendCapabilities {
    /// Backend name (matches `IspEngine::backend_name()`).
    pub name: String,
    /// Whether the backend supports int16 native input directly.
    /// CPU, Vulkan, and OpenCL support int16; OpenGL may not.
    pub supports_native_int16: bool,
    /// Whether the backend supports fp16 storage for internal tensors.
    pub supports_fp16_storage: bool,
    /// Best guess at whether GPU acceleration is actually in use
    /// (vs CPU fallback in a container).
    pub has_gpu_acceleration: bool,
}

impl BackendCapabilities {
    /// Probe capabilities for a given backend name.
    /// Returns a conservative estimate based on known backend characteristics.
    pub fn probe(backend_name: &str) -> Self {
        let lower = backend_name.to_lowercase();
        let (native_int16, fp16, gpu) = if lower.contains("cpu") || lower.contains("neon") {
            (true, false, false)
        } else if lower.contains("vulkan") || lower.contains("opencl") || lower.contains("cl") {
            (true, true, true)
        } else {
            (false, false, false)
        };
        Self {
            name: backend_name.to_string(),
            supports_native_int16: native_int16,
            supports_fp16_storage: fp16,
            has_gpu_acceleration: gpu,
        }
    }

    /// Returns `true` if NativeInt16 mode is recommended for this backend.
    /// NativeInt16 uses `preserve_input_type=true` and `UnpackMode::NativeInt16`.
    pub fn recommend_native_int16(&self) -> bool {
        self.supports_native_int16
    }

    /// Returns `true` if PackedInt32 mode should be used (fallback).
    pub fn recommend_packed_int32(&self) -> bool {
        !self.supports_native_int16
    }
}

/// Factory for creating an IspEngine instance.
/// Engine factory — creates engine instances for a specific backend.
///
/// The `priority` field determines selection order (higher = preferred).
/// This is separate from `IspEngine::priority()` which returns the
/// runtime-computed priority of a specific instance.
pub struct EngineFactory {
    pub name: &'static str,
    pub priority: i32,
    pub create_fn: Box<dyn Fn() -> Box<dyn IspEngine> + Send>,
}

impl fmt::Debug for EngineFactory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("EngineFactory")
            .field("name", &self.name)
            .field("priority", &self.priority)
            .finish()
    }
}

/// Trait for an ISP processing engine.
///
/// Implementations compile an IspBlock chain into a fused model and run per-frame inference.
///
/// # Implementations
///
/// - `CpuEngine`: Pure CPU implementation (priority 70)
/// - `MnnEngine`: MNN-based GPU/NPU acceleration (priority 90)
/// - `OnnxEngine`: ONNX Runtime backend (priority 80)
///
/// # Lifecycle
///
/// ```text
/// 1. select_engine()  → Box<dyn IspEngine>
/// 2. engine.build(blocks, ...)  → compiles pipeline
/// 3. engine.process(params)  → IspFrame
/// ```
pub trait IspEngine: Send + Sync {
    /// Backend identifier (e.g., "CPU", "MNN/Vulkan", "ONNX")
    fn backend_name(&self) -> &'static str;

    /// Engine priority (higher = preferred). Used by select_engine().
    fn priority(&self) -> i32;

    /// Whether the engine has been built and is ready for inference.
    fn is_loaded(&self) -> bool;

    /// Downcast to `dyn Any` for engine-specific configuration before `build()`.
    ///
    /// Every engine MUST implement this (returning `self`) so callers can
    /// downcast to the concrete type. There is intentionally **no default**:
    /// a missing implementation would otherwise panic at runtime the first
    /// time a caller tries to downcast (see `integration.rs`).
    fn as_any(&self) -> &dyn std::any::Any;

    /// Mutable variant of [`IspEngine::as_any`].
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;

    /// Build the engine from a pipeline of blocks.
    ///
    /// Compiles the block chain into an executable model (ONNX/MNN/etc.)
    /// and prepares the engine for inference.
    ///
    /// # Arguments
    /// * `pipeline_head` - First block in the processing chain
    /// * `aux_blocks` - Additional blocks (stats, etc.)
    /// * `warp_block` - Optional geometric correction block
    /// * `opset_version` - ONNX opset version for model generation
    fn build(
        &mut self,
        pipeline_head: Box<dyn IspBlock>,
        aux_blocks: Vec<Box<dyn IspBlock>>,
        warp_block: Option<Box<dyn IspBlock>>,
        opset_version: i64,
    ) -> crate::error::IspResult<()>;

    /// Access the ISP controller for reading AWB/AE/tone state.
    fn controller(&self) -> &Mutex<IspController>;

    /// Process a raw frame through the pipeline.
    ///
    /// Takes raw sensor data and returns a processed IspFrame.
    /// The engine applies all configured blocks in sequence.
    fn process(&self, params: &ProcessParams) -> crate::error::IspResult<IspFrame>;
}

/// Global registry of engine factories.
///
/// Maintains a priority-ordered map of registered backends.
/// Thread-safe: all operations go through a Mutex.
static REGISTRY: std::sync::LazyLock<Mutex<EngineRegistry>> =
    std::sync::LazyLock::new(|| Mutex::new(EngineRegistry::new()));

/// Priority-ordered collection of engine factories.
/// Uses ``BTreeMap`` so iteration is always in priority order (ascending).
struct EngineRegistry {
    /// Map from priority → factory. Priority value is negated on insertion
    /// so iteration yields highest-priority first.
    factories: BTreeMap<i32, EngineFactory>,
    /// Counter to break ties for engines with equal priority.
    next_id: i32,
}

impl EngineRegistry {
    fn new() -> Self {
        Self {
            factories: BTreeMap::new(),
            next_id: 0,
        }
    }

    /// Insert a factory, generating a unique key from ``-priority`` + tiebreaker.
    /// Iterating ``factories.values()`` yields highest-priority first.
    fn insert(&mut self, factory: EngineFactory) {
        let key = -factory.priority * 10000 + self.next_id;
        self.next_id += 1;
        self.factories.insert(key, factory);
    }

    fn remove_by_name(&mut self, name: &str) {
        self.factories.retain(|_, f| f.name != name);
    }

    fn all(&self) -> impl Iterator<Item = &EngineFactory> {
        self.factories.values()
    }

    fn len(&self) -> usize {
        self.factories.len()
    }
}

/// Register an engine factory.
///
/// Called during `init()` to register available backends.
pub fn register_engine(factory: EngineFactory) {
    let mut registry = REGISTRY.lock().unwrap();
    let name = factory.name;
    let priority = factory.priority;
    registry.insert(factory);
    info!(
        "Registered engine: {} (priority={}, total={})",
        name,
        priority,
        registry.len()
    );
}

/// Unregister an engine factory by name.
///
/// Returns ``true`` if an engine was removed.
pub fn unregister_engine(name: &str) -> bool {
    let mut registry = REGISTRY.lock().unwrap();
    let len_before = registry.len();
    registry.remove_by_name(name);
    let removed = registry.len() < len_before;
    if removed {
        info!("Unregistered engine: {} (total={})", name, registry.len());
    }
    removed
}

/// Select the best available engine by priority.
///
/// Returns the highest-priority engine that can be created.
pub fn select_engine() -> Option<Box<dyn IspEngine>> {
    let registry = REGISTRY.lock().unwrap();
    for factory in registry.all() {
        let engine = (factory.create_fn)();
        info!(
            "Trying engine: {} (priority={})",
            factory.name, factory.priority
        );
        if engine.priority() > 0 {
            return Some(engine);
        }
    }
    None
}

/// Select a specific engine by name or ``"auto"`` for best available.
///
/// Name matching is case-insensitive and supports partial matches.
pub fn select_engine_by_name(name: &str) -> Option<Box<dyn IspEngine>> {
    if name == "auto" {
        return select_engine();
    }
    let registry = REGISTRY.lock().unwrap();
    let lower = name.to_lowercase();
    for factory in registry.all() {
        if factory.name.to_lowercase().contains(&lower) {
            let engine = (factory.create_fn)();
            info!(
                "Selected engine: {} (priority={})",
                factory.name, factory.priority
            );
            if engine.priority() > 0 {
                return Some(engine);
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_format_channel_count() {
        assert_eq!(OutputFormat::Rgb.channel_count(), 3);
        assert_eq!(OutputFormat::Rgba.channel_count(), 4);
        assert_eq!(OutputFormat::FloatRgb.channel_count(), 3);
        assert_eq!(OutputFormat::FloatBgra.channel_count(), 4);
        assert_eq!(OutputFormat::PackedRgb.channel_count(), 1);
        assert_eq!(OutputFormat::Float16Rgb.channel_count(), 3);
        assert_eq!(OutputFormat::Float16Bgra.channel_count(), 4);
    }

    #[test]
    fn test_output_format_bytes_per_pixel() {
        assert_eq!(OutputFormat::Rgb.bytes_per_pixel(), 3);
        assert_eq!(OutputFormat::Rgba.bytes_per_pixel(), 4);
        assert_eq!(OutputFormat::FloatRgb.bytes_per_pixel(), 12);
        assert_eq!(OutputFormat::FloatBgra.bytes_per_pixel(), 16);
        assert_eq!(OutputFormat::PackedRgb.bytes_per_pixel(), 2);
        assert_eq!(OutputFormat::Float16Rgb.bytes_per_pixel(), 6);
        assert_eq!(OutputFormat::Float16Bgra.bytes_per_pixel(), 8);
    }

    #[test]
    fn test_output_format_is_fp16() {
        assert!(!OutputFormat::Rgb.is_fp16());
        assert!(!OutputFormat::FloatRgb.is_fp16());
        assert!(OutputFormat::Float16Rgb.is_fp16());
        assert!(OutputFormat::Float16Bgra.is_fp16());
    }

    #[test]
    fn test_process_params_new() {
        let data = vec![0u8; 64];
        let p = ProcessParams::new(8, 8, &data);
        assert_eq!(p.width, 8);
        assert_eq!(p.height, 8);
        assert_eq!(p.buf.len(), 64);
    }

    #[test]
    fn test_default_tone_params() {
        let tp = default_tone_params();
        assert!(tp.contrast >= 0.0 && tp.contrast <= 2.0);
    }

    #[test]
    fn test_select_engine() {
        crate::init();
        let engine = select_engine();
        assert!(engine.is_some());
    }
}
