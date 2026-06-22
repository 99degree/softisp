//! Backend-agnostic inference engine for the ISP pipeline.
//! Ported from com.camcore.isp.engine.IspEngine

use std::sync::Mutex;
use std::fmt;
use log::info;
use cam_types::ToneParams;

use crate::pipeline::{IspBlock, IspFrame};
use crate::controller::IspController;

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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputFormat {
    /// Float RGB [0,1] — return raw f32×3 planar bytes.
    FloatRgb,
    /// Float BGRA [0,255] — return raw f32×4 planar bytes (bg4a).
    FloatBgra,
    /// INT32 packed — return raw packed bytes (R<<16|G<<8|B, 4B/pixel).
    PackedRgb,
    /// BGRA u8 (4 bytes/pixel). Always converted.
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
}

impl Default for OutputFormat {
    fn default() -> Self { Self::Bgra }
}

impl OutputFormat {
    /// Model output channel count for this format.
    pub fn channel_count(self) -> usize {
        match self {
            Self::PackedRgb => 1,
            Self::FloatRgb | Self::Rgb | Self::Bgr => 3,
            _ => 4,
        }
    }
    /// Bytes per pixel in the final output.
    pub fn bytes_per_pixel(self) -> usize {
        match self {
            Self::FloatRgb => 12,     // f32×3
            Self::FloatBgra => 16,    // f32×4
            Self::PackedRgb => 2,     // INT32 per two pixels
            Self::Rgb | Self::Bgr => 3,
            _ => 4,
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
}

impl<'a> ProcessParams<'a> {
    /// Create params with minimal required fields; rest get defaults.
    pub fn new(width: u32, height: u32, buf: &'a [u8]) -> Self {
        Self {
            width,
            height,
            stride_width: width,
            buf,
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

/// Factory for creating an IspEngine instance.
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
/// Implementations compile an IspBlock chain into a fused model and run per-frame inference.
pub trait IspEngine: Send + Sync {
    fn backend_name(&self) -> &'static str;
    fn priority(&self) -> i32;
    fn is_loaded(&self) -> bool;
    /// Downcast to dyn Any for engine-specific configuration before build().
    fn as_any(&self) -> &dyn std::any::Any { unimplemented!() }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any { unimplemented!() }
    
    /// Build the engine from a pipeline of blocks.
    fn build(
        &mut self,
        pipeline_head: Box<dyn IspBlock>,
        aux_blocks: Vec<Box<dyn IspBlock>>,
        warp_block: Option<Box<dyn IspBlock>>,
        opset_version: i64,
    ) -> Result<(), String>;

    /// Access the ISP controller for reading AWB/AE/tone state.
    fn controller(&self) -> &Mutex<IspController>;

    /// Process a raw frame through the pipeline.
    fn process(&self, params: &ProcessParams) -> Result<IspFrame, String>;
}

/// Global registry of engine factories.
static REGISTRY: std::sync::LazyLock<Mutex<Vec<EngineFactory>>> =
    std::sync::LazyLock::new(|| Mutex::new(Vec::new()));

/// Register an engine factory.
pub fn register_engine(factory: EngineFactory) {
    let name = factory.name;
    let priority = factory.priority;
    let mut registry = REGISTRY.lock().unwrap();
    registry.push(factory);
    // Sort by priority descending
    registry.sort_by(|a, b| b.priority.cmp(&a.priority));
    info!("Registered engine: {} (priority={})", name, priority);
}

/// Select the best available engine by priority.
pub fn select_engine() -> Option<Box<dyn IspEngine>> {
    let registry = REGISTRY.lock().unwrap();
    for factory in registry.iter() {
        let engine = (factory.create_fn)();
        info!("Trying engine: {} (priority={})", factory.name, factory.priority);
        // In a real implementation, we would check if the engine can be initialized.
        // For now, we return the first one.
        if engine.priority() > 0 {
            return Some(engine);
        }
    }
    None
}

/// Select a specific engine by name (cpu, ort, mnn, or auto for best available).
pub fn select_engine_by_name(name: &str) -> Option<Box<dyn IspEngine>> {
    let registry = REGISTRY.lock().unwrap();
    let name_lower = name.to_lowercase();
    for factory in registry.iter() {
        if name_lower == "auto" || factory.name.to_lowercase().contains(&name_lower) {
            let engine = (factory.create_fn)();
            info!("Selected engine: {} (priority={})", factory.name, factory.priority);
            if engine.priority() > 0 {
                return Some(engine);
            }
        }
    }
    None
}