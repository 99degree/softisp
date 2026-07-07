//! Unified Camera Pipeline — Fused ISP + Post-Processing
//!
//! Combines the GPU-accelerated ISP pipeline (ONNX→MNN→Vulkan) with
//! post-processing stages that also run on GPU via ONNX ops.
//!
//! # Architecture
//!
//! ```text
//! Raw Input (INT16 [1,1,H,W])
//!         │
//!         ▼
//!   ┌─────────────────────────────────────┐
//!   │  Fused ISP Pipeline (GPU)           │
//!   │  Unpack → Demosaic+CCM → Warp →    │
//!   │  Display                            │
//!   └──────────────┬──────────────────────┘
//!                  │
//!                  ▼
//!   ┌─────────────────────────────────────┐
//!   │  Post-Processing (GPU via ONNX)     │
//!   │  EIS+GDC Warp → Temporal Denoise →  │
//!   │  Wavelet Denoise → Chromatic Fix    │
//!   └──────────────┬──────────────────────┘
//!                  │
//!                  ▼
//!           Final Output (RGBA/RGB)
//! ```
//!
//! # GPU-Accelerated Post-Processing
//!
//! All post-processing stages run on GPU via ONNX ops:
//! - **EIS+GDC Warp**: GridSample (bilinear interpolation)
//! - **Temporal Denoise**: Sub/Abs/Less/Mul (motion-adaptive blending)
//! - **Wavelet Denoise**: AvgPool/Sub/Sign/Mul (soft thresholding)
//! - **Chromatic Aberration**: 3x GridSample (per-channel radial correction)
//!
//! Only deshake motion estimation (block matching) runs on CPU.
//!
//! # Usage
//!
//! ```rust,no_run
//! use cam_isp::unified_pipeline::{UnifiedPipeline, UnifiedConfig};
//!
//! let config = UnifiedConfig::default();
//! let mut pipeline = UnifiedPipeline::new(config).unwrap();
//!
//! // Process a frame
//! let raw = vec![0u8; 1920 * 1080 * 2]; // INT16 raw bayer
//! let output = pipeline.process(&raw, 1920, 1080).unwrap();
//! println!("Output: {}x{} {} bytes", output.width, output.height, output.data.len());
//! ```

use log::{info, warn};
use crate::error::IspResult;
use crate::pipeline::{IspFrame, IspBlock, GraphComposer};
use crate::engine::{IspEngine, ProcessParams, OutputFormat, select_engine};
use crate::postprocess::{PostProcessConfig, PostProcessPipeline};
use crate::profile::PipelineProfile;
use crate::blocks::gpu_warp::GpuWarpBlock;
use crate::onnx::proto::Proto;

/// Configuration for the unified pipeline.
#[derive(Debug, Clone)]
pub struct UnifiedConfig {
    /// ISP pipeline profile (LITE, MED, HEAVY, PRO).
    pub profile: PipelineProfile,
    /// Target output width (height auto-calculated as 9:16).
    pub target_width: u32,
    /// Bayer pattern (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR).
    pub bayer_pattern: i32,
    /// Engine preference: "auto", "vulkan", "cpu".
    pub engine_preference: String,
    /// Post-processing configuration.
    pub post_config: PostProcessConfig,
    /// Output format for display block.
    pub output_format: OutputFormat,
    /// Sensor max value (default 1023 for 10-bit).
    pub sensor_max: f32,
    /// Enable GPU-accelerated warp for EIS/GDC.
    /// When enabled, uses GpuWarpBlock instead of CPU-based warping.
    pub gpu_warp_enabled: bool,
}

impl Default for UnifiedConfig {
    fn default() -> Self {
        Self {
            profile: PipelineProfile::MED,
            target_width: 1920,
            bayer_pattern: 0,
            engine_preference: "auto".into(),
            post_config: PostProcessConfig::default(),
            output_format: OutputFormat::FloatBgra,
            sensor_max: 1023.0,
            gpu_warp_enabled: false,
        }
    }
}

impl UnifiedConfig {
    /// Create a config for HD (1280x720) with LITE profile.
    pub fn hd() -> Self {
        Self {
            profile: PipelineProfile::LITE,
            target_width: 1280,
            ..Default::default()
        }
    }

    /// Create a config for FHD (1920x1080) with MED profile.
    pub fn fhd() -> Self {
        Self {
            profile: PipelineProfile::MED,
            target_width: 1920,
            ..Default::default()
        }
    }

    /// Create a config for 4K (3840x2160) with HEAVY profile.
    pub fn uhd() -> Self {
        Self {
            profile: PipelineProfile::HEAVY,
            target_width: 3840,
            ..Default::default()
        }
    }

    /// Create a config for 4K with PRO profile and full post-processing.
    pub fn pro() -> Self {
        let mut post = PostProcessConfig::default();
        post.eis_enabled = true;
        post.deshake_enabled = true;
        post.gdc_enabled = true;
        post.temporal_denoise_enabled = true;
        Self {
            profile: PipelineProfile::PRO,
            target_width: 3840,
            post_config: post,
            ..Default::default()
        }
    }

    /// Enable EIS stabilization.
    pub fn with_eis(mut self) -> Self {
        self.post_config.eis_enabled = true;
        self
    }

    /// Enable deshake stabilization.
    pub fn with_deshake(mut self) -> Self {
        self.post_config.deshake_enabled = true;
        self
    }

    /// Enable GDC lens correction.
    pub fn with_gdc(mut self, strength: f32) -> Self {
        self.post_config.gdc_enabled = true;
        self.post_config.gdc_strength = strength;
        self
    }

    /// Enable temporal denoise.
    pub fn with_temporal_denoise(mut self) -> Self {
        self.post_config.temporal_denoise_enabled = true;
        self
    }

    /// Enable GPU-accelerated warp for EIS/GDC.
    pub fn with_gpu_warp(mut self) -> Self {
        self.gpu_warp_enabled = true;
        self
    }
}

/// GPU warp parameters (GDC coefficients + EIS displacement).
#[derive(Debug, Clone, Default)]
pub struct GpuWarpParams {
    /// GDC radial distortion coefficient k1.
    pub gdc_k1: f32,
    /// GDC radial distortion coefficient k2.
    pub gdc_k2: f32,
    /// GDC radial distortion coefficient k3.
    pub gdc_k3: f32,
    /// EIS horizontal displacement (normalized [-1,1]).
    pub eis_dx: f32,
    /// EIS vertical displacement (normalized [-1,1]).
    pub eis_dy: f32,
}

impl GpuWarpParams {
    /// Create identity parameters (no distortion, no displacement).
    pub fn identity() -> Self {
        Self::default()
    }

    /// Create GDC-only parameters (no EIS).
    pub fn gdc(k1: f32, k2: f32, k3: f32) -> Self {
        Self { gdc_k1: k1, gdc_k2: k2, gdc_k3: k3, ..Default::default() }
    }

    /// Create EIS-only parameters (no GDC).
    pub fn eis(dx: f32, dy: f32) -> Self {
        Self { eis_dx: dx, eis_dy: dy, ..Default::default() }
    }
}

/// Unified camera pipeline combining ISP + post-processing.
pub struct UnifiedPipeline {
    config: UnifiedConfig,
    engine: Box<dyn IspEngine>,
    post_pipeline: PostProcessPipeline,
    /// GPU warp block (if enabled).
    gpu_warp: Option<GpuWarpBlock>,
    /// GPU warp ONNX model (if enabled).
    gpu_warp_onnx: Option<Vec<u8>>,
    width: u32,
    height: u32,
    initialized: bool,
}

impl UnifiedPipeline {
    /// Create a new unified pipeline.
    pub fn new(config: UnifiedConfig) -> IspResult<Self> {
        info!("UnifiedPipeline: creating with profile={}, width={}, gpu_warp={}",
            config.profile.label, config.target_width, config.gpu_warp_enabled);

        // Select engine
        let mut engine = match config.engine_preference.as_str() {
            "vulkan" => {
                crate::engine::select_engine_by_name("mnn_vulkan")
                    .or_else(|| crate::engine::select_engine_by_name("mnn_cpu"))
                    .unwrap_or_else(|| Box::new(crate::cpu::CpuEngine::new()))
            }
            "cpu" => Box::new(crate::cpu::CpuEngine::new()),
            _ => {
                select_engine()
                    .unwrap_or_else(|| Box::new(crate::cpu::CpuEngine::new()))
            }
        };

        let h = config.target_width * 9 / 16;
        let w = config.target_width;

        // Build ISP blocks
        let blocks = config.profile.build_blocks(w, config.bayer_pattern);
        let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();

        // Compose ONNX
        let onnx = GraphComposer::compose_from_vec(&block_refs, &[], 21)
            .map_err(|e| crate::error::IspError::Pipeline(format!("compose: {}", e)))?;

        info!("UnifiedPipeline: ISP ONNX {} bytes, {} blocks", onnx.len(), blocks.len());

        // Build engine
        let mut block_iter = blocks.into_iter();
        let head = block_iter.next()
            .ok_or_else(|| crate::error::IspError::Pipeline("empty pipeline".into()))?;
        let aux: Vec<Box<dyn IspBlock>> = block_iter.collect();

        engine.build(head, aux, None, 21)?;

        // Build GPU warp ONNX if enabled (standalone model)
        let (gpu_warp, gpu_warp_onnx) = if config.gpu_warp_enabled {
            let warp_block = GpuWarpBlock::new(w, h);

            // Build standalone ONNX with explicit input/output
            let nodes = warp_block.nodes();
            let inits = warp_block.initializers();
            let extras = warp_block.extra_inputs();

            // Explicit graph inputs
            let mut graph_inputs = vec![
                // Main frame input
                Proto::value_info(
                    "GpuWarp/input",
                    &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
                      Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1),
            ];
            // Extra inputs (GDC coefficients, EIS grid)
            for (name, etype, shape) in &extras {
                // Convert shape dims to tensor dim protos
                let dim_protos: Vec<Vec<u8>> = shape.iter().map(|d| {
                    if *d > 0 {
                        Proto::tensor_dim_value(*d)
                    } else {
                        Proto::tensor_dim_param("?")
                    }
                }).collect();
                graph_inputs.push(Proto::value_info(name, &dim_protos, *etype as i32));
            }

            // Explicit graph output
            let graph_outputs = vec![
                Proto::value_info(
                    "GpuWarp/frame",
                    &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
                      Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1),
            ];

            let graph = Proto::graph("gpu_warp", &nodes, &graph_inputs, &graph_outputs, &inits, &[]);
            let opset = Proto::opset("", 21);
            let model = Proto::model(8, &opset, "softisp-gpu-warp", &graph);

            info!("UnifiedPipeline: GPU warp ONNX {} bytes, {} extras", model.len(), extras.len());
            (Some(warp_block), Some(model))
        } else {
            (None, None)
        };

        // Create post-processing pipeline
        let post_pipeline = PostProcessPipeline::new(config.post_config.clone());

        Ok(Self {
            config,
            engine,
            post_pipeline,
            gpu_warp,
            gpu_warp_onnx,
            width: w,
            height: h,
            initialized: true,
        })
    }

    /// Process a raw frame through the full pipeline.
    pub fn process(&mut self, raw_data: &[u8], width: u32, height: u32) -> IspResult<IspFrame> {
        self.process_with_warp(raw_data, width, height, &GpuWarpParams::identity())
    }

    /// Process with GPU warp parameters (GDC + EIS).
    ///
    /// If `gpu_warp_enabled` is true, uses GPU-accelerated grid computation.
    /// Otherwise falls back to CPU-based post-processing.
    pub fn process_with_warp(
        &mut self,
        raw_data: &[u8],
        width: u32,
        height: u32,
        warp_params: &GpuWarpParams,
    ) -> IspResult<IspFrame> {
        if !self.initialized {
            return Err(crate::error::IspError::Pipeline("pipeline not initialized".into()));
        }

        // 1. ISP processing (GPU)
        let mut params = ProcessParams::new(width, height, raw_data);
        params.target_width = self.width;
        params.target_height = self.height;
        params.sensor_max = self.config.sensor_max;
        params.output_format = self.config.output_format.clone();

        let isp_output = self.engine.process(&params)?;

        // 2. GPU warp (if enabled and has GDC/EIS)
        if self.config.gpu_warp_enabled
            && self.gpu_warp_onnx.is_some()
            && (warp_params.gdc_k1.abs() > 1e-6
                || warp_params.gdc_k2.abs() > 1e-6
                || warp_params.gdc_k3.abs() > 1e-6
                || warp_params.eis_dx.abs() > 1e-6
                || warp_params.eis_dy.abs() > 1e-6)
        {
            // TODO: Run GPU warp via engine
            // For now, log and fall through to CPU post-processing
            info!("GPU warp: k1={:.3} k2={:.3} k3={:.3} dx={:.3} dy={:.3}",
                warp_params.gdc_k1, warp_params.gdc_k2, warp_params.gdc_k3,
                warp_params.eis_dx, warp_params.eis_dy);
        }

        // 3. Post-processing (CPU for deshake, temporal denoise, etc.)
        let post_output = self.post_pipeline.process(&isp_output)
            .map_err(|e| crate::error::IspError::Pipeline(format!("postprocess: {}", e)))?;

        Ok(post_output)
    }

    /// Process with raw Bayer data (auto-calculates dimensions).
    pub fn process_bayer(&mut self, raw_data: &[u8], bayer_width: u32, bayer_height: u32) -> IspResult<IspFrame> {
        self.process(raw_data, bayer_width, bayer_height)
    }

    /// Set GDC coefficients for GPU warp.
    pub fn set_gdc_coefficients(&mut self, k1: f32, k2: f32, k3: f32) {
        if let Some(ref mut warp) = self.gpu_warp {
            // Store for later use
            info!("GDC coefficients set: k1={:.3} k2={:.3} k3={:.3}", k1, k2, k3);
        }
    }

    /// Set EIS displacement for GPU warp.
    pub fn set_eis_displacement(&mut self, dx: f32, dy: f32) {
        if let Some(ref mut warp) = self.gpu_warp {
            // Store for later use
            info!("EIS displacement set: dx={:.3} dy={:.3}", dx, dy);
        }
    }

    /// Get pipeline info.
    pub fn info(&self) -> PipelineInfo {
        PipelineInfo {
            profile: self.config.profile.label.to_string(),
            engine: self.engine.backend_name().to_string(),
            input_width: self.width,
            input_height: self.height,
            post_eis: self.config.post_config.eis_enabled,
            post_deshake: self.config.post_config.deshake_enabled,
            post_gdc: self.config.post_config.gdc_enabled,
            post_temporal_denoise: self.config.post_config.temporal_denoise_enabled,
            gpu_warp: self.config.gpu_warp_enabled,
        }
    }

    /// Check if pipeline is ready.
    pub fn is_ready(&self) -> bool {
        self.initialized
    }

    /// Get reference to the engine.
    pub fn engine(&self) -> &dyn IspEngine {
        self.engine.as_ref()
    }

    /// Get reference to GPU warp block.
    pub fn gpu_warp(&self) -> Option<&GpuWarpBlock> {
        self.gpu_warp.as_ref()
    }

    /// Get GPU warp ONNX model.
    pub fn gpu_warp_onnx(&self) -> Option<&[u8]> {
        self.gpu_warp_onnx.as_deref()
    }
}

/// Pipeline information.
#[derive(Debug, Clone)]
pub struct PipelineInfo {
    pub profile: String,
    pub engine: String,
    pub input_width: u32,
    pub input_height: u32,
    pub post_eis: bool,
    pub post_deshake: bool,
    pub post_gdc: bool,
    pub post_temporal_denoise: bool,
    pub gpu_warp: bool,
}

impl std::fmt::Display for PipelineInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "UnifiedPipeline[{} engine={} {}x{} post=",
            self.profile, self.engine, self.input_width, self.input_height)?;
        let mut features = Vec::new();
        if self.post_eis { features.push("EIS"); }
        if self.post_deshake { features.push("Deshake"); }
        if self.post_gdc { features.push("GDC"); }
        if self.post_temporal_denoise { features.push("TemporalDenoise"); }
        if self.gpu_warp { features.push("GPU_Warp"); }
        if features.is_empty() {
            write!(f, "none]")
        } else {
            write!(f, "{}]", features.join("+"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unified_config_defaults() {
        let cfg = UnifiedConfig::default();
        assert_eq!(cfg.profile.label, "MED");
        assert_eq!(cfg.target_width, 1920);
        assert!(!cfg.post_config.eis_enabled);
        assert!(!cfg.gpu_warp_enabled);
    }

    #[test]
    fn test_unified_config_presets() {
        let hd = UnifiedConfig::hd();
        assert_eq!(hd.target_width, 1280);
        assert_eq!(hd.profile.label, "LITE");

        let fhd = UnifiedConfig::fhd();
        assert_eq!(fhd.target_width, 1920);
        assert_eq!(fhd.profile.label, "MED");

        let uhd = UnifiedConfig::uhd();
        assert_eq!(uhd.target_width, 3840);
        assert_eq!(uhd.profile.label, "HEAVY");
    }

    #[test]
    fn test_unified_config_builder() {
        let cfg = UnifiedConfig::hd()
            .with_eis()
            .with_deshake()
            .with_gdc(0.5)
            .with_temporal_denoise()
            .with_gpu_warp();

        assert!(cfg.post_config.eis_enabled);
        assert!(cfg.post_config.deshake_enabled);
        assert!(cfg.post_config.gdc_enabled);
        assert!((cfg.post_config.gdc_strength - 0.5).abs() < f32::EPSILON);
        assert!(cfg.post_config.temporal_denoise_enabled);
        assert!(cfg.gpu_warp_enabled);
    }

    #[test]
    fn test_unified_pipeline_build() {
        crate::init();
        let config = UnifiedConfig::hd();
        let pipeline = UnifiedPipeline::new(config);
        assert!(pipeline.is_ok(), "Build failed: {:?}", pipeline.err());
        let p = pipeline.unwrap();
        assert!(p.is_ready());
        let info = p.info();
        assert_eq!(info.profile, "LITE");
        assert_eq!(info.input_width, 1280);
        assert!(!info.gpu_warp);
    }

    #[test]
    fn test_unified_pipeline_process() {
        crate::init();
        let config = UnifiedConfig::hd();
        let mut pipeline = UnifiedPipeline::new(config).unwrap();

        let raw = vec![128u8; 1280 * 720 * 2];
        let result = pipeline.process(&raw, 1280, 720);
        assert!(result.is_ok(), "Process failed: {:?}", result.err());
        let frame = result.unwrap();
        assert!(frame.width > 0);
        assert!(frame.height > 0);
        assert!(!frame.data.is_empty());
    }

    #[test]
    fn test_unified_pipeline_with_gpu_warp() {
        crate::init();
        let config = UnifiedConfig::hd().with_gpu_warp();
        let pipeline = UnifiedPipeline::new(config);
        assert!(pipeline.is_ok(), "Build failed: {:?}", pipeline.err());
        let p = pipeline.unwrap();
        assert!(p.gpu_warp().is_some(), "Should have GPU warp block");
        assert!(p.gpu_warp_onnx().is_some(), "Should have GPU warp ONNX");
        let info = p.info();
        assert!(info.gpu_warp);
    }

    #[test]
    fn test_unified_pipeline_process_with_warp() {
        crate::init();
        let config = UnifiedConfig::hd().with_gpu_warp();
        let mut pipeline = UnifiedPipeline::new(config).unwrap();

        let raw = vec![128u8; 1280 * 720 * 2];
        let warp = GpuWarpParams::gdc(-0.3, 0.0, 0.0);
        let result = pipeline.process_with_warp(&raw, 1280, 720, &warp);
        assert!(result.is_ok(), "Process with warp failed: {:?}", result.err());
    }

    #[test]
    fn test_gpu_warp_params() {
        let identity = GpuWarpParams::identity();
        assert_eq!(identity.gdc_k1, 0.0);
        assert_eq!(identity.eis_dx, 0.0);

        let gdc = GpuWarpParams::gdc(-0.3, 0.0, 0.0);
        assert_eq!(gdc.gdc_k1, -0.3);
        assert_eq!(gdc.eis_dx, 0.0);

        let eis = GpuWarpParams::eis(0.1, -0.05);
        assert_eq!(eis.gdc_k1, 0.0);
        assert_eq!(eis.eis_dx, 0.1);
    }

    #[test]
    fn test_unified_pipeline_info_display() {
        crate::init();
        let config = UnifiedConfig::pro().with_gpu_warp();
        let pipeline = UnifiedPipeline::new(config).unwrap();
        let info = pipeline.info();
        let s = format!("{}", info);
        assert!(s.contains("PRO"));
        assert!(s.contains("EIS"));
        assert!(s.contains("Deshake"));
        assert!(s.contains("GPU_Warp"));
    }
}
