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
}

/// Unified camera pipeline combining ISP + post-processing.
pub struct UnifiedPipeline {
    config: UnifiedConfig,
    engine: Box<dyn IspEngine>,
    post_pipeline: PostProcessPipeline,
    width: u32,
    height: u32,
    initialized: bool,
}

impl UnifiedPipeline {
    /// Create a new unified pipeline.
    pub fn new(config: UnifiedConfig) -> IspResult<Self> {
        info!("UnifiedPipeline: creating with profile={}, width={}",
            config.profile.label, config.target_width);

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

        info!("UnifiedPipeline: ONNX {} bytes, {} blocks", onnx.len(), blocks.len());

        // Build engine
        let mut block_iter = blocks.into_iter();
        let head = block_iter.next()
            .ok_or_else(|| crate::error::IspError::Pipeline("empty pipeline".into()))?;
        let aux: Vec<Box<dyn IspBlock>> = block_iter.collect();

        engine.build(head, aux, None, 21)?;

        // Create post-processing pipeline
        let post_pipeline = PostProcessPipeline::new(config.post_config.clone());

        Ok(Self {
            config,
            engine,
            post_pipeline,
            width: w,
            height: h,
            initialized: true,
        })
    }

    /// Process a raw frame through the full pipeline.
    pub fn process(&mut self, raw_data: &[u8], width: u32, height: u32) -> IspResult<IspFrame> {
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

        // 2. Post-processing (CPU)
        let post_output = self.post_pipeline.process(&isp_output)
            .map_err(|e| crate::error::IspError::Pipeline(format!("postprocess: {}", e)))?;

        Ok(post_output)
    }

    /// Process with raw Bayer data (auto-calculates dimensions).
    pub fn process_bayer(&mut self, raw_data: &[u8], bayer_width: u32, bayer_height: u32) -> IspResult<IspFrame> {
        self.process(raw_data, bayer_width, bayer_height)
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
            .with_temporal_denoise();

        assert!(cfg.post_config.eis_enabled);
        assert!(cfg.post_config.deshake_enabled);
        assert!(cfg.post_config.gdc_enabled);
        assert!((cfg.post_config.gdc_strength - 0.5).abs() < f32::EPSILON);
        assert!(cfg.post_config.temporal_denoise_enabled);
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
    fn test_unified_pipeline_info_display() {
        crate::init();
        let config = UnifiedConfig::pro();
        let pipeline = UnifiedPipeline::new(config).unwrap();
        let info = pipeline.info();
        let s = format!("{}", info);
        assert!(s.contains("PRO"));
        assert!(s.contains("EIS"));
        assert!(s.contains("Deshake"));
    }
}
