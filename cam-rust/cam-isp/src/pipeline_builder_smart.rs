//! Smart Pipeline Builder — builds pipelines within HW limits.
//!
//! Analyzes hardware capabilities and builds optimal pipeline
//! configuration that stays within performance constraints.

use crate::engine::{IspEngine, select_engine};
use crate::error::IspResult;
use crate::pipeline::{IspBlock, IspFrame};
use crate::pipeline::build::{build_engine, build_engine_with};
use crate::pipeline::traits::ProcessPipeline;
use crate::profile::PipelineProfile;
use crate::frame_rate::{FrameRateController, FrameRateStats};

/// Hardware capability limits.
#[derive(Debug, Clone)]
pub struct HwLimits {
    /// Maximum pipeline complexity (0.0-1.0).
    pub max_complexity: f64,
    /// Maximum number of blocks in pipeline.
    pub max_blocks: usize,
    /// Maximum ONNX model size in bytes.
    pub max_onnx_size: usize,
    /// Maximum memory usage in bytes.
    pub max_memory: usize,
    /// Target frame rate in FPS.
    pub target_fps: f64,
    /// Available GPU memory in bytes (0 = no GPU).
    pub gpu_memory: usize,
    /// GPU compute capability (0.0 = none, 1.0 = basic, 2.0 = advanced).
    pub gpu_compute: f64,
    /// CPU cores available.
    pub cpu_cores: usize,
    /// Has NEON/SSE SIMD support.
    pub has_simd: bool,
}

impl Default for HwLimits {
    fn default() -> Self {
        Self {
            max_complexity: 1.0,
            max_blocks: 20,
            max_onnx_size: 50 * 1024 * 1024, // 50MB
            max_memory: 512 * 1024 * 1024,     // 512MB
            target_fps: 30.0,
            gpu_memory: 0,
            gpu_compute: 0.0,
            cpu_cores: num_cpus(),
            has_simd: has_simd_support(),
        }
    }
}

impl HwLimits {
    /// Detect hardware limits automatically.
    pub fn detect() -> Self {
        let mut limits = Self::default();

        // Try to detect GPU
        if let Some(engine) = select_engine() {
            let backend = engine.backend_name();
            if backend.contains("vulkan") || backend.contains("gpu") {
                limits.gpu_compute = 1.5; // Assume decent GPU
                limits.gpu_memory = 256 * 1024 * 1024; // 256MB estimate
            }
        }

        limits
    }

    /// Create limits for mobile device.
    pub fn mobile() -> Self {
        Self {
            max_complexity: 0.6,
            max_blocks: 12,
            max_onnx_size: 20 * 1024 * 1024, // 20MB
            max_memory: 256 * 1024 * 1024,     // 256MB
            target_fps: 30.0,
            gpu_memory: 128 * 1024 * 1024,
            gpu_compute: 1.0,
            cpu_cores: 4,
            has_simd: true,
        }
    }

    /// Create limits for desktop/server.
    pub fn desktop() -> Self {
        Self {
            max_complexity: 1.0,
            max_blocks: 30,
            max_onnx_size: 100 * 1024 * 1024, // 100MB
            max_memory: 2 * 1024 * 1024 * 1024, // 2GB
            target_fps: 60.0,
            gpu_memory: 1024 * 1024 * 1024,
            gpu_compute: 2.0,
            cpu_cores: 8,
            has_simd: true,
        }
    }

    /// Create limits for embedded/IoT.
    pub fn embedded() -> Self {
        Self {
            max_complexity: 0.3,
            max_blocks: 6,
            max_onnx_size: 5 * 1024 * 1024, // 5MB
            max_memory: 64 * 1024 * 1024,     // 64MB
            target_fps: 15.0,
            gpu_memory: 0,
            gpu_compute: 0.0,
            cpu_cores: 2,
            has_simd: false,
        }
    }
}

/// Smart pipeline configuration.
#[derive(Debug, Clone)]
pub struct SmartPipelineConfig {
    /// Target output width.
    pub width: u32,
    /// Target output height.
    pub height: u32,
    /// Bayer pattern (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR).
    pub bayer_pattern: i32,
    /// Hardware limits.
    pub hw_limits: HwLimits,
    /// Profile to use (HEAVY will be downgraded if needed).
    pub profile: PipelineProfile,
    /// Enable GPU acceleration.
    pub gpu_enabled: bool,
    /// Enable SIMD acceleration.
    pub simd_enabled: bool,
    /// Enable temporal denoise (requires extra memory).
    pub temporal_denoise: bool,
    /// Enable lens correction (requires extra compute).
    pub lens_correction: bool,
}

impl Default for SmartPipelineConfig {
    fn default() -> Self {
        Self {
            width: 1920,
            height: 1080,
            bayer_pattern: 0,
            hw_limits: HwLimits::default(),
            profile: PipelineProfile::MED,
            gpu_enabled: true,
            simd_enabled: true,
            temporal_denoise: false,
            lens_correction: false,
        }
    }
}

impl SmartPipelineConfig {
    /// Create from hardware detection.
    pub fn detect() -> Self {
        Self {
            hw_limits: HwLimits::detect(),
            ..Default::default()
        }
    }

    /// Create for mobile device.
    pub fn mobile(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            bayer_pattern: 0,
            hw_limits: HwLimits::mobile(),
            profile: PipelineProfile::LITE,
            gpu_enabled: true,
            simd_enabled: true,
            temporal_denoise: false,
            lens_correction: false,
        }
    }

    /// Create for desktop.
    pub fn desktop(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            bayer_pattern: 0,
            hw_limits: HwLimits::desktop(),
            profile: PipelineProfile::HEAVY,
            gpu_enabled: true,
            simd_enabled: true,
            temporal_denoise: true,
            lens_correction: true,
        }
    }
}

/// Smart pipeline builder that optimizes for HW limits.
pub struct SmartPipelineBuilder {
    config: SmartPipelineConfig,
    /// Blocks to include.
    blocks: Vec<Box<dyn IspBlock>>,
    /// Frame rate controller.
    fps_controller: FrameRateController,
    /// Estimated complexity (0.0-1.0).
    estimated_complexity: f64,
    /// Estimated memory usage in bytes.
    estimated_memory: usize,
    /// Estimated ONNX size in bytes.
    estimated_onnx_size: usize,
}

impl SmartPipelineBuilder {
    /// Create a new smart builder.
    pub fn new(config: SmartPipelineConfig) -> Self {
        let fps_controller = FrameRateController::new(config.hw_limits.target_fps);
        Self {
            config,
            blocks: Vec::new(),
            fps_controller,
            estimated_complexity: 0.0,
            estimated_memory: 0,
            estimated_onnx_size: 0,
        }
    }

    /// Build the optimal pipeline.
    pub fn build(mut self) -> IspResult<SmartPipeline> {
        // 1. Select optimal profile based on HW limits
        let profile = self.select_optimal_profile();

        // 2. Build blocks with complexity estimation
        let blocks = self.build_blocks_for_profile(&profile);

        // 3. Check if we need to simplify further
        let final_blocks = self.optimize_for_limits(blocks);

        // 4. Build the engine
        let engine = build_engine(final_blocks)?;

        // 5. Create the pipeline with frame rate control
        Ok(SmartPipeline {
            engine,
            fps_controller: self.fps_controller,
            config: self.config,
            estimated_complexity: self.estimated_complexity,
            estimated_memory: self.estimated_memory,
            estimated_onnx_size: self.estimated_onnx_size,
        })
    }

    /// Select optimal profile based on HW limits.
    fn select_optimal_profile(&self) -> PipelineProfile {
        let limits = &self.config.hw_limits;

        // Check if requested profile fits within limits
        let requested_complexity = self.profile_complexity(&self.config.profile);

        if requested_complexity <= limits.max_complexity {
            return self.config.profile;
        }

        // Downgrade profile until it fits
        let profiles = [
            (PipelineProfile::HEAVY, 1.0),
            (PipelineProfile::PRO, 0.8),
            (PipelineProfile::MED, 0.6),
            (PipelineProfile::LITE, 0.3),
        ];

        for (profile, complexity) in profiles {
            if complexity <= limits.max_complexity {
                return profile;
            }
        }

        // Fallback to LITE
        PipelineProfile::LITE
    }

    /// Get complexity estimate for a profile.
    fn profile_complexity(&self, profile: &PipelineProfile) -> f64 {
        match profile.label {
            "LITE" => 0.3,
            "MED" => 0.6,
            "HEAVY" => 1.0,
            "PRO" => 0.8,
            "UNIFIED" => 1.0,
            _ => 0.5,
        }
    }

    /// Build blocks for a profile.
    fn build_blocks_for_profile(&mut self, profile: &PipelineProfile) -> Vec<Box<dyn IspBlock>> {
        let blocks = profile.build_blocks(self.config.width, self.config.bayer_pattern);

        // Estimate complexity based on block count and types
        self.estimated_complexity = blocks.len() as f64 / 20.0; // Normalize to 20 blocks max
        self.estimated_memory = blocks.len() * 1024 * 1024; // Rough estimate: 1MB per block

        blocks
    }

    /// Optimize blocks to fit within HW limits.
    fn optimize_for_limits(&mut self, blocks: Vec<Box<dyn IspBlock>>) -> Vec<Box<dyn IspBlock>> {
        let limits = &self.config.hw_limits;
        let mut result = Vec::new();

        for block in blocks {
            // Check block count limit
            if result.len() >= limits.max_blocks {
                break;
            }

            // Check complexity limit
            if self.estimated_complexity > limits.max_complexity {
                // Try to skip non-essential blocks
                if self.is_essential_block(block.id()) {
                    result.push(block);
                } else {
                    continue;
                }
            } else {
                result.push(block);
            }
        }

        result
    }

    /// Check if a block is essential (cannot be skipped).
    fn is_essential_block(&self, block_id: &str) -> bool {
        // Essential blocks: unpack, demosaic, display
        matches!(block_id, 
            "unpack" | "unpack_cfa" | "demosaic" | "demosaic_ccm" | "display"
        )
    }

    /// Get estimated complexity.
    pub fn estimated_complexity(&self) -> f64 {
        self.estimated_complexity
    }

    /// Get estimated memory usage.
    pub fn estimated_memory(&self) -> usize {
        self.estimated_memory
    }

    /// Get the frame rate controller.
    pub fn fps_controller(&self) -> &FrameRateController {
        &self.fps_controller
    }
}

/// Smart pipeline with frame rate control.
pub struct SmartPipeline {
    /// The underlying engine.
    engine: Box<dyn IspEngine>,
    /// Frame rate controller.
    fps_controller: FrameRateController,
    /// Configuration used.
    config: SmartPipelineConfig,
    /// Estimated complexity.
    estimated_complexity: f64,
    /// Estimated memory usage.
    estimated_memory: usize,
    /// Estimated ONNX size.
    estimated_onnx_size: usize,
}

impl SmartPipeline {
    /// Process a frame with frame rate control.
    ///
    /// Returns `None` if the frame should be skipped (too fast).
    pub fn process_controlled(
        &mut self,
        params: &crate::engine::ProcessParams,
    ) -> IspResult<Option<IspFrame>> {
        // Check if we should process this frame
        if !self.fps_controller.should_process() {
            return Ok(None); // Skip frame
        }

        let start = Instant::now();

        // Process the frame
        let frame = self.engine.process(params)?;

        let elapsed = start.elapsed();
        self.fps_controller.record_frame(elapsed);

        Ok(Some(frame))
    }

    /// Process a frame without rate control.
    pub fn process(&self, params: &crate::engine::ProcessParams) -> IspResult<IspFrame> {
        self.engine.process(params)
    }

    /// Get frame rate statistics.
    pub fn fps_stats(&self) -> FrameRateStats {
        self.fps_controller.stats()
    }

    /// Get recommended complexity based on current load.
    pub fn recommended_complexity(&self) -> f64 {
        self.fps_controller.recommended_complexity()
    }

    /// Check if pipeline is under heavy load.
    pub fn is_heavy_load(&self) -> bool {
        self.fps_controller.is_heavy_load()
    }

    /// Get backend name.
    pub fn backend_name(&self) -> &str {
        self.engine.backend_name()
    }

    /// Check if pipeline is loaded.
    pub fn is_loaded(&self) -> bool {
        self.engine.is_loaded()
    }

    /// Get estimated complexity.
    pub fn estimated_complexity(&self) -> f64 {
        self.estimated_complexity
    }

    /// Get estimated memory usage.
    pub fn estimated_memory(&self) -> usize {
        self.estimated_memory
    }
}

impl ProcessPipeline for SmartPipeline {
    fn process(&self, params: &crate::engine::ProcessParams) -> IspResult<IspFrame> {
        self.engine.process(params)
    }

    fn engine(&self) -> &dyn IspEngine {
        self.engine.as_ref()
    }

    fn is_loaded(&self) -> bool {
        self.engine.is_loaded()
    }
}

/// Helper to detect number of CPU cores.
fn num_cpus() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4)
}

/// Helper to detect SIMD support.
fn has_simd_support() -> bool {
    #[cfg(target_arch = "aarch64")]
    {
        // Check for NEON support (always available on AArch64)
        true
    }
    #[cfg(target_arch = "x86_64")]
    {
        // Check for SSE2 (always available on x86_64)
        true
    }
    #[cfg(not(any(target_arch = "aarch64", target_arch = "x86_64")))]
    {
        false
    }
}

use std::time::Instant;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hw_limits_defaults() {
        let limits = HwLimits::default();
        assert!(limits.max_complexity > 0.0);
        assert!(limits.max_blocks > 0);
        assert!(limits.cpu_cores > 0);
    }

    #[test]
    fn test_hw_limits_mobile() {
        let limits = HwLimits::mobile();
        assert!(limits.max_complexity <= 0.6);
        assert!(limits.max_blocks <= 12);
        assert!(limits.target_fps >= 20.0);
    }

    #[test]
    fn test_smart_builder_creation() {
        let config = SmartPipelineConfig::mobile(640, 480);
        let builder = SmartPipelineBuilder::new(config);
        assert!(builder.estimated_complexity() >= 0.0);
    }

    #[test]
    fn test_profile_selection() {
        let config = SmartPipelineConfig {
            hw_limits: HwLimits::embedded(),
            profile: PipelineProfile::HEAVY,
            ..Default::default()
        };
        let builder = SmartPipelineBuilder::new(config);
        // Should downgrade from HEAVY to something simpler
        let profile = builder.select_optimal_profile();
        assert_ne!(profile.label, "HEAVY");
    }

    #[test]
    fn test_essential_blocks() {
        let builder = SmartPipelineBuilder::new(SmartPipelineConfig::default());
        assert!(builder.is_essential_block("unpack"));
        assert!(builder.is_essential_block("demosaic"));
        assert!(builder.is_essential_block("display"));
        assert!(!builder.is_essential_block("gamma"));
        assert!(!builder.is_essential_block("sharpen"));
    }
}
