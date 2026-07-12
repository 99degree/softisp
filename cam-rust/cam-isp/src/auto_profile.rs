//! Auto Profile — automatic pipeline optimization based on HW capabilities.
//!
//! Detects hardware capabilities, profiles block performance, and
//! automatically selects the optimal pipeline configuration.

use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::blocks::*;
use crate::engine::{select_engine, IspEngine, ProcessParams};
use crate::error::IspResult;
use crate::pipeline::traits::ProcessPipeline;
use crate::pipeline::{IspBlock, IspFrame};
use crate::pipeline_config::{BlockConfig, PipelineConfig, PipelineProfiler};

/// Auto-profile configuration — automatically optimized for HW.
pub struct AutoProfile {
    /// Detected HW capabilities.
    hw_caps: HwCapabilities,
    /// Optimized pipeline config.
    config: PipelineConfig,
    /// Block performance profiles.
    profiler: PipelineProfiler,
    /// Whether profiling has been done.
    profiled: bool,
    /// Target FPS.
    target_fps: f64,
}

/// Hardware capabilities detected from the system.
#[derive(Debug, Clone)]
pub struct HwCapabilities {
    /// CPU cores.
    pub cpu_cores: usize,
    /// Has SIMD (NEON/SSE).
    pub has_simd: bool,
    /// GPU available (Vulkan/OpenCL).
    pub has_gpu: bool,
    /// GPU name/description.
    pub gpu_name: String,
    /// GPU memory in bytes (0 = unknown).
    pub gpu_memory: usize,
    /// Available system memory in bytes.
    pub system_memory: usize,
    /// CPU frequency in MHz (0 = unknown).
    pub cpu_freq_mhz: u32,
    /// GPU compute capability (0.0=none, 1.0=basic, 2.0=advanced).
    pub gpu_compute: f64,
    /// Thermal throttling detected.
    pub thermal_throttling: bool,
}

impl Default for HwCapabilities {
    fn default() -> Self {
        Self {
            cpu_cores: num_cpus(),
            has_simd: has_simd_support(),
            has_gpu: false,
            gpu_name: "none".to_string(),
            gpu_memory: 0,
            system_memory: 0,
            cpu_freq_mhz: 0,
            gpu_compute: 0.0,
            thermal_throttling: false,
        }
    }
}

impl HwCapabilities {
    /// Detect hardware capabilities.
    pub fn detect() -> Self {
        let mut caps = Self::default();

        // Try to detect GPU
        if let Some(engine) = select_engine() {
            let backend = engine.backend_name();
            if backend.contains("vulkan") || backend.contains("gpu") {
                caps.has_gpu = true;
                caps.gpu_name = backend.to_string();
                caps.gpu_compute = 1.5; // Assume decent GPU
                caps.gpu_memory = 256 * 1024 * 1024; // 256MB estimate
            }
        }

        // Detect system memory
        caps.system_memory = detect_system_memory();

        caps
    }

    /// Get HW tier based on capabilities.
    pub fn tier(&self) -> HwTier {
        let score = self.cpu_cores as f64
            + if self.has_simd { 1.0 } else { 0.0 }
            + if self.has_gpu { 3.0 } else { 0.0 }
            + (self.gpu_compute * 2.0)
            + (self.system_memory as f64 / (1024.0 * 1024.0 * 1024.0)); // 1 point per GB

        if score >= 10.0 {
            HwTier::High
        } else if score >= 5.0 {
            HwTier::Medium
        } else {
            HwTier::Low
        }
    }
}

/// Hardware tier for pipeline selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HwTier {
    /// Low-end device (embedded, old phone).
    Low,
    /// Mid-range device (modern phone).
    Medium,
    /// High-end device (desktop, flagship phone).
    High,
}

impl HwTier {
    /// Get recommended target FPS.
    pub fn target_fps(&self) -> f64 {
        match self {
            Self::Low => 24.0,
            Self::Medium => 30.0,
            Self::High => 60.0,
        }
    }

    /// Get recommended max complexity.
    pub fn max_complexity(&self) -> f64 {
        match self {
            Self::Low => 0.4,
            Self::Medium => 0.7,
            Self::High => 1.0,
        }
    }

    /// Get recommended max blocks.
    pub fn max_blocks(&self) -> usize {
        match self {
            Self::Low => 8,
            Self::Medium => 12,
            Self::High => 20,
        }
    }

    /// Get recommended memory limit.
    pub fn max_memory(&self) -> usize {
        match self {
            Self::Low => 64 * 1024 * 1024,     // 64MB
            Self::Medium => 256 * 1024 * 1024, // 256MB
            Self::High => 1024 * 1024 * 1024,  // 1GB
        }
    }
}

impl AutoProfile {
    /// Create a new auto-profile.
    pub fn new(width: u32, height: u32) -> Self {
        let hw_caps = HwCapabilities::detect();
        let tier = hw_caps.tier();

        let config = PipelineConfig::new(width, height)
            .target_fps(tier.target_fps())
            .max_complexity(tier.max_complexity())
            .max_memory(tier.max_memory());

        Self {
            hw_caps,
            config,
            profiler: PipelineProfiler::new(100),
            profiled: false,
            target_fps: tier.target_fps(),
        }
    }

    /// Create with explicit target FPS.
    pub fn with_target_fps(width: u32, height: u32, target_fps: f64) -> Self {
        let mut profile = Self::new(width, height);
        profile.target_fps = target_fps;
        profile.config.target_fps = target_fps;
        profile
    }

    /// Get hardware capabilities.
    pub fn hw_caps(&self) -> &HwCapabilities {
        &self.hw_caps
    }

    /// Get hardware tier.
    pub fn tier(&self) -> HwTier {
        self.hw_caps.tier()
    }

    /// Get the optimized config.
    pub fn config(&self) -> &PipelineConfig {
        &self.config
    }

    /// Build the optimal pipeline.
    pub fn build(&mut self) -> IspResult<AutoPipeline> {
        // 1. Select optimal profile based on HW
        self.select_optimal_blocks();

        // 2. Optimize to fit limits
        self.config.optimize();

        // 3. Build blocks
        let blocks = self.build_blocks()?;

        // 4. Build engine
        let engine = crate::pipeline::build::build_engine(blocks)?;

        // 5. Create pipeline
        Ok(AutoPipeline {
            engine,
            config: self.config.clone(),
            profiler: PipelineProfiler::new(100),
            profiled: self.profiled,
        })
    }

    /// Select optimal blocks based on HW capabilities.
    fn select_optimal_blocks(&mut self) {
        let tier = self.hw_caps.tier();

        // Start with all blocks
        self.config.blocks = vec![
            BlockConfig::new("unpack"),
            BlockConfig::new("blc"),
            BlockConfig::new("bayer_wb"),
            BlockConfig::new("demosaic"),
            BlockConfig::new("ccm"),
            BlockConfig::new("fcs"),
            BlockConfig::new("tone"),
            BlockConfig::new("ee"),
            BlockConfig::new("ldci"),
            BlockConfig::new("gamma"),
            BlockConfig::new("sharpen"),
            BlockConfig::new("display"),
        ];

        // Disable blocks based on tier
        match tier {
            HwTier::Low => {
                // Minimal pipeline
                self.disable_block("ee");
                self.disable_block("ldci");
                self.disable_block("gamma");
                self.disable_block("sharpen");
                self.disable_block("fcs");
            }
            HwTier::Medium => {
                // Balanced pipeline
                self.disable_block("ldci");
                self.disable_block("sharpen");
            }
            HwTier::High => {
                // Full pipeline
            }
        }

        // Check if GPU is available for advanced blocks
        if !self.hw_caps.has_gpu {
            self.disable_block("warp");
        }

        // Check thermal throttling
        if self.hw_caps.thermal_throttling {
            self.disable_block("ee");
            self.disable_block("ldci");
        }
    }

    /// Disable a block by ID.
    fn disable_block(&mut self, id: &str) {
        for block in &mut self.config.blocks {
            if block.id == id {
                block.enabled = false;
            }
        }
    }

    /// Build blocks from config.
    fn build_blocks(&mut self) -> IspResult<Vec<Box<dyn IspBlock>>> {
        // Auto-select blocks if not already configured
        if self.config.blocks.is_empty() {
            self.select_optimal_blocks();
        }

        let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();

        for block_config in self.config.enabled_blocks() {
            let block = self.create_block(&block_config.id)?;
            blocks.push(block);
        }

        Ok(blocks)
    }

    /// Create a block by ID.
    fn create_block(&self, id: &str) -> IspResult<Box<dyn IspBlock>> {
        match id {
            "unpack" => Ok(Box::new(UnpackBlock::new())),
            "blc" => Ok(Box::new(BlcBlock::new())),
            "bayer_wb" => Ok(Box::new(BayerWbBlock::new())),
            "demosaic" => Ok(Box::new(DemosaicBlock::new(0))), // demosaic only, not fused CCM
            "ccm" => Ok(Box::new(CcmBlock::new())),            // separate CCM after demosaic
            "fcs" => Ok(Box::new(FcsBlock::new())),
            "tone" => Ok(Box::new(ToneBlock::new())),
            "ee" => Ok(Box::new(EeBlock::new())),
            "ldci" => Ok(Box::new(LdciBlock::new())),
            "gamma" => Ok(Box::new(GammaBlock::new(2.2))),
            "sharpen" => Ok(Box::new(SharpenBlock::new(0.5))),
            "display" => Ok(Box::new(DisplayBlock::new(self.config.width))),
            "grayscale" => Ok(Box::new(GrayscaleBlock::new())),
            "warp" => Ok(Box::new(WarpGridBlock::new(
                self.config.width,
                self.config.height,
            ))),
            _ => Err(crate::error::IspError::Pipeline(format!(
                "Unknown block: {}",
                id
            ))),
        }
    }

    /// Profile the pipeline to measure actual performance.
    pub fn profile(&mut self, params: &ProcessParams, iterations: usize) -> IspResult<()> {
        let pipeline = self.build()?;

        // Warmup
        for _ in 0..10 {
            let _ = pipeline.process(params);
        }

        // Profile
        let mut block_times: HashMap<String, Duration> = HashMap::new();

        for _ in 0..iterations {
            let start = Instant::now();
            let _ = pipeline.process(params)?;
            let total_time = start.elapsed();

            // Record total time (individual block timing requires instrumentation)
            block_times
                .entry("total".to_string())
                .and_modify(|t| *t += total_time)
                .or_insert(total_time);
        }

        // Update profiler
        for (id, total) in &block_times {
            let avg = *total / iterations as u32;
            self.profiler.record(id, avg);
        }

        self.profiled = true;
        Ok(())
    }

    /// Get profiling report.
    pub fn report(&self) -> String {
        let mut lines = Vec::new();

        lines.push("=== Auto Profile Report ===".to_string());
        lines.push(format!("HW Tier: {:?}", self.hw_caps.tier()));
        lines.push(format!(
            "CPU: {} cores, SIMD: {}",
            self.hw_caps.cpu_cores, self.hw_caps.has_simd
        ));
        lines.push(format!(
            "GPU: {} (compute: {:.1})",
            self.hw_caps.gpu_name, self.hw_caps.gpu_compute
        ));
        lines.push("".to_string());

        lines.push(self.config.optimization_report());
        lines.push("".to_string());

        if self.profiled {
            lines.push(self.profiler.report());
        } else {
            lines.push("Not profiled yet. Call profile() to measure performance.".to_string());
        }

        lines.join("\n")
    }
}

/// Auto-optimized pipeline.
pub struct AutoPipeline {
    /// The underlying engine.
    engine: Box<dyn IspEngine>,
    /// The optimized config.
    config: PipelineConfig,
    /// Performance profiles.
    profiler: PipelineProfiler,
    /// Whether profiled.
    profiled: bool,
}

impl AutoPipeline {
    /// Get the optimized config.
    pub fn config(&self) -> &PipelineConfig {
        &self.config
    }

    /// Get profiling report.
    pub fn report(&self) -> String {
        let mut lines = Vec::new();

        lines.push("=== Auto Pipeline ===".to_string());
        lines.push(self.config.optimization_report());

        if self.profiled {
            lines.push("".to_string());
            lines.push(self.profiler.report());
        }

        lines.join("\n")
    }

    /// Get FPS estimate.
    pub fn estimated_fps(&self) -> f64 {
        self.profiler.estimated_fps()
    }
}

impl ProcessPipeline for AutoPipeline {
    fn process(&self, params: &ProcessParams) -> IspResult<IspFrame> {
        self.engine.process(params)
    }

    fn engine(&self) -> &dyn IspEngine {
        self.engine.as_ref()
    }

    fn is_loaded(&self) -> bool {
        self.engine.is_loaded()
    }
}

/// Detect system memory.
fn detect_system_memory() -> usize {
    // Try to read from /proc/meminfo on Linux
    #[cfg(target_os = "linux")]
    {
        if let Ok(content) = std::fs::read_to_string("/proc/meminfo") {
            for line in content.lines() {
                if line.starts_with("MemTotal:") {
                    if let Some(kb) = line.split_whitespace().nth(1) {
                        if let Ok(kb) = kb.parse::<usize>() {
                            return kb * 1024; // Convert KB to bytes
                        }
                    }
                }
            }
        }
    }

    // Fallback: assume 4GB
    4 * 1024 * 1024 * 1024
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
        true
    }
    #[cfg(target_arch = "x86_64")]
    {
        true
    }
    #[cfg(not(any(target_arch = "aarch64", target_arch = "x86_64")))]
    {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hw_capabilities_detection() {
        let caps = HwCapabilities::detect();
        assert!(caps.cpu_cores > 0);
        assert!(caps.tier() != HwTier::Low || caps.cpu_cores <= 4);
    }

    #[test]
    fn test_hw_tier_scoring() {
        let low = HwCapabilities {
            cpu_cores: 2,
            has_simd: false,
            has_gpu: false,
            ..Default::default()
        };
        assert_eq!(low.tier(), HwTier::Low);

        let high = HwCapabilities {
            cpu_cores: 8,
            has_simd: true,
            has_gpu: true,
            gpu_compute: 2.0,
            system_memory: 8 * 1024 * 1024 * 1024,
            ..Default::default()
        };
        assert_eq!(high.tier(), HwTier::High);
    }

    #[test]
    fn test_auto_profile_creation() {
        let profile = AutoProfile::new(1920, 1080);
        assert!(profile.hw_caps().cpu_cores > 0);
        assert!(profile.config().width == 1920);
    }

    #[test]
    fn test_auto_profile_build() {
        crate::init();
        let mut profile = AutoProfile::new(640, 480);
        // Use CPU engine for test (handles all ONNX ops without MNN conversion)
        let blocks = profile.build_blocks().unwrap();
        let engine = crate::pipeline::build::build_engine_with(
            blocks,
            Box::new(crate::cpu::CpuEngine::new()),
        );
        assert!(engine.is_ok(), "Build failed: {:?}", engine.err());
    }

    #[test]
    fn test_auto_profile_report() {
        let profile = AutoProfile::new(1920, 1080);
        let report = profile.report();
        assert!(report.contains("Auto Profile Report"));
        assert!(report.contains("HW Tier"));
    }
}
