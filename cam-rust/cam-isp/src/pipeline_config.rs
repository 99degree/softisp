//! Pipeline Configuration — fine-grained block control.
//!
//! Provides detailed control over which blocks are enabled,
//! their parameters, and HW-aware optimization.

use std::collections::HashMap;
use std::time::Duration;

/// Block configuration — controls a single ISP block.
#[derive(Debug, Clone)]
pub struct BlockConfig {
    /// Block ID (e.g., "unpack", "demosaic", "gamma").
    pub id: String,
    /// Whether this block is enabled.
    pub enabled: bool,
    /// Block-specific parameters.
    pub params: HashMap<String, ParamValue>,
    /// Priority (higher = more important, used when HW limits force reduction).
    pub priority: u32,
    /// Estimated complexity (0.0-1.0).
    pub complexity: f64,
    /// Estimated memory usage in bytes.
    pub memory_bytes: usize,
    /// Measured processing time (if profiled).
    pub measured_time: Option<Duration>,
}

impl BlockConfig {
    /// Create a new block config.
    pub fn new(id: &str) -> Self {
        Self {
            id: id.to_string(),
            enabled: true,
            params: HashMap::new(),
            priority: Self::default_priority(id),
            complexity: Self::default_complexity(id),
            memory_bytes: Self::default_memory(id),
            measured_time: None,
        }
    }

    /// Set enabled state.
    pub fn enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Set parameter.
    pub fn param(mut self, key: &str, value: ParamValue) -> Self {
        self.params.insert(key.to_string(), value);
        self
    }

    /// Set priority.
    pub fn priority(mut self, priority: u32) -> Self {
        self.priority = priority;
        self
    }

    /// Get default priority for a block.
    fn default_priority(id: &str) -> u32 {
        match id {
            "unpack" | "unpack_cfa" => 100,  // Essential
            "demosaic" | "demosaic_ccm" => 95, // Essential
            "display" => 90,                   // Essential
            "blc" => 80,                       // Important
            "bayer_wb" => 75,                  // Important
            "ccm" => 70,                       // Important
            "fcs" => 60,                       // Nice to have
            "tone" => 65,                      // Important
            "ee" => 50,                        // Optional
            "ldci" => 45,                      // Optional
            "gamma" => 40,                     // Optional
            "sharpen" => 35,                   // Optional
            "grayscale" => 30,                 // Special case
            "warp" => 55,                      // If needed
            _ => 25,
        }
    }

    /// Get default complexity for a block.
    fn default_complexity(id: &str) -> f64 {
        match id {
            "unpack" | "unpack_cfa" => 0.05,
            "blc" | "bayer_wb" => 0.03,
            "ccm" => 0.05,
            "demosaic" => 0.15,
            "demosaic_ccm" => 0.18,
            "fcs" => 0.08,
            "tone" => 0.06,
            "ee" => 0.12,
            "ldci" => 0.10,
            "gamma" => 0.04,
            "sharpen" => 0.08,
            "display" => 0.05,
            "warp" => 0.15,
            "grayscale" => 0.02,
            _ => 0.05,
        }
    }

    /// Get default memory for a block.
    fn default_memory(id: &str) -> usize {
        match id {
            "unpack" | "unpack_cfa" => 4 * 1024 * 1024,    // 4MB
            "demosaic" | "demosaic_ccm" => 8 * 1024 * 1024, // 8MB
            "ee" => 6 * 1024 * 1024,                        // 6MB
            "ldci" => 4 * 1024 * 1024,                      // 4MB
            "warp" => 8 * 1024 * 1024,                      // 8MB
            _ => 2 * 1024 * 1024,                            // 2MB default
        }
    }
}

/// Parameter value for block configuration.
#[derive(Debug, Clone)]
pub enum ParamValue {
    /// Integer parameter.
    Int(i64),
    /// Float parameter.
    Float(f64),
    /// String parameter.
    String(String),
    /// Boolean parameter.
    Bool(bool),
}

impl From<i64> for ParamValue {
    fn from(v: i64) -> Self {
        Self::Int(v)
    }
}

impl From<f64> for ParamValue {
    fn from(v: f64) -> Self {
        Self::Float(v)
    }
}

impl From<&str> for ParamValue {
    fn from(v: &str) -> Self {
        Self::String(v.to_string())
    }
}

impl From<bool> for ParamValue {
    fn from(v: bool) -> Self {
        Self::Bool(v)
    }
}

/// Pipeline configuration — collection of block configs.
#[derive(Debug, Clone)]
pub struct PipelineConfig {
    /// Output width.
    pub width: u32,
    /// Output height.
    pub height: u32,
    /// Bayer pattern.
    pub bayer_pattern: i32,
    /// Block configurations.
    pub blocks: Vec<BlockConfig>,
    /// Target FPS (0 = unlimited).
    pub target_fps: f64,
    /// Maximum complexity (0.0-1.0).
    pub max_complexity: f64,
    /// Maximum memory in bytes.
    pub max_memory: usize,
}

impl PipelineConfig {
    /// Create a new empty config.
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            bayer_pattern: 0,
            blocks: Vec::new(),
            target_fps: 30.0,
            max_complexity: 1.0,
            max_memory: 512 * 1024 * 1024,
        }
    }

    /// Add a block to the pipeline.
    pub fn block(mut self, block: BlockConfig) -> Self {
        self.blocks.push(block);
        self
    }

    /// Enable a block by ID.
    pub fn enable_block(mut self, id: &str) -> Self {
        for block in &mut self.blocks {
            if block.id == id {
                block.enabled = true;
            }
        }
        self
    }

    /// Disable a block by ID.
    pub fn disable_block(mut self, id: &str) -> Self {
        for block in &mut self.blocks {
            if block.id == id {
                block.enabled = false;
            }
        }
        self
    }

    /// Set target FPS.
    pub fn target_fps(mut self, fps: f64) -> Self {
        self.target_fps = fps;
        self
    }

    /// Set maximum complexity.
    pub fn max_complexity(mut self, complexity: f64) -> Self {
        self.max_complexity = complexity;
        self
    }

    /// Set maximum memory.
    pub fn max_memory(mut self, memory: usize) -> Self {
        self.max_memory = memory;
        self
    }

    /// Get enabled blocks.
    pub fn enabled_blocks(&self) -> Vec<&BlockConfig> {
        self.blocks.iter().filter(|b| b.enabled).collect()
    }

    /// Calculate total complexity.
    pub fn total_complexity(&self) -> f64 {
        self.enabled_blocks()
            .iter()
            .map(|b| b.complexity)
            .sum()
    }

    /// Calculate total memory.
    pub fn total_memory(&self) -> usize {
        self.enabled_blocks()
            .iter()
            .map(|b| b.memory_bytes)
            .sum()
    }

    /// Optimize config to fit within limits.
    ///
    /// Disables lowest-priority blocks until within limits.
    /// Essential blocks (priority >= 90) are never disabled.
    pub fn optimize(&mut self) {
        // Sort blocks by priority (lowest first)
        let mut indices: Vec<usize> = (0..self.blocks.len())
            .filter(|&i| self.blocks[i].enabled)
            .collect();
        indices.sort_by_key(|&i| self.blocks[i].priority);

        // Disable blocks until within limits
        for &idx in &indices {
            if !self.blocks[idx].enabled {
                continue;
            }

            // Never disable essential blocks (priority >= 90)
            if self.blocks[idx].priority >= 90 {
                continue;
            }

            let new_complexity = self.total_complexity() - self.blocks[idx].complexity;
            let new_memory = self.total_memory() - self.blocks[idx].memory_bytes;

            if new_complexity <= self.max_complexity && new_memory <= self.max_memory {
                // Can afford to keep this block
                continue;
            }

            // Need to disable this block
            self.blocks[idx].enabled = false;
        }
    }

    /// Get optimization report.
    pub fn optimization_report(&self) -> String {
        let enabled = self.enabled_blocks();
        let total_complexity = self.total_complexity();
        let total_memory = self.total_memory();

        format!(
            "Pipeline: {}x{} | Blocks: {}/{} enabled | Complexity: {:.2}/{:.2} | Memory: {:.1}MB/{:.1}MB",
            self.width,
            self.height,
            enabled.len(),
            self.blocks.len(),
            total_complexity,
            self.max_complexity,
            total_memory as f64 / (1024.0 * 1024.0),
            self.max_memory as f64 / (1024.0 * 1024.0),
        )
    }
}

/// Profiled block performance data.
#[derive(Debug, Clone)]
pub struct BlockProfile {
    /// Block ID.
    pub id: String,
    /// Average processing time.
    pub avg_time: Duration,
    /// Minimum processing time.
    pub min_time: Duration,
    /// Maximum processing time.
    pub max_time: Duration,
    /// Number of samples.
    pub samples: usize,
    /// Measured FPS impact.
    pub fps_impact: f64,
}

impl BlockProfile {
    /// Create a new block profile.
    pub fn new(id: &str) -> Self {
        Self {
            id: id.to_string(),
            avg_time: Duration::ZERO,
            min_time: Duration::ZERO,
            max_time: Duration::ZERO,
            samples: 0,
            fps_impact: 0.0,
        }
    }

    /// Record a measurement.
    pub fn record(&mut self, time: Duration) {
        if self.samples == 0 {
            self.avg_time = time;
            self.min_time = time;
            self.max_time = time;
        } else {
            // Running average
            let total = self.avg_time * self.samples as u32 + time;
            self.avg_time = total / (self.samples + 1) as u32;
            self.min_time = self.min_time.min(time);
            self.max_time = self.max_time.max(time);
        }
        self.samples += 1;

        // Calculate FPS impact (assuming 30 FPS target)
        self.fps_impact = 1000.0 / (self.avg_time.as_secs_f64() * 1000.0);
    }
}

/// Pipeline profiler — measures performance of each block.
pub struct PipelineProfiler {
    /// Block profiles.
    profiles: HashMap<String, BlockProfile>,
    /// Number of samples per block.
    samples_per_block: usize,
}

impl PipelineProfiler {
    /// Create a new profiler.
    pub fn new(samples_per_block: usize) -> Self {
        Self {
            profiles: HashMap::new(),
            samples_per_block,
        }
    }

    /// Record a block's processing time.
    pub fn record(&mut self, block_id: &str, time: Duration) {
        let profile = self.profiles
            .entry(block_id.to_string())
            .or_insert_with(|| BlockProfile::new(block_id));
        profile.record(time);
    }

    /// Get profile for a block.
    pub fn get_profile(&self, block_id: &str) -> Option<&BlockProfile> {
        self.profiles.get(block_id)
    }

    /// Get all profiles.
    pub fn profiles(&self) -> &HashMap<String, BlockProfile> {
        &self.profiles
    }

    /// Check if we have enough samples for a block.
    pub fn has_enough_samples(&self, block_id: &str) -> bool {
        self.profiles.get(block_id)
            .map(|p| p.samples >= self.samples_per_block)
            .unwrap_or(false)
    }

    /// Get sorted block profiles by average time (fastest first).
    pub fn sorted_by_time(&self) -> Vec<&BlockProfile> {
        let mut profiles: Vec<&BlockProfile> = self.profiles.values().collect();
        profiles.sort_by_key(|p| p.avg_time);
        profiles
    }

    /// Get total pipeline time.
    pub fn total_time(&self) -> Duration {
        self.profiles.values()
            .map(|p| p.avg_time)
            .sum()
    }

    /// Get estimated FPS.
    pub fn estimated_fps(&self) -> f64 {
        let total = self.total_time();
        if total.as_secs_f64() > 0.0 {
            1.0 / total.as_secs_f64()
        } else {
            0.0
        }
    }

    /// Print profiling report.
    pub fn report(&self) -> String {
        let mut lines = Vec::new();
        lines.push("=== Block Performance Profile ===".to_string());
        lines.push(format!("Total pipeline time: {:.2}ms", self.total_time().as_secs_f64() * 1000.0));
        lines.push(format!("Estimated FPS: {:.1}", self.estimated_fps()));
        lines.push("".to_string());

        let sorted = self.sorted_by_time();
        for profile in sorted {
            lines.push(format!(
                "  {}: avg={:.2}ms, min={:.2}ms, max={:.2}ms ({} samples)",
                profile.id,
                profile.avg_time.as_secs_f64() * 1000.0,
                profile.min_time.as_secs_f64() * 1000.0,
                profile.max_time.as_secs_f64() * 1000.0,
                profile.samples,
            ));
        }

        lines.join("\n")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_block_config_creation() {
        let block = BlockConfig::new("unpack");
        assert_eq!(block.id, "unpack");
        assert!(block.enabled);
        assert!(block.priority > 0);
    }

    #[test]
    fn test_block_config_builder() {
        let block = BlockConfig::new("gamma")
            .enabled(false)
            .priority(10)
            .param("gamma", ParamValue::Float(2.2));
        
        assert!(!block.enabled);
        assert_eq!(block.priority, 10);
        assert!(matches!(block.params.get("gamma"), Some(ParamValue::Float(2.2))));
    }

    #[test]
    fn test_pipeline_config() {
        let config = PipelineConfig::new(1920, 1080)
            .block(BlockConfig::new("unpack"))
            .block(BlockConfig::new("demosaic"))
            .block(BlockConfig::new("display"))
            .target_fps(30.0)
            .max_complexity(0.5);

        assert_eq!(config.width, 1920);
        assert_eq!(config.blocks.len(), 3);
        assert_eq!(config.target_fps, 30.0);
        assert_eq!(config.max_complexity, 0.5);
    }

    #[test]
    fn test_pipeline_optimization() {
        let mut config = PipelineConfig::new(1920, 1080)
            .block(BlockConfig::new("unpack"))
            .block(BlockConfig::new("demosaic"))
            .block(BlockConfig::new("ee"))
            .block(BlockConfig::new("ldci"))
            .block(BlockConfig::new("display"))
            .max_complexity(0.3); // Tight limit

        config.optimize();

        // Should keep essential blocks
        let enabled: Vec<String> = config.enabled_blocks()
            .iter()
            .map(|b| b.id.clone())
            .collect();
        
        assert!(enabled.contains(&"unpack".to_string()));
        assert!(enabled.contains(&"demosaic".to_string()));
        assert!(enabled.contains(&"display".to_string()));
    }

    #[test]
    fn test_profiler() {
        let mut profiler = PipelineProfiler::new(3);
        
        // Record some measurements
        profiler.record("unpack", Duration::from_millis(5));
        profiler.record("unpack", Duration::from_millis(6));
        profiler.record("unpack", Duration::from_millis(4));
        
        let profile = profiler.get_profile("unpack").unwrap();
        assert_eq!(profile.samples, 3);
        assert!(profile.avg_time > Duration::ZERO);
    }

    #[test]
    fn test_param_values() {
        let val_int: ParamValue = 42i64.into();
        let val_float: ParamValue = 3.14.into();
        let val_str: ParamValue = "test".into();
        let val_bool: ParamValue = true.into();

        assert!(matches!(val_int, ParamValue::Int(42)));
        assert!(matches!(val_float, ParamValue::Float(3.14)));
        assert!(matches!(val_str, ParamValue::String(_)));
        assert!(matches!(val_bool, ParamValue::Bool(true)));
    }
}
