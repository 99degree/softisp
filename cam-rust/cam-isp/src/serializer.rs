//! Pipeline serialization — save/load pipeline topology as text.
//!
//! Format is intentionally simple:
//! ```text
//! # softisp pipeline v1
//! W=1920 H=1080
//! unpack
//! demosaic_ccm
//! display
//! ```
//!
//! No external dependencies needed.

use crate::pipeline::IspBlock;

/// Serializable pipeline configuration.
#[derive(Debug, Clone)]
pub struct PipelineConfig {
    pub version: u32,
    pub width: u32,
    pub height: u32,
    pub block_ids: Vec<String>,
}

impl PipelineConfig {
    pub fn new(w: u32, h: u32) -> Self {
        Self { version: 1, width: w, height: h, block_ids: Vec::new() }
    }

    /// Create a config from a pipeline (block list).
    pub fn from_blocks(blocks: &[&dyn IspBlock], w: u32, h: u32) -> Self {
        let mut cfg = Self::new(w, h);
        for blk in blocks {
            cfg.block_ids.push(blk.id().to_string());
        }
        cfg
    }

    /// Serialize to text.
    pub fn to_text(&self) -> String {
        let mut lines = vec![
            format!("# softisp pipeline v{}", self.version),
            format!("W={} H={}", self.width, self.height),
        ];
        for id in &self.block_ids {
            lines.push(id.clone());
        }
        lines.join("\n")
    }

    /// Deserialize from text.
    pub fn from_text(s: &str) -> Result<Self, String> {
        let mut version = 1u32;
        let mut width = 0u32;
        let mut height = 0u32;
        let mut block_ids = Vec::new();

        for line in s.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                if line.starts_with("# softisp pipeline v") {
                    version = line.split_whitespace().last()
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(1);
                }
                continue;
            }
            if line.starts_with("W=") || line.starts_with("H=") {
                for part in line.split_whitespace() {
                    if let Some(v) = part.strip_prefix("W=") {
                        width = v.parse().map_err(|e| format!("bad W: {}", e))?;
                    }
                    if let Some(v) = part.strip_prefix("H=") {
                        height = v.parse().map_err(|e| format!("bad H: {}", e))?;
                    }
                }
                continue;
            }
            block_ids.push(line.to_string());
        }

        Ok(Self { version, width, height, block_ids })
    }

    /// Number of blocks.
    pub fn block_count(&self) -> usize {
        self.block_ids.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pipeline_config_roundtrip() {
        let mut cfg = PipelineConfig::new(1920, 1080);
        cfg.block_ids.extend(["unpack", "demosaic", "display"].iter().map(|s| s.to_string()));

        let text = cfg.to_text();
        let loaded = PipelineConfig::from_text(&text).unwrap();
        assert_eq!(loaded.width, 1920);
        assert_eq!(loaded.height, 1080);
        assert_eq!(loaded.block_count(), 3);
        assert_eq!(loaded.block_ids, vec!["unpack", "demosaic", "display"]);
    }

    #[test]
    fn test_pipeline_config_empty() {
        let cfg = PipelineConfig::new(640, 480);
        let text = cfg.to_text();
        let loaded = PipelineConfig::from_text(&text).unwrap();
        assert_eq!(loaded.block_count(), 0);
        assert_eq!(loaded.width, 640);
    }

    #[test]
    fn test_pipeline_config_from_blocks() {
        use crate::blocks::{UnpackBlock, DemosaicCcmBlock, DisplayBlock};
        use crate::pipeline::GraphComposer;

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(640)),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let cfg = PipelineConfig::from_blocks(&refs, 640, 480);
        assert_eq!(cfg.block_count(), 3);
        assert!(cfg.block_ids.contains(&"unpack".to_string()));
    }

    #[test]
    fn test_pipeline_config_invalid_text() {
        let result = PipelineConfig::from_text("W=abc H=1080\nunpack");
        assert!(result.is_err());
    }

    #[test]
    fn test_pipeline_config_version_preserved() {
        let mut cfg = PipelineConfig::new(3840, 2160);
        cfg.block_ids.push("unpack".into());
        cfg.block_ids.push("display".into());
        let text = cfg.to_text();
        assert!(text.contains("v1"));
        let loaded = PipelineConfig::from_text(&text).unwrap();
        assert_eq!(loaded.version, 1);
    }
}
