//! PipelineDiff — compare two pipeline configurations.
//!
//! Useful for A/B testing pipeline variants, detecting regressions,
//! and tracking pipeline changes across versions.

use crate::serializer::PipelineConfig;

/// Differences between two pipeline configurations.
#[derive(Debug, Clone)]
pub struct PipelineDiff {
    pub width_changed: bool,
    pub height_changed: bool,
    pub blocks_added: Vec<String>,
    pub blocks_removed: Vec<String>,
    pub blocks_reordered: bool,
}

impl PipelineDiff {
    /// Compute diff between two pipeline configs.
    pub fn compute(a: &PipelineConfig, b: &PipelineConfig) -> Self {
        let width_changed = a.width != b.width;
        let height_changed = a.height != b.height;

        let a_ids: Vec<&str> = a.block_ids.iter().map(|s| s.as_str()).collect();
        let b_ids: Vec<&str> = b.block_ids.iter().map(|s| s.as_str()).collect();

        let blocks_added: Vec<String> = b_ids
            .iter()
            .filter(|id| !a_ids.contains(id))
            .map(|s| s.to_string())
            .collect();

        let blocks_removed: Vec<String> = a_ids
            .iter()
            .filter(|id| !b_ids.contains(id))
            .map(|s| s.to_string())
            .collect();

        let blocks_reordered =
            !blocks_added.is_empty() || !blocks_removed.is_empty() || a.block_ids != b.block_ids;

        Self {
            width_changed,
            height_changed,
            blocks_added,
            blocks_removed,
            blocks_reordered,
        }
    }

    /// Check if the diff is empty (configs are identical).
    pub fn is_empty(&self) -> bool {
        !self.width_changed
            && !self.height_changed
            && self.blocks_added.is_empty()
            && self.blocks_removed.is_empty()
    }

    /// Human-readable summary.
    pub fn summary(&self) -> String {
        if self.is_empty() {
            return "No changes".into();
        }
        let mut parts = Vec::new();
        if self.width_changed || self.height_changed {
            parts.push("Resolution changed".into());
        }
        if !self.blocks_added.is_empty() {
            parts.push(format!(
                "+{} blocks: {}",
                self.blocks_added.len(),
                self.blocks_added.join(", ")
            ));
        }
        if !self.blocks_removed.is_empty() {
            parts.push(format!(
                "-{} blocks: {}",
                self.blocks_removed.len(),
                self.blocks_removed.join(", ")
            ));
        }
        parts.join("; ")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config(ids: &[&str], w: u32, h: u32) -> PipelineConfig {
        let mut cfg = PipelineConfig::new(w, h);
        cfg.block_ids = ids.iter().map(|s| s.to_string()).collect();
        cfg
    }

    #[test]
    fn test_diff_identical() {
        let a = make_config(&["unpack", "demosaic", "display"], 640, 480);
        let b = make_config(&["unpack", "demosaic", "display"], 640, 480);
        let diff = PipelineDiff::compute(&a, &b);
        assert!(diff.is_empty());
    }

    #[test]
    fn test_diff_resolution_changed() {
        let a = make_config(&["unpack", "display"], 640, 480);
        let b = make_config(&["unpack", "display"], 1920, 1080);
        let diff = PipelineDiff::compute(&a, &b);
        assert!(diff.width_changed);
        assert!(diff.height_changed);
    }

    #[test]
    fn test_diff_blocks_added() {
        let a = make_config(&["unpack", "display"], 640, 480);
        let b = make_config(&["unpack", "demosaic", "display"], 640, 480);
        let diff = PipelineDiff::compute(&a, &b);
        assert_eq!(diff.blocks_added, vec!["demosaic"]);
        assert!(diff.blocks_removed.is_empty());
    }

    #[test]
    fn test_diff_blocks_removed() {
        let a = make_config(&["unpack", "demosaic", "display"], 640, 480);
        let b = make_config(&["unpack", "display"], 640, 480);
        let diff = PipelineDiff::compute(&a, &b);
        assert!(diff.blocks_added.is_empty());
        assert_eq!(diff.blocks_removed, vec!["demosaic"]);
    }

    #[test]
    fn test_diff_summary() {
        let a = make_config(&["unpack", "display"], 640, 480);
        let b = make_config(&["unpack", "demosaic", "display"], 640, 480);
        let diff = PipelineDiff::compute(&a, &b);
        let s = diff.summary();
        assert!(s.contains("+1"));
        assert!(s.contains("demosaic"));
    }

    #[test]
    fn test_diff_empty_summary() {
        let a = make_config(&["unpack", "display"], 640, 480);
        let b = make_config(&["unpack", "display"], 640, 480);
        let diff = PipelineDiff::compute(&a, &b);
        assert_eq!(diff.summary(), "No changes");
    }
}
