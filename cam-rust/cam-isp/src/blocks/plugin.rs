//! PluginBlock — dynamic ONNX plugin block.
//!
//! Loads an arbitrary `.onnx` model at runtime and wraps it as an ISP block.
//! This enables third-party extensions without recompiling the pipeline.
//!
//! Usage:
//!   ```no_run
//!   use cam_isp::blocks::PluginBlock;
//!
//!   // Load a custom denoise model
//!   let block = PluginBlock::from_file("custom_denoise.onnx", "CustomDenoise");
//!   ```
//!
//! The plugin model must:
//!   - Have exactly one float32 input named "input"
//!   - Have exactly one float32 output named "output"
//!   - Be in NCHW format: `[1, C, H, W]`

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;
use std::path::{Path, PathBuf};

/// PluginBlock — loads and runs arbitrary ONNX models as ISP blocks.
///
/// Enables dynamic extensions without recompilation. The loaded model
/// is converted to MNN and runs on the selected backend (Vulkan/CPU).
pub struct PluginBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// Path to the `.onnx` model file.
    pub model_path: PathBuf,
    /// Number of input channels (default: 3).
    pub input_channels: i64,
    /// Number of output channels (default: 3).
    pub output_channels: i64,
}

impl Default for PluginBlock {
    fn default() -> Self {
        Self::new("unnamed.onnx", "plugin")
    }
}

impl PluginBlock {
    pub fn new(model_path: &str, block_id: &str) -> Self {
        Self {
            id: block_id.into(),
            prev_block: None,
            next_block: None,
            frame_tensor: format!("PluginBlock/{}/frame", block_id),
            input_source: String::new(),
            model_path: PathBuf::from(model_path),
            input_channels: 3,
            output_channels: 3,
        }
    }

    pub fn from_file(model_path: impl AsRef<Path>, block_id: &str) -> Self {
        let path = model_path.as_ref().to_path_buf();
        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or(block_id);
        Self::new(&path.to_string_lossy(), &format!("plugin_{}", name))
    }

    pub fn with_input_channels(mut self, c: i64) -> Self {
        self.input_channels = c;
        self
    }

    pub fn with_output_channels(mut self, c: i64) -> Self {
        self.output_channels = c;
        self
    }
}

impl IspBlock for PluginBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        format!("Plugin/{}", self.id)
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }
    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.into();
    }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev_block.as_ref()
    }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) {
        self.prev_block = Some(block);
    }
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next_block.as_ref()
    }
    fn set_next(&mut self, block: Box<dyn IspBlock>) {
        self.next_block = Some(block);
    }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.input_source,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(self.input_channels),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(self.output_channels),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        // PluginBlock emits a single Identity node.
        // The actual model is loaded separately by the engine at runtime.
        // The ONNX graph here is a placeholder for topology validation.
        vec![Proto::node(
            "Identity",
            &[&self.input_source],
            &[&self.frame_tensor],
            &[],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plugin_new() {
        let b = PluginBlock::new("test.onnx", "test_plugin");
        assert_eq!(b.id(), "test_plugin");
        assert_eq!(b.model_path, PathBuf::from("test.onnx"));
    }

    #[test]
    fn test_plugin_from_file() {
        let b = PluginBlock::from_file("/path/to/denoise.onnx", "custom");
        assert_eq!(b.id(), "plugin_denoise");
        assert_eq!(b.model_path, PathBuf::from("/path/to/denoise.onnx"));
    }

    #[test]
    fn test_plugin_channels() {
        let b = PluginBlock::new("test.onnx", "test")
            .with_input_channels(1)
            .with_output_channels(4);
        assert_eq!(b.input_channels, 1);
        assert_eq!(b.output_channels, 4);
    }

    #[test]
    fn test_plugin_emit_onnx() {
        let b = PluginBlock::new("test.onnx", "test");
        let nodes = b.nodes();
        assert_eq!(nodes.len(), 1);
    }

    #[test]
    fn test_plugin_has_input_output() {
        let b = PluginBlock::new("test.onnx", "test");
        assert_eq!(b.input_tensors().len(), 1);
        assert_eq!(b.output_tensors().len(), 1);
    }

    #[test]
    fn test_plugin_model_path() {
        let b = PluginBlock::new("test.onnx", "test");
        assert_eq!(b.model_path, PathBuf::from("test.onnx"));
    }

    #[test]
    fn test_plugin_tensor_ns() {
        let b = PluginBlock::new("test.onnx", "my_plugin");
        assert_eq!(b.tensor_ns(), "Plugin/my_plugin");
    }
}
