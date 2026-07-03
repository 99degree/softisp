//! Identity/placeholder block — pass-through for TEST profile fast path.
//! Generates minimal ONNX nodes (Identity op) to exercise the graph composer
//! and ONNX runtime without expensive ISP computation.

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// Identity block — just passes input to output.
/// Used for TEST profile to quickly verify ONNX pipeline path.
pub struct IdentityBlock {
    id: String,
    prev: Option<Box<dyn IspBlock>>,
    next: Option<Box<dyn IspBlock>>,
    input_source: String,
    output_name: String,
    channels: i64,
}

impl Default for IdentityBlock {
    fn default() -> Self {
        Self::new("identity")
    }
}

impl IdentityBlock {
    pub fn new(name: &str) -> Self {
        Self {
            id: name.to_string(),
            prev: None,
            next: None,
            input_source: String::new(),
            output_name: format!("{}/out", name),
            channels: 4, // RGBA
        }
    }

    pub fn with_channels(mut self, channels: i64) -> Self {
        self.channels = channels;
        self
    }
}

impl IspBlock for IdentityBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "IdentityBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.output_name) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    fn input_tensors(&self) -> Vec<String> {
        if self.input_source.is_empty() { vec![] } else { vec![self.input_source.clone()] }
    }

    fn output_tensors(&self) -> Vec<String> { vec![self.output_name.clone()] }

    fn graph_input_name(&self) -> Option<&str> {
        if self.is_head() { Some(&self.output_name) } else { None }
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.output_name, &[
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(self.channels),
            Proto::tensor_dim_param("H"),
            Proto::tensor_dim_param("W"),
        ], 1)) // FLOAT
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        if self.input_source.is_empty() {
            vec![]
        } else {
            vec![Proto::node("Identity", &[&self.input_source], &[&self.output_name], &[])]
        }
    }
}

/// Fast demosaic block — bilinear only for TEST profile.
pub struct FastDemosaicBlock {
    id: String,
    prev: Option<Box<dyn IspBlock>>,
    next: Option<Box<dyn IspBlock>>,
    input_source: String,
    output_name: String,
    bayer_pattern: i32,
}

impl Default for FastDemosaicBlock {
    fn default() -> Self {
        Self::new("fast_demosaic")
    }
}

impl FastDemosaicBlock {
    pub fn new(name: &str) -> Self {
        Self {
            id: name.to_string(),
            prev: None,
            next: None,
            input_source: String::new(),
            output_name: format!("{}/out", name),
            bayer_pattern: 2, // GBRG
        }
    }

    pub fn with_pattern(mut self, pattern: i32) -> Self {
        self.bayer_pattern = pattern;
        self
    }
}

impl IspBlock for FastDemosaicBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "FastDemosaicBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.output_name) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    fn input_tensors(&self) -> Vec<String> {
        if self.input_source.is_empty() { vec![] } else { vec![self.input_source.clone()] }
    }

    fn output_tensors(&self) -> Vec<String> { vec![self.output_name.clone()] }

    fn graph_input_name(&self) -> Option<&str> {
        if self.is_head() { Some(&self.output_name) } else { None }
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.output_name, &[
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(3), // RGB
            Proto::tensor_dim_param("H"),
            Proto::tensor_dim_param("W"),
        ], 1)) // FLOAT
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        if self.input_source.is_empty() {
            vec![]
        } else {
            // Use Identity for fast test path
            vec![Proto::node("Identity", &[&self.input_source], &[&self.output_name], &[])]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_id() {
        let b = IdentityBlock::new("test_id");
        assert_eq!(b.id(), "test_id");
    }

    #[test]
    fn test_identity_default_channels() {
        let b = IdentityBlock::default();
        assert_eq!(b.channels, 4);
    }

    #[test]
    fn test_identity_with_channels() {
        let b = IdentityBlock::new("id").with_channels(3);
        assert_eq!(b.channels, 3);
    }

    #[test]
    fn test_identity_empty_source_no_nodes() {
        let b = IdentityBlock::new("id");
        assert!(b.nodes().is_empty());
    }

    #[test]
    fn test_identity_with_source_one_node() {
        let mut b = IdentityBlock::new("id");
        b.set_input_source("in/frame");
        assert_eq!(b.nodes().len(), 1);
    }

    #[test]
    fn test_fast_demosaic_id() {
        let b = FastDemosaicBlock::new("fd");
        assert_eq!(b.id(), "fd");
    }

    #[test]
    fn test_fast_demosaic_with_pattern() {
        let b = FastDemosaicBlock::new("fd").with_pattern(0);
        assert_eq!(b.bayer_pattern, 0);
    }
}