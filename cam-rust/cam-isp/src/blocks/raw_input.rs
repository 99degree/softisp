//! RawInputBlock — pipeline head, declares INT32 input tensor.
//!
//! This block is the entry point of the ISP pipeline. It declares the input
//! tensor that will receive raw sensor data from the camera.
//!
//! # Input Format
//!
//! - Raw 16-bit Bayer data (packed or unpacked)
//! - Stored as INT32 in the ONNX model (for compatibility)
//! - Supports RGGB, GRBG, GBRG, BGGR patterns
//!
//! # Usage
//!
//! ```rust,ignore
//! let raw_input = RawInputBlock::new();
//! let pipeline = raw_input
//!     .chain(CfaBlock::new())
//!     .chain(DemosaicBlock::new(0))
//!     .chain(DisplayBlock::new(width));
//! ```
use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct RawInputBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
    pub elem_type: i32,
}

impl Default for RawInputBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl RawInputBlock {
    pub fn new() -> Self {
        Self {
            id: "raw_input".to_string(),
            prev: None,
            next: None,
            frame_tensor: "RawInputBlock/frame".to_string(),
            input_source: String::new(),
            concrete_h: None,
            concrete_w: None,
            elem_type: 6, // INT32 default — no UINT16 in pipeline
        }
    }
    
    /// Set concrete height/width for fixed-shape models (avoids MNN resize crash).
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }
    /// Set only the concrete width (height stays symbolic).
    pub fn with_concrete_width(mut self, w: i64) -> Self {
        self.concrete_w = Some(w);
        self
    }
    
    /// Set element type: 1=FLOAT, 6=INT32 (default).
    pub fn with_elem_type(mut self, t: i32) -> Self {
        self.elem_type = t;
        self
    }
}

impl IspBlock for RawInputBlock {
    #[inline]
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "RawInputBlock".to_string() }
    #[inline]
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    #[inline]
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    #[inline]
    fn graph_input_name(&self) -> Option<&str> { Some(&self.frame_tensor) }
    #[inline]
    fn input_elem_type(&self) -> i32 { self.elem_type }

    fn input_tensors(&self) -> Vec<String> { vec![] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        let dims: Vec<Vec<u8>> = match (self.concrete_h, self.concrete_w) {
            (Some(h), Some(w)) => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h), Proto::tensor_dim_value(w),
            ],
            (None, Some(w)) => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"), Proto::tensor_dim_value(w),
            ],
            _ => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W"),
            ],
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, self.elem_type))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_raw_input_id() {
        assert_eq!(RawInputBlock::new().id(), "raw_input");
    }

    #[test]
    fn test_raw_input_graph_input() {
        let b = RawInputBlock::new();
        assert_eq!(b.graph_input_name(), Some("RawInputBlock/frame"));
    }

    #[test]
    fn test_raw_input_concrete_dims() {
        let b = RawInputBlock::new().with_concrete_dims(1080, 1920);
        assert_eq!(b.concrete_h, Some(1080));
        assert_eq!(b.concrete_w, Some(1920));
    }

    #[test]
    fn test_raw_input_elem_type() {
        let b = RawInputBlock::new();
        assert_eq!(b.elem_type, 6); // INT32
        let b = RawInputBlock::new().with_elem_type(1);
        assert_eq!(b.elem_type, 1); // FLOAT
    }

    #[test]
    fn test_raw_input_emit_onnx() {
        let b = RawInputBlock::new();
        let nodes = b.nodes();
        // RawInput has no nodes, only the graph input
        assert!(nodes.is_empty());
    }
}
