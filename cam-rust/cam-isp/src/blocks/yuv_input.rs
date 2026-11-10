//! YUV input blocks — pipeline heads for YUV sensor input tensors.
//!
//! These blocks handle YUV sensor input formats (YUV_420_888, NV12, NV21, etc.)
//! and convert them to the standard RGB/Bayer pipeline format.
//!
//! # YUV Formats Supported
//!
//! - **YUV_420_888** — Three planes: Y (H×W), U (H/2×W/2), V (H/2×W/2)
//! - **NV12** — Y plane (H×W) + interleaved UV plane (H/2×W/2)
//! - **NV21** — Y plane (H×W) + interleaved VU plane (H/2×W/2)
//!
//! # Pipeline Integration
//!
//! These blocks serve as pipeline heads, similar to RawInputBlock, but for
//! YUV formats. They declare the input tensor shape and pass data through
//! to the YUV-to-RGB conversion stage.
//!
//! ```rust,ignore
//! let yuv_input = Yuv420InputBlock::new();
//! let pipeline = yuv_input
//!     .chain(YuvToRgbBlock::new())
//!     .chain(DisplayBlock::new(width));
//! ```

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Generates a YUV input block variant.
macro_rules! impl_yuv_input_block {
    ($struct:ident, $id:literal, $ns:literal, $frame:literal, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug)]
        pub struct $struct {
            pub id: String,
            pub prev: Option<Box<dyn IspBlock>>,
            pub next: Option<Box<dyn IspBlock>>,
            pub frame_tensor: String,
            pub input_source: String,
            pub concrete_h: Option<i64>,
            pub concrete_w: Option<i64>,
        }

        impl Default for $struct {
            fn default() -> Self {
                Self::new()
            }
        }

        impl $struct {
            pub fn new() -> Self {
                Self {
                    id: $id.to_string(),
                    prev: None,
                    next: None,
                    frame_tensor: $frame.to_string(),
                    input_source: String::new(),
                    concrete_h: None,
                    concrete_w: None,
                }
            }

            /// Set concrete height/width for fixed-shape models.
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

            /// Set concrete height.
            pub fn with_concrete_height(mut self, h: i64) -> Self {
                self.concrete_h = Some(h);
                self
            }
        }

        impl IspBlock for $struct {
            #[inline]
            fn id(&self) -> &str {
                &self.id
            }
            fn tensor_ns(&self) -> String {
                $ns.to_string()
            }
            #[inline]
            fn frame_tensor(&self) -> Option<&str> {
                Some(&self.frame_tensor)
            }
            #[inline]
            fn input_source(&self) -> Option<&str> {
                Some(&self.input_source)
            }
            fn set_input_source(&mut self, name: &str) {
                self.input_source = name.to_string();
            }
            fn prev(&self) -> Option<&Box<dyn IspBlock>> {
                self.prev.as_ref()
            }
            fn set_prev(&mut self, block: Box<dyn IspBlock>) {
                self.prev = Some(block);
            }
            fn next(&self) -> Option<&Box<dyn IspBlock>> {
                self.next.as_ref()
            }
            fn set_next(&mut self, block: Box<dyn IspBlock>) {
                self.next = Some(block);
            }

            #[inline]
            fn graph_input_name(&self) -> Option<&str> {
                Some(&self.frame_tensor)
            }
            #[inline]
            fn input_elem_type(&self) -> i32 {
                1 // FLOAT32
            }

            fn input_tensors(&self) -> Vec<String> {
                vec![]
            }
            fn output_tensors(&self) -> Vec<String> {
                vec![self.frame_tensor.clone()]
            }

            fn input_value_info(&self) -> Option<Vec<u8>> {
                let dims: Vec<Vec<u8>> = match (self.concrete_h, self.concrete_w) {
                    (Some(h), Some(w)) => vec![
                        Proto::tensor_dim_value(1),
                        Proto::tensor_dim_value(3), // YUV has 3 planes
                        Proto::tensor_dim_value(h),
                        Proto::tensor_dim_value(w),
                    ],
                    (None, Some(w)) => vec![
                        Proto::tensor_dim_value(1),
                        Proto::tensor_dim_value(3),
                        Proto::tensor_dim_param("H"),
                        Proto::tensor_dim_value(w),
                    ],
                    _ => vec![
                        Proto::tensor_dim_value(1),
                        Proto::tensor_dim_value(3),
                        Proto::tensor_dim_param("H"),
                        Proto::tensor_dim_param("W"),
                    ],
                };
                Some(Proto::value_info(&self.frame_tensor, &dims, 1)) // FLOAT32
            }
            fn output_value_info(&self) -> Option<Vec<u8>> {
                self.input_value_info()
            }
        }
    };
}

impl_yuv_input_block!(
    Yuv420InputBlock,
    "yuv420_input",
    "Yuv420InputBlock",
    "Yuv420InputBlock/frame",
    "YUV_420_888 input `[1,3,H,W]` FLOAT32 — three planes Y/U/V."
);

impl_yuv_input_block!(
    Nv12InputBlock,
    "nv12_input",
    "Nv12InputBlock",
    "Nv12InputBlock/frame",
    "NV12 input `[1,2,H,W]` FLOAT32 — Y + interleaved UV."
);

impl_yuv_input_block!(
    Nv21InputBlock,
    "nv21_input",
    "Nv21InputBlock",
    "Nv21InputBlock/frame",
    "NV21 input `[1,2,H,W]` FLOAT32 — Y + interleaved VU."
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_yuv420_input_id() {
        assert_eq!(Yuv420InputBlock::new().id(), "yuv420_input");
    }

    #[test]
    fn test_nv12_input_id() {
        assert_eq!(Nv12InputBlock::new().id(), "nv12_input");
    }

    #[test]
    fn test_nv21_input_id() {
        assert_eq!(Nv21InputBlock::new().id(), "nv21_input");
    }

    #[test]
    fn test_yuv420_input_graph_input() {
        let b = Yuv420InputBlock::new();
        assert_eq!(b.graph_input_name(), Some("Yuv420InputBlock/frame"));
    }

    #[test]
    fn test_yuv420_input_concrete_dims() {
        let b = Yuv420InputBlock::new().with_concrete_dims(1080, 1920);
        assert_eq!(b.concrete_h, Some(1080));
        assert_eq!(b.concrete_w, Some(1920));
    }

    #[test]
    fn test_yuv420_input_elem_type() {
        assert_eq!(Yuv420InputBlock::new().input_elem_type(), 1); // FLOAT32
        assert_eq!(Nv12InputBlock::new().input_elem_type(), 1);
        assert_eq!(Nv21InputBlock::new().input_elem_type(), 1);
    }

    #[test]
    fn test_yuv420_input_emit_onnx() {
        let b = Yuv420InputBlock::new();
        let nodes = b.nodes();
        assert!(nodes.is_empty());
    }

    #[test]
    fn test_yuv420_input_dims() {
        let vi = Yuv420InputBlock::new()
            .with_concrete_dims(48, 64)
            .input_value_info()
            .unwrap();
        assert!(!vi.is_empty());
    }
}
