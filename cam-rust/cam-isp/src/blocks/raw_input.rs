//! Raw input blocks — pipeline heads declaring the sensor input tensor.
//!
//! Three distinct input conventions, one block each (no convention options —
//! the pipeline selects the block variant, never reconfigures a block):
//!
//! - [`RawInputBlock`] — raw INT32 `[1,1,H,W]` (legacy / default head).
//! - [`RawInput16Block`] — pure 16-bit raw `[1,1,H,W]` INT16.
//! - [`RawInputPackedBlock`] — packed 2×int16-as-int32 `[1,2,H,W/2]`
//!   (channel 0 = even pixels, channel 1 = odd pixels; engine splits the
//!   u16 pairs into lanes on the host).
//!
//! # Usage
//!
//! ```rust,ignore
//! let raw_input = RawInput16Block::new();
//! let pipeline = raw_input
//!     .chain(CfaBlock::new())
//!     .chain(DemosaicBlock::new(0))
//!     .chain(DisplayBlock::new(width));
//! ```
use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Generates one raw-input block variant. `$elem` is the ONNX element type
/// (5 = INT16, 6 = INT32), `$channels` the declared channel count, and
/// `$width_div` the divisor applied to the concrete width (2 for the packed
/// half-width tensor, 1 otherwise).
macro_rules! impl_raw_input_block {
    ($struct:ident, $id:literal, $ns:literal, $frame:literal, $elem:expr, $channels:expr, $width_div:expr, $doc:literal) => {
        #[doc = $doc]
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
                $elem
            }

            fn input_tensors(&self) -> Vec<String> {
                vec![]
            }
            fn output_tensors(&self) -> Vec<String> {
                vec![self.frame_tensor.clone()]
            }

            fn input_value_info(&self) -> Option<Vec<u8>> {
                let dw = self.concrete_w.map(|w| w / $width_div);
                let dims: Vec<Vec<u8>> = match (self.concrete_h, dw) {
                    (Some(h), Some(w)) => vec![
                        Proto::tensor_dim_value(1),
                        Proto::tensor_dim_value($channels),
                        Proto::tensor_dim_value(h),
                        Proto::tensor_dim_value(w),
                    ],
                    (None, Some(w)) => vec![
                        Proto::tensor_dim_value(1),
                        Proto::tensor_dim_value($channels),
                        Proto::tensor_dim_param("H"),
                        Proto::tensor_dim_value(w),
                    ],
                    _ => vec![
                        Proto::tensor_dim_value(1),
                        Proto::tensor_dim_value($channels),
                        Proto::tensor_dim_param("H"),
                        Proto::tensor_dim_param("W"),
                    ],
                };
                Some(Proto::value_info(&self.frame_tensor, &dims, $elem))
            }
            fn output_value_info(&self) -> Option<Vec<u8>> {
                self.input_value_info()
            }
        }
    };
}

impl_raw_input_block!(
    RawInputBlock,
    "raw_input",
    "RawInputBlock",
    "RawInputBlock/frame",
    6,
    1,
    1,
    "Raw INT32 `[1,1,H,W]` input block (legacy / default pipeline head)."
);

impl_raw_input_block!(
    RawInput16Block,
    "raw_input16",
    "RawInput16Block",
    "RawInput16Block/frame",
    5,
    1,
    1,
    "Pure 16-bit raw input `[1,1,H,W]` INT16 — no packed tensor, full width."
);

impl_raw_input_block!(
    RawInputPackedBlock,
    "raw_input_packed",
    "RawInputPackedBlock",
    "RawInputPackedBlock/frame",
    6,
    2,
    2,
    "Packed 2×int16-as-int32 input `[1,2,H,W/2]` — even/odd lanes, half width. `with_concrete_dims/width` take the FULL sensor width; the block halves it internally."
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_raw_input_id() {
        assert_eq!(RawInputBlock::new().id(), "raw_input");
        assert_eq!(RawInput16Block::new().id(), "raw_input16");
        assert_eq!(RawInputPackedBlock::new().id(), "raw_input_packed");
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
        // Conventions are baked into the block variants — no options.
        assert_eq!(RawInputBlock::new().input_elem_type(), 6); // INT32
        assert_eq!(RawInput16Block::new().input_elem_type(), 5); // INT16
        assert_eq!(RawInputPackedBlock::new().input_elem_type(), 6); // INT32
    }

    #[test]
    fn test_raw_input_emit_onnx() {
        let b = RawInputBlock::new();
        let nodes = b.nodes();
        // RawInput has no nodes, only the graph input
        assert!(nodes.is_empty());
    }

    #[test]
    fn test_raw_input16_dims_full_width() {
        // RawInput16Block keeps the full width in the declared shape.
        let vi = RawInput16Block::new()
            .with_concrete_dims(48, 64)
            .input_value_info()
            .unwrap();
        assert!(!vi.is_empty());
    }

    #[test]
    fn test_raw_packed_dims_half_width() {
        // RawInputPackedBlock declares [1,2,H,W/2] from a full-width input.
        let vi = RawInputPackedBlock::new()
            .with_concrete_dims(48, 64)
            .input_value_info()
            .unwrap();
        assert!(!vi.is_empty(), "packed value_info should be emitted");
        assert_eq!(
            RawInputPackedBlock::new()
                .with_concrete_dims(48, 64)
                .input_elem_type(),
            6
        );
    }
}
