//! ColorSpaceBlock — RGB ↔ YUV conversion via 3×3 Conv.
//!
//! Uses standard BT.601 or BT.709 matrices.
//! Output channels: Y, U, V (or inverse to R, G, B).
//!
//! ONNX subgraph:
//!   1. Conv(input, matrix) → [1, 3, H, W] (YUV or RGB)
//!
//! Conv kernel: [3, 3, 3, 3] with group=3 and per-channel weights,
//! or [1, 3, 3, 3] for full matrix multiplication.

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

#[derive(Clone, Copy, PartialEq)]
pub enum ColorSpace {
    RgbToYuv601,
    Yuv601ToRgb,
    RgbToYuv709,
    Yuv709ToRgb,
}

pub struct ColorSpaceBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub mode: ColorSpace,
}

impl ColorSpaceBlock {
    pub fn new(mode: ColorSpace) -> Self {
        Self {
            id: "colorspace".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "ColorSpaceBlock/frame".into(),
            input_source: String::new(),
            mode,
        }
    }
}

impl IspBlock for ColorSpaceBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "ColorSpace".into() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.into(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev_block.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev_block = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next_block.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next_block = Some(block); }

    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn graph_output_name(&self) -> Option<&str> { Some(&self.frame_tensor) }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.input_source,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let kernel_name = format!("{}/kernel", ns);
        vec![Proto::node(
            "Conv", &[&self.input_source, &kernel_name], &[&self.frame_tensor],
            &[Proto::attribute_ints("kernel_shape", &[1, 1]),
              Proto::attribute_ints("pads", &[0, 0, 0, 0])],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // BT.601 RGB→YUV: [1, 3, 3, 3] = (out_ch, in_ch, kH, kW)
        // Full 3×3 matrix: output = matrix @ input
        let m = match self.mode {
            // BT.601: Y=0.299R+0.587G+0.114B, U=-0.169R-0.331G+0.500B+128, V=0.500R-0.419G-0.081B+128
            ColorSpace::RgbToYuv601 => vec![
                0.299,  0.587,  0.114,
               -0.169, -0.331,  0.500,
                0.500, -0.419, -0.081,
            ],
            // BT.601 inverse: R=Y+1.402(V-128), G=Y-0.344(U-128)-0.714(V-128), B=Y+1.772(U-128)
            ColorSpace::Yuv601ToRgb => vec![
                1.000,  0.000,  1.402,
                1.000, -0.344, -0.714,
                1.000,  1.772,  0.000,
            ],
            // BT.709: Y=0.2126R+0.7152G+0.0722B, U=-0.1146R-0.3854G+0.500B+128, V=0.500R-0.4542G-0.0458B+128
            ColorSpace::RgbToYuv709 => vec![
                0.2126,  0.7152,  0.0722,
               -0.1146, -0.3854,  0.500,
                0.500,  -0.4542, -0.0458,
            ],
            // BT.709 inverse
            ColorSpace::Yuv709ToRgb => vec![
                1.000,  0.000,  1.5748,
                1.000, -0.1873, -0.4681,
                1.000,  1.8556,  0.000,
            ],
        };
        vec![Proto::tensor_proto_float(
            &format!("{}/kernel", ns), &[1, 3, 3, 3], &m)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_colorspace_rgb_to_yuv() {
        let block = ColorSpaceBlock::new(ColorSpace::RgbToYuv601);
        assert_eq!(block.input_tensors().len(), 1);
        assert_eq!(block.output_tensors().len(), 1);
        assert_eq!(block.initializers().len(), 1);
    }

    #[test]
    fn test_colorspace_yuv_to_rgb() {
        let block = ColorSpaceBlock::new(ColorSpace::Yuv601ToRgb);
        let inits = block.initializers();
        assert_eq!(inits.len(), 1);
    }

    #[test]
    fn test_colorspace_emits_conv() {
        let block = ColorSpaceBlock::new(ColorSpace::RgbToYuv709);
        let nodes = block.nodes();
        assert!(nodes.iter().any(|n| String::from_utf8_lossy(n).contains("Conv")));
    }

    #[test]
    fn test_colorspace_709_different_from_601() {
        let b1 = ColorSpaceBlock::new(ColorSpace::RgbToYuv601);
        let b2 = ColorSpaceBlock::new(ColorSpace::RgbToYuv709);
        assert_ne!(b1.initializers(), b2.initializers(),
            "BT.601 and BT.709 should have different kernels");
    }
}
