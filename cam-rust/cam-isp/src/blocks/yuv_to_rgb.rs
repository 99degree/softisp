//! YUV to RGB conversion block.
//!
//! Converts YUV input (from Yuv420InputBlock, Nv12InputBlock, Nv21InputBlock)
//! to RGB/Bayer format for the standard ISP pipeline.
//!
//! This block performs the standard ITU-R BT.601 or BT.709 YUV-to-RGB conversion
//! using a 3x3 color matrix, applied via a 1x1 convolution.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// YUV to RGB color space conversion matrix types.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum YuvColorSpace {
    /// ITU-R BT.601 (standard definition)
    Bt601,
    /// ITU-R BT.709 (high definition)
    #[default]
    Bt709,
    /// Full-range (JPEG) YUV
    FullRange,
}

/// YUV to RGB conversion block.
///
/// Converts YUV [1,3,H,W] or [1,2,H,W] FLOAT32 input to RGB [1,3,H,W] FLOAT32
/// using a 1x1 convolution with the appropriate color matrix.
#[derive(Debug, Default)]
pub struct YuvToRgbBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub color_space: YuvColorSpace,
}

impl YuvToRgbBlock {
    pub fn new() -> Self {
        Self {
            id: "yuv_to_rgb".to_string(),
            prev: None,
            next: None,
            frame_tensor: "YuvToRgbBlock/frame".to_string(),
            input_source: String::new(),
            color_space: YuvColorSpace::Bt709,
        }
    }

    /// Set the YUV color space standard.
    pub fn with_color_space(mut self, cs: YuvColorSpace) -> Self {
        self.color_space = cs;
        self
    }

    /// Return the effective input tensor name: `input_source` if set,
    /// otherwise `graph_input_name()`.
    fn effective_input(&self) -> String {
        if self.input_source.is_empty() {
            self.graph_input_name()
                .unwrap_or("YuvToRgbBlock/frame")
                .to_string()
        } else {
            self.input_source.clone()
        }
    }

    /// Get the 3x3 YUV-to-RGB conversion matrix for the selected color space.
    fn color_matrix(&self) -> [f32; 9] {
        match self.color_space {
            YuvColorSpace::Bt601 => {
                // BT.601 limited range (16-235)
                // R = 1.164*(Y-16) + 1.596*(V-128)
                // G = 1.164*(Y-16) - 0.813*(V-128) - 0.391*(U-128)
                // B = 1.164*(Y-16) + 2.018*(U-128)
                [
                    1.164, 0.0, 1.596, // R row
                    1.164, -0.391, -0.813, // G row
                    1.164, 2.018, 0.0, // B row
                ]
            }
            YuvColorSpace::Bt709 => {
                // BT.709 limited range (16-235)
                // R = 1.164*(Y-16) + 1.793*(V-128)
                // G = 1.164*(Y-16) - 0.534*(V-128) - 0.213*(U-128)
                // B = 1.164*(Y-16) + 2.115*(U-128)
                [
                    1.164, 0.0, 1.793, // R row
                    1.164, -0.213, -0.534, // G row
                    1.164, 2.115, 0.0, // B row
                ]
            }
            YuvColorSpace::FullRange => {
                // Full range (0-255) - JPEG style
                // R = Y + 1.402*V
                // G = Y - 0.344*U - 0.714*V
                // B = Y + 1.772*U
                [
                    1.0, 0.0, 1.402, // R row
                    1.0, -0.344, -0.714, // G row
                    1.0, 1.772, 0.0, // B row
                ]
            }
        }
    }

    /// Get the bias vector for the color space (accounting for 128 offset in limited range).
    fn bias(&self) -> [f32; 3] {
        match self.color_space {
            YuvColorSpace::Bt601 => [
                -128.0 * 1.596,          // R bias
                128.0 * (0.391 + 0.813), // G bias
                -128.0 * 2.018,          // B bias
            ],
            YuvColorSpace::Bt709 => [
                -128.0 * 1.793,          // R bias
                128.0 * (0.213 + 0.534), // G bias
                -128.0 * 2.115,          // B bias
            ],
            YuvColorSpace::FullRange => [0.0, 0.0, 0.0],
        }
    }
}

impl IspBlock for YuvToRgbBlock {
    #[inline]
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "YuvToRgbBlock".to_string()
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
        // Expects input from YUV block: either "Yuv420InputBlock/frame", "Nv12InputBlock/frame", or "Nv21InputBlock/frame"
        vec![]
    }
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        // Accepts [1,3,H,W] or [1,2,H,W] FLOAT32
        None
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        use crate::onnx::proto::Proto;
        let dims = vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(3), // RGB output
            Proto::tensor_dim_param("H"),
            Proto::tensor_dim_param("W"),
        ];
        Some(Proto::value_info(&self.frame_tensor, &dims, 1)) // FLOAT32
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        // The input is [1,C,H,W] where C=3 (YUV_420) or C=2 (NV12/NV21)
        // We need to apply the color matrix via a 1x1 convolution.
        // For now, we'll create a minimal working version that the MNN converter can process.

        let ns = self.tensor_ns();
        let mat_name = format!("{}/color_matrix", ns);
        let bias_name = format!("{}/bias", ns);

        vec![Proto::node(
            "Conv",
            &[&self.effective_input(), &mat_name, &bias_name],
            &[&self.frame_tensor],
            &[
                Proto::attribute_ints("kernel_shape", &[1, 1]),
                Proto::attribute_ints("strides", &[1, 1]),
                Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                Proto::attribute_int("group", 1),
            ],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        use crate::onnx::proto::Proto;
        let ns = self.tensor_ns();
        let mat_name = format!("{}/color_matrix", ns);
        let _bias_name = format!("{}/bias", ns);

        let mat = self.color_matrix();
        let bias = self.bias();

        vec![
            // Color matrix: [3, 3, 1, 1] for 3-channel input
            Proto::tensor_proto_float(&mat_name, &[3, 3, 1, 1], &mat),
            // Bias: [3]
            Proto::tensor_proto_float(&_bias_name, &[3], &bias),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        // No runtime inputs - matrix and bias are baked as initializers
        vec![]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_yuv_to_rgb_id() {
        assert_eq!(YuvToRgbBlock::new().id(), "yuv_to_rgb");
    }

    #[test]
    fn test_yuv_to_rgb_matrix_bt601() {
        let block = YuvToRgbBlock::new().with_color_space(YuvColorSpace::Bt601);
        let mat = block.color_matrix();
        // R row: 1.164, 0.0, 1.596
        assert!((mat[0] - 1.164).abs() < 0.001);
        assert!((mat[2] - 1.596).abs() < 0.001);
    }

    #[test]
    fn test_yuv_to_rgb_matrix_bt709() {
        let block = YuvToRgbBlock::new().with_color_space(YuvColorSpace::Bt709);
        let mat = block.color_matrix();
        assert!((mat[0] - 1.164).abs() < 0.001);
        assert!((mat[2] - 1.793).abs() < 0.001);
    }

    #[test]
    fn test_yuv_to_rgb_bias_bt601() {
        let block = YuvToRgbBlock::new().with_color_space(YuvColorSpace::Bt601);
        let bias = block.bias();
        assert!((bias[0] - (-128.0 * 1.596)).abs() < 0.01);
    }

    #[test]
    fn test_yuv_to_rgb_emit_onnx() {
        let block = YuvToRgbBlock::new();
        let nodes = block.nodes();
        assert!(!nodes.is_empty());

        let inits = block.initializers();
        assert!(!inits.is_empty());
    }
}
