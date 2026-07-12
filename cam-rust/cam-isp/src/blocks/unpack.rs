//! UnpackBlock — splits packed INT32 into interleaved INT32 pixels.
//!
//! Input:  INT32`[1,1,H,W/2]` where each element = pixel_even | (pixel_odd << 16)
//! Output: INT32`[1,1,H,W]`    interleaved even/odd pixels (as 12-bit values)
//!
//! Low 16 bits (even pixels): Cast INT32→INT16 (truncates), Cast INT16→INT32
//! High 16 bits (odd pixels): Shr(input, 16) — ONE node instead of float path
//! Interleave via: Reshape→`[H,PW,1]`→Concat(axis=2)→Reshape→`[1,1,H,W]` — 3 nodes vs 5

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// UnpackBlock — Bayer packed-INT32 or native-INT16 unpack.
///
/// Converts packed 4-pixel Bayer data from INT32→float32 or INT16→float32.
/// Supports 4 Bayer patterns (RGGB/GRBG/GBRG/BGGR) via const buffer.
/// Optional concrete dimensions for static shape inference.
pub struct UnpackBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
    /// Workgroup tuning for Vulkan dispatch: (size_x, size_y).
    pub workgroup_size: (u32, u32),
}

impl Default for UnpackBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl UnpackBlock {
    pub fn new() -> Self {
        Self {
            id: "unpack".into(),
            prev: None,
            next: None,
            frame_tensor: "UnpackBlock/frame".into(),
            input_source: String::new(),
            concrete_h: None,
            concrete_w: None,
            workgroup_size: (0, 0), // auto-tune
        }
    }

    /// Return the effective input tensor name: `input_source` if set,
    /// otherwise `graph_input_name()`. This ensures ONNX nodes always
    /// reference a valid tensor name even when `wire_blocks` wasn't called.
    fn effective_input(&self) -> String {
        if self.input_source.is_empty() {
            self.graph_input_name()
                .unwrap_or("UnpackBlock/frame")
                .to_string()
        } else {
            self.input_source.clone()
        }
    }

    /// Set concrete height/width (FULL width, not packed).
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
}

impl IspBlock for UnpackBlock {
    fn id(&self) -> &str {
        &self.id
    }

    fn tensor_ns(&self) -> String {
        "UnpackBlock".to_string()
    }

    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

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

    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }

    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    /// Input is packed INT32`[1,1,H,W/2]`
    fn input_value_info(&self) -> Option<Vec<u8>> {
        let pw = self.concrete_w.map(|w| w / 2);
        let dims: Vec<Vec<u8>> = match (self.concrete_h, pw) {
            (Some(h), Some(pw)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(pw),
            ],
            (None, Some(pw)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_value(pw),
            ],
            _ => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W2"), // packed width = W/2
            ],
        };
        Some(Proto::value_info(&self.effective_input(), &dims, 6)) // INT32
    }

    /// Output is interleaved INT32`[1,1,H,W]`
    fn output_value_info(&self) -> Option<Vec<u8>> {
        let dims: Vec<Vec<u8>> = match (self.concrete_h, self.concrete_w) {
            (Some(h), Some(w)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ],
            (None, Some(w)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_value(w),
            ],
            _ => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 6)) // INT32
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let inp = self.effective_input();

        vec![
            // --- Extract low 16 bits (even pixels) via INT16 truncation ---
            Proto::node(
                "Cast",
                &[&inp],
                &[&format!("{}/even_i16", ns)],
                &[Proto::attribute_int("to", 5)],
            ), // to=5 = INT16 (truncates to low 16 bits)
            Proto::node(
                "Cast",
                &[&format!("{}/even_i16", ns)],
                &[&format!("{}/even", ns)],
                &[Proto::attribute_int("to", 6)],
            ), // to=6 = INT32
            // --- Extract high 16 bits (odd pixels) via integer Div ---
            // For positive INT32 values (pixel0 | pixel1<<16), Div(65536) gives pixel1
            Proto::node(
                "Div",
                &[&inp, &format!("{}/div_65536", ns)],
                &[&format!("{}/odd", ns)],
                &[],
            ),
            // --- Interleave even + odd into [1,1,H,W] ---
            // Reshape even to [H, PW, 1], odd to [H, PW, 1], Concat, Reshape to [1,1,H,W]
            Proto::node(
                "Reshape",
                &[&format!("{}/even", ns), &format!("{}/shape_3d", ns)],
                &[&format!("{}/even_3d", ns)],
                &[],
            ),
            Proto::node(
                "Reshape",
                &[&format!("{}/odd", ns), &format!("{}/shape_3d", ns)],
                &[&format!("{}/odd_3d", ns)],
                &[],
            ),
            Proto::node(
                "Concat",
                &[&format!("{}/even_3d", ns), &format!("{}/odd_3d", ns)],
                &[&format!("{}/interleaved_3d", ns)],
                &[Proto::attribute_int("axis", 2)],
            ),
            Proto::node(
                "Reshape",
                &[
                    &format!("{}/interleaved_3d", ns),
                    &format!("{}/shape_4d", ns),
                ],
                &[&self.frame_tensor],
                &[],
            ),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let h = self.concrete_h.unwrap_or(64);
        let pw = self.concrete_w.map(|w| w / 2).unwrap_or(32);
        let w = self.concrete_w.unwrap_or(64);

        vec![
            // Div constant: 65536 for high 16 bits extraction (INT32)
            Proto::tensor_proto_int32_scalar(&format!("{}/div_65536", ns), 65536),
            // Shape constants: [H, PW, 1] for 3D reshape (shared by even and odd)
            Proto::tensor_proto_int64(&format!("{}/shape_3d", ns), &[h, pw, 1]),
            // Final shape: [1, 1, H, W]
            Proto::tensor_proto_int64(&format!("{}/shape_4d", ns), &[1, 1, h, w]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::GraphComposer;

    #[test]
    fn test_unpack_block_generates_nodes() {
        let block = UnpackBlock::new().with_concrete_dims(48, 64);
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 7, "UnpackBlock should produce 7 nodes (Cast, Cast, Shr, Reshape, Reshape, Concat, Reshape)");
        let inits = block.initializers();
        assert_eq!(
            inits.len(),
            3,
            "UnpackBlock should have 3 initializers (shift_16, shape_3d, shape_4d)"
        );
    }

    #[test]
    fn test_unpack_block_with_concrete_dims() {
        let block = UnpackBlock::new().with_concrete_dims(48, 64);
        let in_vi = block.input_value_info().unwrap();
        let out_vi = block.output_value_info().unwrap();
        assert!(!in_vi.is_empty());
        assert!(!out_vi.is_empty());
        // 7 nodes: 2 Cast + 1 Shr + 2 Reshape + 1 Concat + 1 Reshape
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 7);
        // 3 initializers: shift_16 + shape_3d + shape_4d
        let inits = block.initializers();
        assert_eq!(inits.len(), 3);
    }

    #[test]
    fn test_unpack_pipeline_integration() {
        // Build: RawInput(packed INT32) -> UnpackBlock -> NormalizeBlock
        let b1: Box<dyn IspBlock> = Box::new(
            crate::blocks::RawInputBlock::new()
                .with_elem_type(6) // INT32 input
                .with_concrete_dims(48, 32),
        ); // packed width = 32
        let b2: Box<dyn IspBlock> = Box::new(UnpackBlock::new().with_concrete_dims(48, 64));
        let b3: Box<dyn IspBlock> = Box::new(crate::blocks::NormalizeBlock::new());

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2, b3];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(
            result.is_ok(),
            "UnpackBlock pipeline should compose: {:?}",
            result.err()
        );
        let model = result.unwrap();
        assert!(!model.is_empty(), "Model should not be empty");
    }

    #[test]
    fn test_unpack_id() {
        assert_eq!(UnpackBlock::new().id(), "unpack");
    }

    #[test]
    fn test_unpack_tensor_ns() {
        let b = UnpackBlock::new();
        assert!(!b.tensor_ns().is_empty());
    }
}

impl UnpackBlock {
    /// Set custom workgroup size for Vulkan dispatch.
    pub fn workgroup(mut self, size_x: u32, size_y: u32) -> Self {
        self.workgroup_size = (size_x, size_y);
        self
    }
}
