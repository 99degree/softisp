//! UnpackBlock — splits packed INT32 into interleaved INT32 pixels.
//!
//! Input:  INT32[1,1,H,W/2] where each element = pixel_even | (pixel_odd << 16)
//! Output: INT32[1,1,H,W]    interleaved even/odd pixels (as 12-bit values)
//!
//! Low 16 bits (even pixels): Cast INT32→INT16 (truncates), Cast INT16→INT32
//! High 16 bits (odd pixels): Cast INT32→FLOAT, Div(65536), Floor, Cast→INT32
//! Interleave via: Reshape→[2,W/2]→Concat(axis=1)→Reshape→[1,1,H,W]

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct UnpackBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,  // FULL width (original W, NOT packed)
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

    /// Input is packed INT32[1,1,H,W/2]
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
                Proto::tensor_dim_param("W2"),  // packed width = W/2
            ],
        };
        Some(Proto::value_info(&self.input_source, &dims, 6)) // INT32
    }

    /// Output is interleaved INT32[1,1,H,W]
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

        // Build shape constants for Reshape ops
        // (created in initializers(); referenced here for clarity)

        vec![
            // --- Extract low 16 bits (even pixels) ---
            Proto::node("Cast",
                &[&self.input_source],
                &[&format!("{}/even_i16", ns)],
                &[Proto::attribute_int("to", 5)]),  // to=5 = INT16 (truncates to low 16 bits)
            Proto::node("Cast",
                &[&format!("{}/even_i16", ns)],
                &[&format!("{}/even", ns)],
                &[Proto::attribute_int("to", 6)]),  // to=6 = INT32

            // --- Extract high 16 bits (odd pixels) ---
            Proto::node("Cast",
                &[&self.input_source],
                &[&format!("{}/odd_float", ns)],
                &[Proto::attribute_int("to", 1)]),  // to=1 = FLOAT
            Proto::node("Div",
                &[&format!("{}/odd_float", ns), &format!("{}/max_val", ns)],
                &[&format!("{}/odd_scaled", ns)],
                &[]),
            Proto::node("Floor",
                &[&format!("{}/odd_scaled", ns)],
                &[&format!("{}/odd_trunc", ns)],
                &[]),
            Proto::node("Cast",
                &[&format!("{}/odd_trunc", ns)],
                &[&format!("{}/odd", ns)],
                &[Proto::attribute_int("to", 6)]),  // to=6 = INT32

            // --- Interleave even + odd into [1,1,H,W] ---
            // Reshape even to [H, PW] → [H, PW, 1]
            Proto::node("Reshape",
                &[&format!("{}/even", ns), &format!("{}/shape_2d_even", ns)],
                &[&format!("{}/even_2d", ns)],
                &[]),
            Proto::node("Reshape",
                &[&format!("{}/even_2d", ns), &format!("{}/shape_3d_even", ns)],
                &[&format!("{}/even_3d", ns)],
                &[]),
            // Reshape odd to [H, PW] → [H, PW, 1]
            Proto::node("Reshape",
                &[&format!("{}/odd", ns), &format!("{}/shape_2d_odd", ns)],
                &[&format!("{}/odd_2d", ns)],
                &[]),
            Proto::node("Reshape",
                &[&format!("{}/odd_2d", ns), &format!("{}/shape_3d_odd", ns)],
                &[&format!("{}/odd_3d", ns)],
                &[]),
            // Concat along axis=2 → [H, PW, 2]
            Proto::node("Concat",
                &[&format!("{}/even_3d", ns), &format!("{}/odd_3d", ns)],
                &[&format!("{}/interleaved_3d", ns)],
                &[Proto::attribute_int("axis", 2)]),
            // Reshape to [1, 1, H, W]
            Proto::node("Reshape",
                &[&format!("{}/interleaved_3d", ns), &format!("{}/shape_4d", ns)],
                &[&self.frame_tensor],
                &[]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let h  = self.concrete_h.unwrap_or(64);
        let pw = self.concrete_w.map(|w| w / 2).unwrap_or(32);
        let w  = self.concrete_w.unwrap_or(64);

        vec![
            // Div constant: 65536.0 for high 16 bits
            Proto::tensor_proto_float_scalar(&format!("{}/max_val", ns), 65536.0),
            // Shape constants for Reshape ops (concrete values if known)
            Proto::tensor_proto_int64(&format!("{}/shape_2d_even", ns), &[h, pw]),
            Proto::tensor_proto_int64(&format!("{}/shape_2d_odd",  ns), &[h, pw]),
            Proto::tensor_proto_int64(&format!("{}/shape_3d_even", ns), &[h, pw, 1]),
            Proto::tensor_proto_int64(&format!("{}/shape_3d_odd",  ns), &[h, pw, 1]),
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
        assert_eq!(nodes.len(), 12, "UnpackBlock should produce 12 nodes");
        let inits = block.initializers();
        assert_eq!(inits.len(), 6, "UnpackBlock should have 6 initializers");
    }

    #[test]
    fn test_unpack_block_with_concrete_dims() {
        let block = UnpackBlock::new().with_concrete_dims(48, 64);
        let in_vi = block.input_value_info().unwrap();
        let out_vi = block.output_value_info().unwrap();
        assert!(!in_vi.is_empty());
        assert!(!out_vi.is_empty());
        // 12 nodes: 6 for extraction + 6 for interleave/reshape
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 12);
        // 6 initializers: 1 float + 5 int64 shape constants
        let inits = block.initializers();
        assert_eq!(inits.len(), 6);
    }

    #[test]
    fn test_unpack_pipeline_integration() {
        // Build: RawInput(packed INT32) -> UnpackBlock -> NormalizeBlock
        let b1: Box<dyn IspBlock> = Box::new(crate::blocks::RawInputBlock::new()
            .with_elem_type(6)   // INT32 input
            .with_concrete_dims(48, 32));  // packed width = 32
        let b2: Box<dyn IspBlock> = Box::new(UnpackBlock::new().with_concrete_dims(48, 64));
        let b3: Box<dyn IspBlock> = Box::new(crate::blocks::NormalizeBlock::new());

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2, b3];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(result.is_ok(), "UnpackBlock pipeline should compose: {:?}", result.err());
        let model = result.unwrap();
        assert!(!model.is_empty(), "Model should not be empty");
    }
}
