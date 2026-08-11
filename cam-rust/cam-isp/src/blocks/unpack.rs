//! UnpackBlock — converts the engine-split even/odd INT32 lanes to float.
//!
//! Input:  INT32 `[1,2,H,W/2]` — channel 0 = even pixels, channel 1 = odd
//!         pixels (each a zero-extended u16 value, 0..65535).
//! Output: FLOAT32 `[1,2,H,W/2]`.
//!
//! The even/odd lane split itself is performed engine-side (CPU): MNN's
//! Vulkan backend cannot execute integer elementwise ops (Div/Mod/Sub) or
//! INT16 casts, so extracting the two u16 lanes from a packed INT32 in-graph
//! is impossible on GPU. Splitting on the host is exact and signedness-free.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// UnpackBlock — engine-split even/odd Bayer lanes, INT32 → FLOAT32.
pub struct UnpackBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
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

    /// Input is split even/odd INT32 `[1,2,H,W/2]`
    fn input_value_info(&self) -> Option<Vec<u8>> {
        let pw = self.concrete_w.map(|w| w / 2);
        let dims: Vec<Vec<u8>> = match (self.concrete_h, pw) {
            (Some(h), Some(pw)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(2),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(pw),
            ],
            (None, Some(pw)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(2),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_value(pw),
            ],
            _ => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(2),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W2"), // packed width = W/2
            ],
        };
        Some(Proto::value_info(&self.effective_input(), &dims, 6)) // INT32
    }

    /// Output is FLOAT32 `[1,2,H,W/2]` (consumed by CfaBlock packed conv)
    fn output_value_info(&self) -> Option<Vec<u8>> {
        let pw = self.concrete_w.map(|w| w / 2);
        let dims: Vec<Vec<u8>> = match (self.concrete_h, pw) {
            (Some(h), Some(pw)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(2),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(pw),
            ],
            (None, Some(pw)) => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(2),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_value(pw),
            ],
            _ => vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(2),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W2"), // packed width = W/2
            ],
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 1)) // FLOAT32
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let inp = self.effective_input();

        vec![
            // INT32 → FLOAT32 lane conversion. Attribute-only, rank-4, and
            // Vulkan-supported (Cast int32→float is a 4-byte→4-byte op).
            Proto::node(
                "Cast",
                &[&inp],
                &[&self.frame_tensor],
                &[Proto::attribute_int("to", 1)],
            ), // to=1 = FLOAT
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }
}

impl UnpackBlock {
    /// Return the effective input tensor name: `input_source` if set,
    /// otherwise `graph_input_name()`.
    fn effective_input(&self) -> String {
        if self.input_source.is_empty() {
            self.graph_input_name()
                .unwrap_or("UnpackBlock/frame")
                .to_string()
        } else {
            self.input_source.clone()
        }
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
        assert_eq!(nodes.len(), 1, "UnpackBlock should produce 1 Cast node");
        let inits = block.initializers();
        assert_eq!(inits.len(), 0, "UnpackBlock should have no initializers");
    }

    #[test]
    fn test_unpack_block_with_concrete_dims() {
        let block = UnpackBlock::new().with_concrete_dims(48, 64);
        let in_vi = block.input_value_info().unwrap();
        let out_vi = block.output_value_info().unwrap();
        assert!(!in_vi.is_empty());
        assert!(!out_vi.is_empty());
        // 1 node: Cast (engine pre-splits even/odd lanes)
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 1);
        // no initializers
        let inits = block.initializers();
        assert_eq!(inits.len(), 0);
    }

    #[test]
    fn test_unpack_pipeline_integration() {
        // Build: RawInput(split INT32) -> UnpackBlock -> NormalizeBlock
        let b1: Box<dyn IspBlock> =
            Box::new(crate::blocks::RawInputPackedBlock::new().with_concrete_dims(48, 64)); // packed width = 32
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
