use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// UnpackBlc16Block -- INT16 Bayer -> FLOAT32 with fused BLC.
///
/// Graph: Input(INT16) → Cast(to=FLOAT32) → Sub(BLC) → Clip(0, 65504).
/// The Cast makes the ONNX graph self-consistent: the Sub node receives
/// two FLOAT32 inputs instead of relying on MNN's silent INT16→F32 promotion.
///
/// Input:  INT16  [1,1,H,W]  (native Bayer — ONNX elem_type 5)
/// Output: FLOAT32 [1,1,H,W]  (BLC-corrected)
pub struct UnpackBlc16Block {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
}

impl Default for UnpackBlc16Block {
    fn default() -> Self {
        Self::new()
    }
}

impl UnpackBlc16Block {
    pub fn new() -> Self {
        Self {
            id: "unpack_blc16".into(),
            prev: None,
            next: None,
            frame_tensor: "UnpackBlc16Block/frame".into(),
            input_source: String::new(),
        }
    }

    fn effective_input(&self) -> String {
        if self.input_source.is_empty() {
            self.graph_input_name()
                .unwrap_or("UnpackBlc16Block/frame")
                .to_string()
        } else {
            self.input_source.clone()
        }
    }
}

impl IspBlock for UnpackBlc16Block {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "UnpackBlc16Block".into()
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
        vec![self.effective_input()]
    }
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.effective_input(),
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            5, // INT16
        ))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1, // FLOAT32 — MNN upcasts INT16 to FLOAT32
        ))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        // Cast INT16 input → FLOAT32, then Sub(BLC) → Clip.
        let ns = self.tensor_ns();
        let inp = self.effective_input();
        vec![
            Proto::node(
                "Cast",
                &[&inp],
                &[&format!("{}/float", ns)],
                &[Proto::attribute_int("to", 1)], // to FLOAT32
            ),
            Proto::node(
                "Sub",
                &[&format!("{}/float", ns), &format!("{}/blc_val", ns)],
                &[&format!("{}/subbed", ns)],
                &[],
            ),
            Proto::node(
                "Clip",
                &[
                    &format!("{}/subbed", ns),
                    &format!("{}/clip_min", ns),
                    &format!("{}/clip_max", ns),
                ],
                &[&self.frame_tensor],
                &[],
            ),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float_scalar(&format!("{}/blc_val", ns), 64.0),
            Proto::tensor_proto_float_scalar(&format!("{}/clip_min", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/clip_max", ns), 65504.0),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/blc_val", self.tensor_ns()), 1, vec![1, 1, 1, 1])]
    }

    fn input_elem_type(&self) -> i32 {
        5 // INT16
    }
    fn output_elem_type(&self) -> i32 {
        1 // FLOAT32 — MNN upcasts INT16 to FLOAT32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::GraphComposer;

    #[test]
    fn test_unpack_blc16_id() {
        assert_eq!(UnpackBlc16Block::new().id(), "unpack_blc16");
    }

    #[test]
    fn test_unpack_blc16_elem_types() {
        let b = UnpackBlc16Block::new();
        assert_eq!(b.input_elem_type(), 5);
        assert_eq!(b.output_elem_type(), 1);
    }

    #[test]
    fn test_unpack_blc16_nodes() {
        let mut b = UnpackBlc16Block::new();
        b.set_input_source("in/bayer");
        // Cast + Sub + Clip = 3 nodes
        assert_eq!(b.nodes().len(), 3);
    }

    #[test]
    fn test_unpack_blc16_initializers() {
        let b = UnpackBlc16Block::new();
        assert_eq!(b.initializers().len(), 3);
    }

    #[test]
    fn test_unpack_blc16_extra_inputs() {
        let b = UnpackBlc16Block::new();
        let extra = b.extra_inputs();
        assert_eq!(extra.len(), 1);
        assert_eq!(extra[0].2, vec![1, 1, 1, 1]);
    }

    #[test]
    fn test_unpack_blc16_tensors() {
        let mut b = UnpackBlc16Block::new();
        b.set_input_source("in/bayer");
        assert_eq!(b.input_tensors(), vec!["in/bayer".to_string()]);
        assert_eq!(
            b.output_tensors(),
            vec!["UnpackBlc16Block/frame".to_string()]
        );
    }

    #[test]
    fn test_unpack_blc16_pipeline() {
        use crate::blocks::*;
        let b1: Box<dyn IspBlock> = Box::new(RawInput16Block::new().with_concrete_dims(48, 64));
        let b2: Box<dyn IspBlock> = Box::new(UnpackBlc16Block::new());
        let b3: Box<dyn IspBlock> = Box::new(NormalizeBlock::new());

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2, b3];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(
            result.is_ok(),
            "pipeline compose failed: {:?}",
            result.err()
        );
    }
}
