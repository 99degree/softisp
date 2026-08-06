use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// UnpackBlc16Block -- INT16 Bayer -> FLOAT16 with fused BLC.
///
/// Treats each Bayer element as INT16, casts through FP32,
/// applies BLC, then converts to FLOAT16.
///
/// Input:  INT16  [1,1,H,W]  (native Bayer)
/// Output: FLOAT16 [1,1,H,W]  (BLC-corrected)
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
            10, // FLOAT16
        ))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let inp = self.effective_input();
        vec![
            Proto::node(
                "Cast",
                &[&inp],
                &[&format!("{}/f32", ns)],
                &[Proto::attribute_int("to", 1)], // FLOAT
            ),
            Proto::node(
                "Sub",
                &[&format!("{}/f32", ns), &format!("{}/blc_val", ns)],
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
                &[&format!("{}/clipped", ns)],
                &[],
            ),
            Proto::node(
                "Cast",
                &[&format!("{}/clipped", ns)],
                &[&self.frame_tensor],
                &[Proto::attribute_int("to", 10)], // FLOAT16
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
        10 // FLOAT16
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
        assert_eq!(b.output_elem_type(), 10);
    }

    #[test]
    fn test_unpack_blc16_nodes() {
        let mut b = UnpackBlc16Block::new();
        b.set_input_source("in/bayer");
        assert_eq!(b.nodes().len(), 4);
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
        let b1: Box<dyn IspBlock> = Box::new(
            RawInputBlock::new()
                .with_elem_type(5)
                .with_concrete_dims(48, 64),
        );
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
