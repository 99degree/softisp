use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct ToneBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
}
impl ToneBlock {
    pub fn new() -> Self { Self { id: "tone".into(), prev: None, next: None, frame_tensor: "ToneBlock/frame".into(), input_source: String::new() } }
}
impl IspBlock for ToneBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "ToneBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn input_value_info(&self) -> Option<Vec<u8>> { Some(Proto::value_info(&self.input_source, &[Proto::tensor_dim_value(1),Proto::tensor_dim_value(3),Proto::tensor_dim_param("H"),Proto::tensor_dim_param("W")], 1)) }
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::node("Mul", &[&self.input_source, &format!("{}/contrast", ns)], &[&format!("{}/contrasted", ns)], &[]),
            Proto::node("Add", &[&format!("{}/contrasted", ns), &format!("{}/brightness", ns)], &[&format!("{}/brightened", ns)], &[]),
            Proto::node("Clip", &[&format!("{}/brightened", ns), &format!("{}/zero", ns), &format!("{}/one", ns)], &[&self.frame_tensor], &[]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float_scalar(&format!("{}/gamma_recip", ns), 1.0),
            Proto::tensor_proto_float_scalar(&format!("{}/contrast", ns), 1.0),
            Proto::tensor_proto_float_scalar(&format!("{}/brightness", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        vec![
            (format!("{}/contrast", ns), 1, vec![1]),
            (format!("{}/brightness", ns), 1, vec![1]),
            (format!("{}/gamma_recip", ns), 1, vec![1]),
        ]
    }

    /// Signals that ToneBlock can make use of FCS, LDCI, and EE aux blocks.
    fn signals_aux(&self) -> Vec<String> {
        vec!["fcs".into(), "ldci".into(), "ee".into()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tone_id() {
        assert_eq!(ToneBlock::new().id(), "tone");
    }

    #[test]
    fn test_tone_emit_onnx() {
        let mut b = ToneBlock::new();
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        // Mul + Add + Clip = 3 nodes
        assert_eq!(nodes.len(), 3);
    }

    #[test]
    fn test_tone_signals_aux() {
        let b = ToneBlock::new();
        assert_eq!(b.signals_aux(), vec!["fcs", "ldci", "ee"]);
    }
}

