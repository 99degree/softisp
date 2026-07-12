use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// BlcBlock — Black Level Correction with instance-aware naming.
///
/// Subtracts black level offset per Bayer channel.
/// Supports multiple instances via `with_instance()` for pipeline stages
/// like DPC + black level in a single model.
pub struct BlcBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    instance: String,
}
impl Default for BlcBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl BlcBlock {
    /// Create BlcBlock with default instance (single-use).
    pub fn new() -> Self {
        Self::with_instance("")
    }

    /// Create BlcBlock with a unique instance suffix.
    /// Required when multiple BlcBlocks are used in the same pipeline
    /// (e.g., one for DPC, one for black level correction) to avoid
    /// duplicate tensor names.
    pub fn with_instance(suffix: &str) -> Self {
        let inst = suffix.to_string();
        let ns = if suffix.is_empty() {
            "BlcBlock".to_string()
        } else {
            format!("BlcBlock_{}", suffix)
        };
        let bid = if suffix.is_empty() {
            "blc".to_string()
        } else {
            format!("blc_{}", suffix)
        };
        Self {
            id: bid,
            prev: None,
            next: None,
            frame_tensor: format!("{}/frame", ns),
            input_source: String::new(),
            instance: inst,
        }
    }
}
impl IspBlock for BlcBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        if self.instance.is_empty() {
            "BlcBlock".to_string()
        } else {
            format!("BlcBlock_{}", self.instance)
        }
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
    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.input_source,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("C"),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> {
        self.input_value_info()
    }
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::node(
                "Sub",
                &[&self.input_source, &format!("{}/blc_vals", ns)],
                &[&format!("{}/subbed", ns)],
                &[],
            ),
            Proto::node(
                "Clip",
                &[
                    &format!("{}/subbed", ns),
                    &format!("{}/zero", ns),
                    &format!("{}/one", ns),
                ],
                &[&self.frame_tensor],
                &[],
            ),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float(
                &format!("{}/blc_vals", ns),
                &[1, 4, 1, 1],
                &[0.0, 0.0, 0.0, 0.0],
            ),
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(
            format!("{}/blc_vals", self.tensor_ns()),
            1,
            vec![1, 4, 1, 1],
        )]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_blc_id() {
        assert_eq!(BlcBlock::new().id(), "blc");
    }

    #[test]
    fn test_blc_instance_id() {
        let b = BlcBlock::with_instance("dpc");
        assert_eq!(b.id(), "blc_dpc");
    }

    #[test]
    fn test_blc_emit_onnx() {
        let mut b = BlcBlock::new();
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        // Sub + Clip = 2 nodes
        assert_eq!(nodes.len(), 2);
    }

    #[test]
    fn test_blc_extra_inputs() {
        let b = BlcBlock::new();
        let extra = b.extra_inputs();
        assert_eq!(extra.len(), 1);
        assert_eq!(extra[0].2, vec![1, 4, 1, 1]);
    }

    #[test]
    fn test_blc_different_instances() {
        let b1 = BlcBlock::new();
        let b2 = BlcBlock::with_instance("dpc");
        assert_ne!(b1.id(), b2.id());
    }

    #[test]
    fn test_blc_tensor_ns_unique() {
        let b1 = BlcBlock::new();
        let b2 = BlcBlock::with_instance("dpc");
        assert_ne!(b1.tensor_ns(), b2.tensor_ns());
    }
}
