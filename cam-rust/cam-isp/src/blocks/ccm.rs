use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct CcmBlock {
    pub id: String, pub prev: Option<Box<dyn IspBlock>>, pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String, pub input_source: String,
    instance: String,
    in_ch: i64,
    out_ch: i64,
}
impl CcmBlock {
    /// Create a default CCM block (single instance, 3 input channels).
    pub fn new() -> Self { Self::with_instance("") }
    /// Create a CCM block with a unique instance suffix.
    /// Required when multiple CcmBlocks are used in the same pipeline
    /// (e.g., LSC, CCM, LDCI, unsharp placeholders) to avoid duplicate
    /// tensor names.
    pub fn with_instance(suffix: &str) -> Self {
        let inst = suffix.to_string();
        let ns = if suffix.is_empty() { "CcmBlock".to_string() } else { format!("CcmBlock_{}", suffix) };
        let bid = if suffix.is_empty() { "ccm".to_string() } else { format!("ccm_{}", suffix) };
        Self {
            id: bid,
            prev: None, next: None,
            frame_tensor: format!("{}/frame", ns),
            input_source: String::new(),
            instance: inst,
            in_ch: 3,  // default: RGB input
            out_ch: 3, // default: RGB output
        }
    }
    /// Set input and output channel count.
    /// Used for LSC placeholder which receives 4 Bayer channels.
    pub fn with_channels(mut self, ch: i64) -> Self {
        self.in_ch = ch;
        self.out_ch = ch;
        self
    }
}
impl IspBlock for CcmBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String {
        if self.instance.is_empty() { "CcmBlock".to_string() } else { format!("CcmBlock_{}", self.instance) }
    }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }
    fn input_value_info(&self) -> Option<Vec<u8>> { Some(Proto::value_info(&self.input_source, &[Proto::tensor_dim_value(1),Proto::tensor_dim_param("C"),Proto::tensor_dim_param("H"),Proto::tensor_dim_param("W")], 1)) }
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::node("Conv", &[&self.input_source, &format!("{}/matrix", ns), &format!("{}/bias", ns)], &[&format!("{}/applied", ns)],
                &[Proto::attribute_ints("kernel_shape", &[1, 1]),
                  Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                  Proto::attribute_ints("strides", &[1, 1])]),
            Proto::node("Clip", &[&format!("{}/applied", ns), &format!("{}/zero", ns), &format!("{}/one", ns)], &[&self.frame_tensor], &[]),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Identity matrix for Conv: [out_ch, in_ch, 1, 1]
        // Only diagonal elements are 1.0 when out_ch == in_ch.
        let n = self.out_ch.max(self.in_ch) as usize;
        let mut identity = vec![0.0f32; n * n];
        for i in 0..self.out_ch.min(self.in_ch) as usize {
            identity[i * n + i] = 1.0;
        }
        // Trim to actual dimensions
        let actual_size = (self.out_ch * self.in_ch) as usize;
        identity.truncate(actual_size);
        
        vec![
            Proto::tensor_proto_float(&format!("{}/matrix", ns), &[self.out_ch, self.in_ch, 1, 1], &identity),
            Proto::tensor_proto_float(&format!("{}/bias", ns), &[self.out_ch], &vec![0.0; self.out_ch as usize]),
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/matrix", self.tensor_ns()), 1, vec![self.out_ch, self.in_ch, 1, 1])]
    }
}
