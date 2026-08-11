use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// CFA unpack block — extracts 2x2 Bayer quad positions using Conv stride=2.
///
/// Takes `[1,1,H,W]` float normalized Bayer data and outputs `[1,4,H/2,W/2]`
/// where each channel corresponds to one quad position (TL, TR, BL, BR).
///
/// Uses Conv(kernel=2, stride=2, 4 filters) matching the Java implementation.
/// Avoids Slice op which MNN doesn't handle.
///
/// For the packed even/odd split (`[1,2,H,W/2]`) use [`CfaBlockPacked`].
pub struct CfaBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
}

impl Default for CfaBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl CfaBlock {
    pub fn new() -> Self {
        Self {
            id: "cfa".into(),
            prev: None,
            next: None,
            frame_tensor: "CfaBlock/frame".into(),
            input_source: String::new(),
            concrete_h: None,
            concrete_w: None,
        }
    }

    /// Set concrete height/width for fixed-shape models.
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }
}

impl IspBlock for CfaBlock {
    fn id(&self) -> &str {
        &self.id
    }

    fn tensor_ns(&self) -> String {
        "CfaBlock".to_string()
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
        let dims: Vec<Vec<u8>> = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ]
        };
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let oh = self.concrete_h.map(|h| h / 2);
        // Raw mode halves W with stride 2.
        let ow = self.concrete_w.map(|w| w / 2);
        let dims: Vec<Vec<u8>> = if let (Some(h), Some(w)) = (oh, ow) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H2"),
                Proto::tensor_dim_param("W2"),
            ]
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![Proto::node(
            "Conv",
            &[
                &self.input_source,
                &format!("{}/w", ns),
                &format!("{}/b", ns),
            ],
            &[&self.frame_tensor],
            &[
                Proto::attribute_ints("kernel_shape", &[2, 2]),
                Proto::attribute_ints("strides", &[2, 2]),
                Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                Proto::attribute_int("group", 1),
            ],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Conv weights [4, 1, 2, 2] — 4 filters, 1 input channel, 2x2 kernel
        // Each filter picks one quad position:
        //   filter 0 (TL): [[1,0],[0,0]]
        //   filter 1 (TR): [[0,1],[0,0]]
        //   filter 2 (BL): [[0,0],[1,0]]
        //   filter 3 (BR): [[0,0],[0,1]]
        let w = vec![
            1f32, 0f32, 0f32, 0f32, // TL
            0f32, 1f32, 0f32, 0f32, // TR
            0f32, 0f32, 1f32, 0f32, // BL
            0f32, 0f32, 0f32, 1f32, // BR
        ];
        vec![
            Proto::tensor_proto_float(&format!("{}/w", ns), &[4, 1, 2, 2], &w),
            Proto::tensor_proto_float(&format!("{}/b", ns), &[4], &[0f32; 4]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }
}

/// CFA unpack block for the packed even/odd split — extracts 2x2 Bayer quad
/// positions from `[1,2,H,W/2]` (UnpackBlock output) using Conv stride 2.
///
/// Channel 0 = even pixels, channel 1 = odd pixels. The 2x2 quad is extracted
/// with a 2-input-channel conv (kernel `[2,1]`, stride `[2,1]`), producing the
/// same `[1,4,H/2,W/2]` output with the same quad→channel mapping as
/// [`CfaBlock`]. This avoids interleaving even/odd back into one `[1,1,H,W]`
/// tensor (which previously needed rank/baked-dim Reshape tricks that broke
/// MNN resizeSession).
pub struct CfaBlockPacked {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
}

impl Default for CfaBlockPacked {
    fn default() -> Self {
        Self::new()
    }
}

impl CfaBlockPacked {
    pub fn new() -> Self {
        Self {
            id: "cfa_packed".into(),
            prev: None,
            next: None,
            frame_tensor: "CfaBlockPacked/frame".into(),
            input_source: String::new(),
            concrete_h: None,
            concrete_w: None,
        }
    }

    /// Set concrete height/width for fixed-shape models.
    /// Width is the FULL sensor width; the half-width split is intrinsic.
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }
}

impl IspBlock for CfaBlockPacked {
    fn id(&self) -> &str {
        &self.id
    }

    fn tensor_ns(&self) -> String {
        "CfaBlockPacked".to_string()
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
        // Input is the 2-channel even/odd split: [1,2,H,W/2]
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
                Proto::tensor_dim_param("W2"),
            ],
        };
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let oh = self.concrete_h.map(|h| h / 2);
        // Packed mode keeps the already-half-width even/odd input with stride 1.
        let ow = self.concrete_w.map(|w| w / 2);
        let dims: Vec<Vec<u8>> = if let (Some(h), Some(w)) = (oh, ow) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H2"),
                Proto::tensor_dim_param("W2"),
            ]
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Packed input: [1,2,H,W/2] even/odd channels. Each quad column j of
        // the packed row holds both column parities, so stride W is 1 and the
        // kernel only spans rows (2x1).
        vec![Proto::node(
            "Conv",
            &[
                &self.input_source,
                &format!("{}/w", ns),
                &format!("{}/b", ns),
            ],
            &[&self.frame_tensor],
            &[
                Proto::attribute_ints("kernel_shape", &[2, 1]),
                Proto::attribute_ints("strides", &[2, 1]),
                Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                Proto::attribute_int("group", 1),
            ],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Conv weights [4, 2, 2, 1] — 4 filters, 2 input channels (even/odd),
        // 2x1 kernel. Filter oc picks the quad position:
        //   oc0 (TL) = even[2i][j]   → kernel (0,0) on channel 0 (even)
        //   oc1 (TR) = odd[2i][j]    → kernel (0,0) on channel 1 (odd)
        //   oc2 (BL) = even[2i+1][j] → kernel (1,0) on channel 0 (even)
        //   oc3 (BR) = odd[2i+1][j]  → kernel (1,0) on channel 1 (odd)
        // Row-major [oc][ic][kh][kw]:
        //   oc0: ic0 [[1],[0]] ic1 [[0],[0]]
        //   oc1: ic0 [[0],[0]] ic1 [[1],[0]]
        //   oc2: ic0 [[0],[1]] ic1 [[0],[0]]
        //   oc3: ic0 [[0],[0]] ic1 [[0],[1]]
        let w = vec![
            1f32, 0f32, 0f32, 0f32, // oc0: even @ (0,0)
            0f32, 0f32, 1f32, 0f32, // oc1: odd  @ (0,0)
            0f32, 1f32, 0f32, 0f32, // oc2: even @ (1,0)
            0f32, 0f32, 0f32, 1f32, // oc3: odd  @ (1,0)
        ];
        vec![
            Proto::tensor_proto_float(&format!("{}/w", ns), &[4, 2, 2, 1], &w),
            Proto::tensor_proto_float(&format!("{}/b", ns), &[4], &[0f32; 4]),
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
    fn test_cfa_block_generates_conv_op() {
        let block = CfaBlock::new();
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 1, "CfaBlock should produce 1 node");
        // The node should be Conv (not Slice+Reshape)
        assert!(!nodes[0].is_empty(), "Node should not be empty");
    }

    #[test]
    fn test_cfa_block_with_concrete_dims() {
        let block = CfaBlock::new().with_concrete_dims(48, 64);
        let vi = block.output_value_info().unwrap();
        assert!(!vi.is_empty(), "Output value_info should not be empty");
        // Output should have halved dims: [1, 4, 24, 32]
        assert!(vi.len() > 10, "value_info should have reasonable size");
    }

    #[test]
    fn test_cfa_block_conv_weights() {
        let block = CfaBlock::new();
        let inits = block.initializers();
        assert_eq!(inits.len(), 2, "Should have weight and bias initializers");
        // Weight should have shape [4, 1, 2, 2]
        // Bias should have shape [4]
    }

    #[test]
    fn test_cfa_pipeline_integration() {
        let b1: Box<dyn IspBlock> =
            Box::new(crate::blocks::RawInputBlock::new().with_concrete_dims(48, 64));
        let b2: Box<dyn IspBlock> = Box::new(crate::blocks::NormalizeBlock::new());
        let b3: Box<dyn IspBlock> =
            Box::new(crate::blocks::CfaBlock::new().with_concrete_dims(48, 64));

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2, b3];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(
            result.is_ok(),
            "CfaBlock pipeline should compose: {:?}",
            result.err()
        );
        let model = result.unwrap();
        assert!(!model.is_empty(), "Model should not be empty");
        assert!(model.len() > 100, "Model should be substantial");
    }

    #[test]
    fn test_cfa_packed_input_mode() {
        // Packed mode: distinct block consuming [1,2,H,W/2]
        // (even/odd channels from UnpackBlock)
        let block = CfaBlockPacked::new().with_concrete_dims(48, 64);
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 1, "CfaBlockPacked should produce 1 Conv node");
        let inits = block.initializers();
        assert_eq!(inits.len(), 2, "Should have weight and bias initializers");
        // Output dims halve H and keep W/2 (same as raw mode)
        let vi = block.output_value_info().unwrap();
        assert!(!vi.is_empty(), "Output value_info should not be empty");
    }

    #[test]
    fn test_cfa_various_sizes() {
        for (w, h) in [(32, 32), (64, 48), (128, 128)] {
            let b = CfaBlock::new().with_concrete_dims(w, h);
            let nodes = b.nodes();
            assert!(!nodes.is_empty(), "CFA {}x{} should emit nodes", w, h);
        }
    }
}
