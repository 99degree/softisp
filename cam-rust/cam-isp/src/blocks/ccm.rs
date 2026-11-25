//! CcmBlock — Color Correction Matrix via Conv(1×1).
//!
//! Applies 3×3 color matrix to RGB channels. Supports instance-aware
//! naming for multiple CCM stages (LSC, CCM, LDCI placeholders).

use crate::onnx::proto::Proto;
use crate::pipeline::{BlockOpsetMode, IspBlock};
use crate::tensor_names;

/// CcmBlock — Color Correction Matrix via per-channel Conv(1×1).
///
/// Applies user-configurable color correction matrix to transform
/// sensor RGB → sRGB. Parameters hot-swappable at runtime.
#[derive(Debug)]
pub struct CcmBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    instance: String,
    in_ch: i64,
    out_ch: i64,
    /// Color correction matrix (row-major: [out_ch * in_ch])
    matrix: Vec<f32>,
    /// Per-channel bias
    bias: Vec<f32>,
    /// Override the emitted opset. `None` keeps the Primitive `Conv + Clip`.
    pub forced_mode: Option<BlockOpsetMode>,
}
impl Default for CcmBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl CcmBlock {
    /// Create a default CCM block (single instance, 3 input channels).
    pub fn new() -> Self {
        Self::with_instance("")
    }
    /// Create a CCM block with a unique instance suffix.
    /// Required when multiple CcmBlocks are used in the same pipeline
    /// (e.g., LSC, CCM, LDCI, unsharp placeholders) to avoid duplicate
    /// tensor names.
    pub fn with_instance(suffix: &str) -> Self {
        let inst = suffix.to_string();
        let ns = if suffix.is_empty() {
            "CcmBlock".to_string()
        } else {
            format!("CcmBlock_{}", suffix)
        };
        let bid = if suffix.is_empty() {
            "ccm".to_string()
        } else {
            format!("ccm_{}", suffix)
        };
        Self {
            id: bid,
            prev: None,
            next: None,
            frame_tensor: format!("{}/frame", ns),
            input_source: String::new(),
            instance: inst,
            in_ch: 3,  // default: RGB input
            out_ch: 3, // default: RGB output
            matrix: vec![
                1.0, 0.0, 0.0, // R row
                0.0, 1.0, 0.0, // G row
                0.0, 0.0, 1.0,
            ], // B row
            bias: vec![0.0; 3],
            forced_mode: None,
        }
    }
    /// Set input and output channel count.
    /// Used for LSC placeholder which receives 4 Bayer channels.
    pub fn with_channels(mut self, ch: i64) -> Self {
        self.in_ch = ch;
        self.out_ch = ch;
        self.matrix = vec![0.0; (ch * ch) as usize];
        for i in 0..ch as usize {
            self.matrix[i * (ch as usize + 1)] = 1.0; // identity diagonal
        }
        self.bias = vec![0.0; ch as usize];
        self
    }

    /// Select the emitted opset mode (Primitive or MnnIspMin).
    pub fn with_opset_mode(mut self, mode: BlockOpsetMode) -> Self {
        self.forced_mode = Some(mode);
        self
    }
}

impl IspBlock for CcmBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        if self.instance.is_empty() {
            "CcmBlock".to_string()
        } else {
            format!("CcmBlock_{}", self.instance)
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
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("C"),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let matrix = tensor_names::ccm_matrix_name(&self.instance);
        let bias = tensor_names::ccm_bias_name(&self.instance);
        if self.opset_mode() == BlockOpsetMode::MnnIspMin {
            // One `isp.ccm` dispatch instead of Conv(1x1) + Clip. The matrix
            // arrives at binding 5 as a runtime tensor, which is the same
            // `{ns}/matrix` the primitive Conv consumes — the engine already
            // writes it, so only the node changes.
            //
            // `isp_ccm.comp` declares exactly nine coefficients and no bias, so
            // this path is only equivalent while the bias is zero. A non-zero
            // bias would be dropped silently, so it stays on the primitive form.
            let flat: Vec<f32> = self.matrix.clone();
            if self.bias.iter().all(|b| *b == 0.0) && flat.len() == 9 {
                return vec![Proto::node_with_domain(
                    "isp.ccm",
                    "isp",
                    &[&self.input_source, &matrix],
                    &[&self.frame_tensor],
                    &[Proto::attribute_int("elementwise", 1)],
                )];
            }
        }
        vec![
            Proto::node(
                "Conv",
                &[&self.input_source, &matrix, &bias],
                &[&format!("{}/applied", ns)],
                &[
                    Proto::attribute_ints("kernel_shape", &[1, 1]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                    Proto::attribute_ints("strides", &[1, 1]),
                ],
            ),
            Proto::node(
                "Clip",
                &[
                    &format!("{}/applied", ns),
                    &format!("{}/zero", ns),
                    &format!("{}/one", ns),
                ],
                &[&self.frame_tensor],
                &[],
            ),
        ]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    /// `CcmBlock` can emit `isp.ccm`, so it declares the op type the other
    /// fusion-capable blocks declare.
    ///
    /// This is static metadata — it says the block *can* fuse, not that it is
    /// fusing. `MnnEngine::chain_has_fused_op` pairs it with
    /// `opset_mode() == MnnIspMin` to decide whether the graph will actually
    /// contain an `isp.*` op, and therefore whether fp16 tensor storage is safe.
    /// Without this declaration a chain that fused only its CCM was misjudged as
    /// unfused, kept fp16, and the shader read halves as floats — a black frame
    /// with no error.
    fn custom_op_types(&self) -> Option<&[&str]> {
        Some(&["isp.ccm"])
    }

    fn opset_mode(&self) -> BlockOpsetMode {
        self.forced_mode.unwrap_or(BlockOpsetMode::Primitive)
    }

    fn set_opset_mode(&mut self, mode: BlockOpsetMode) {
        self.forced_mode = Some(mode);
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        vec![
            (
                tensor_names::ccm_matrix_name(&self.instance),
                1,
                vec![self.out_ch, self.in_ch, 1, 1],
            ),
            (
                tensor_names::ccm_bias_name(&self.instance),
                1,
                vec![self.out_ch],
            ),
            (format!("{}/zero", ns).to_string(), 1, vec![]),
            (format!("{}/one", ns).to_string(), 1, vec![]),
        ]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        vec![
            // Conv weights [out_ch, in_ch, 1, 1] — row-major.
            (
                tensor_names::ccm_matrix_name(&self.instance),
                self.matrix
                    .iter()
                    .flat_map(|v| v.to_ne_bytes())
                    .collect::<Vec<u8>>(),
            ),
            (
                tensor_names::ccm_bias_name(&self.instance),
                self.bias
                    .iter()
                    .flat_map(|v| v.to_ne_bytes())
                    .collect::<Vec<u8>>(),
            ),
            (
                format!("{}/zero", ns).to_string(),
                (0.0f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/one", ns).to_string(),
                (1.0f32).to_ne_bytes().to_vec(),
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ccm_id() {
        assert_eq!(CcmBlock::new().id(), "ccm");
    }

    #[test]
    fn test_ccm_instance_id() {
        let b = CcmBlock::with_instance("lsc");
        assert_eq!(b.id(), "ccm_lsc");
    }

    #[test]
    fn test_ccm_nodes() {
        let mut b = CcmBlock::new();
        b.set_input_source("in/rgb");
        let nodes = b.nodes();
        // Conv + Clip = 2 nodes
        assert_eq!(nodes.len(), 2);
    }

    #[test]
    fn test_ccm_with_channels() {
        let b = CcmBlock::new().with_channels(4);
        let inits = b.extra_input_defaults();
        assert_eq!(inits.len(), 4);
    }

    #[test]
    fn test_ccm_different_channels() {
        let b3 = CcmBlock::new().with_channels(3);
        let b4 = CcmBlock::new().with_channels(4);
        // 4-channel has more initializer data than 3-channel
        assert!(b4.extra_input_defaults().len() >= b3.extra_input_defaults().len());
    }

    #[test]
    fn test_ccm_extra_inputs() {
        let b = CcmBlock::new();
        let extra = b.extra_inputs();
        assert_eq!(extra[0].0, tensor_names::CCM_MATRIX);
        assert_eq!(extra[1].0, tensor_names::CCM_BIAS);

        let production = CcmBlock::with_instance("ccm");
        assert_eq!(production.extra_inputs()[0].0, tensor_names::CCM_MATRIX);

        let other = CcmBlock::with_instance("lsc");
        assert_eq!(other.extra_inputs()[0].0, "CcmBlock_lsc/matrix");
        assert_eq!(other.extra_inputs()[1].0, "CcmBlock_lsc/bias");
    }
}
