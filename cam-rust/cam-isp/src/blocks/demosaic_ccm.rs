use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// Fused Demosaic + CCM block — replaces DemosaicBlock + CcmBlock.
///
/// Input:  [1, 4, H, W]   (CFA 4‑channel from UnpackCfaBlock or CfaBlock)
/// Output: [1, 3, H, W]   (RGB after demosaic + color correction)
///
/// Graph:  Conv1 (demosaic, static) → Conv2 (CCM, dynamic) → Clip(0,1)
///
/// Two Convs instead of one because the CCM matrix is dynamic (per‑frame
/// from AWB).  The session overhead is avoided by having both in one block.
///
/// Bayer pattern → demosaic weight mapping (same as DemosaicBlock):
///   RGGB (0): R=TL, G=avg(TR,BL), B=BR
///   GRBG (1): R=TR, G=avg(TL,BR), B=BL
///   GBRG (2): R=BL, G=avg(TL,BR), B=TR
///   BGGR (3): R=BR, G=avg(TR,BL), B=TL
pub struct DemosaicCcmBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub bayer_pattern: i32,
    pub in_ch: i64,
    pub out_ch: i64,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
}

impl DemosaicCcmBlock {
    pub fn new(bayer_pattern: i32) -> Self {
        Self {
            id: "demosaic_ccm".into(),
            prev: None,
            next: None,
            frame_tensor: "DemosaicCcmBlock/frame".into(),
            input_source: String::new(),
            bayer_pattern,
            in_ch: 4,
            out_ch: 3,
            concrete_h: None,
            concrete_w: None,
        }
    }

    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }

    /// Demosaic weights [3, 4, 1, 1] — same as DemosaicBlock.
    fn demosaic_weights(&self) -> Vec<f32> {
        match self.bayer_pattern {
            0 => vec![ // RGGB
                1.0, 0.0, 0.0, 0.0,
                0.0, 0.5, 0.5, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
            1 => vec![ // GRBG
                0.0, 1.0, 0.0, 0.0,
                0.5, 0.0, 0.0, 0.5,
                0.0, 0.0, 1.0, 0.0,
            ],
            2 => vec![ // GBRG
                0.0, 0.0, 1.0, 0.0,
                0.5, 0.0, 0.0, 0.5,
                0.0, 1.0, 0.0, 0.0,
            ],
            _ => vec![ // BGGR
                0.0, 0.0, 0.0, 1.0,
                0.0, 0.5, 0.5, 0.0,
                1.0, 0.0, 0.0, 0.0,
            ],
        }
    }
}

impl IspBlock for DemosaicCcmBlock {
    fn id(&self) -> &str { &self.id }

    fn tensor_ns(&self) -> String {
        "DemosaicCcmBlock".to_string()
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

    fn input_value_info(&self) -> Option<Vec<u8>> {
        let dims: Vec<Vec<u8>> = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(self.in_ch),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(self.in_ch),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ]
        };
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let dims: Vec<Vec<u8>> = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(self.out_ch),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(self.out_ch),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ]
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            // Conv1: demosaic (static) — 4ch CFA → 3ch RGB
            Proto::node(
                "Conv",
                &[&self.input_source, &format!("{}/demo_w", ns), &format!("{}/demo_b", ns)],
                &[&format!("{}/demo_out", ns)],
                &[
                    Proto::attribute_ints("kernel_shape", &[1, 1]),
                    Proto::attribute_ints("strides", &[1, 1]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                ],
            ),
            // Conv2: CCM (dynamic via extra_inputs) — 3ch RGB → 3ch RGB
            Proto::node(
                "Conv",
                &[&format!("{}/demo_out", ns), &format!("zzz_ccm/matrix"), &format!("zzz_ccm/bias")],
                &[&format!("{}/ccm_out", ns)],
                &[
                    Proto::attribute_ints("kernel_shape", &[1, 1]),
                    Proto::attribute_ints("strides", &[1, 1]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                ],
            ),
            // Clip to [0, 1]
            Proto::node(
                "Clip",
                &[&format!("{}/ccm_out", ns), &format!("{}/zero", ns), &format!("{}/one", ns)],
                &[&self.frame_tensor],
                &[],
            ),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            // Demosaic weights [3, 4, 1, 1]
            Proto::tensor_proto_float(
                &format!("{}/demo_w", ns),
                &[3, 4, 1, 1],
                &self.demosaic_weights(),
            ),
            // Demosaic bias [3]
            Proto::tensor_proto_float(
                &format!("{}/demo_b", ns),
                &[3],
                &[0.0; 3],
            ),
            // Clip constants
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
            // Default CCM matrix (identity) — overridden by extra_inputs at runtime
            Proto::tensor_proto_float(
                "zzz_ccm/matrix",
                &[3, 3, 1, 1],
                &[1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0],
            ),
            // Default CCM bias
            Proto::tensor_proto_float(
                "zzz_ccm/bias",
                &[3],
                &[0.0; 3],
            ),
        ]
    }

    /// Expose CCM matrix and bias as dynamic extra inputs.
    /// The `zzz_` prefix ensures these sort AFTER the main frame input
    /// in MNN's alphabetical input ordering.
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![
            ("zzz_ccm/matrix".into(), 1, vec![3, 3, 1, 1]),
            ("zzz_ccm/bias".into(), 1, vec![3]),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::GraphComposer;

    #[test]
    fn test_demosaic_ccm_block_ops() {
        let block = DemosaicCcmBlock::new(2);
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 3, "Should produce 3 nodes: Conv, Conv, Clip");
    }

    #[test]
    fn test_demosaic_ccm_initializers() {
        let block = DemosaicCcmBlock::new(0);
        let inits = block.initializers();
        assert_eq!(inits.len(), 6, "Should have 6 initializers");
    }

    #[test]
    fn test_demosaic_ccm_extra_inputs() {
        let block = DemosaicCcmBlock::new(2);
        let extra = block.extra_inputs();
        assert_eq!(extra.len(), 2, "Should expose 2 extra inputs");
        assert_eq!(extra[0].0, "zzz_ccm/matrix");
        assert_eq!(extra[1].0, "zzz_ccm/bias");
    }

    #[test]
    fn test_demosaic_ccm_bayer_patterns() {
        for pattern in 0..=3 {
            let block = DemosaicCcmBlock::new(pattern);
            let w = block.demosaic_weights();
            assert_eq!(w.len(), 12, "Pattern {}: 12 weights", pattern);
            let sum: f32 = w.iter().sum();
            assert!((sum - 3.0).abs() < 0.01, "Pattern {}: sum={}", pattern, sum);
        }
    }

    #[test]
    fn test_demosaic_ccm_pipeline_integration() {
        let b1: Box<dyn IspBlock> = Box::new(crate::blocks::RawInputBlock::new()
            .with_elem_type(1).with_concrete_dims(48, 64));
        let b2: Box<dyn IspBlock> = Box::new(crate::blocks::NormalizeBlock::new());
        let b3: Box<dyn IspBlock> = Box::new(crate::blocks::CfaBlock::new().with_concrete_dims(48, 64));
        let b4: Box<dyn IspBlock> = Box::new(DemosaicCcmBlock::new(2).with_concrete_dims(24, 32));

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2, b3, b4];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(result.is_ok(), "Pipeline through DemosaicCcmBlock: {:?}", result.err());
    }

    #[test]
    fn test_demosaic_ccm_replaces_separate_blocks() {
        // Verify the fused block produces the same number of nodes
        // as demosaic (1) + ccm (2) = 3 nodes
        let fused = DemosaicCcmBlock::new(2);
        assert_eq!(fused.nodes().len(), 3,
            "Fused should have same op count as separate demosaic(1)+ccm(2)");
    }
}
