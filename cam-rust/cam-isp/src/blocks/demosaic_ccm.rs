use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// Fused Demosaic + CCM block — single Conv1×1 with runtime‑fused weights.
///
/// Input:  [1, 4, H, W]   (CFA 4‑channel from UnpackCfaBlock or CfaBlock)
/// Output: [1, 3, H, W]   (RGB after demosaic + color correction)
///
/// Graph:  Conv1×1 → Clip(0, 1)   — **2 nodes** (was 3 with 2 Convs)
///
/// The Conv weight tensor `DemosaicCcm/w` [3, 4, 1, 1] is exposed as an
/// extra input.  At build time it is initialised to the demosaic weights
/// (identity CCM fused).  At runtime, the controller pre‑multiplies the
/// CCM matrix with the demosaic weights and sets the fused result:
///
///   fused_w[i, j] = Σₖ ccm[i, k] · demosaic[k, j]    (3×3 @ 3×4 = 3×4)
///
/// 12 FMA ops per frame — negligible.  Saves one entire Conv execution
/// compared to two sequential Conv1×1 ops.
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
    /// Sensor max value for de-normalizing INT16 input.
    pub sensor_max: f32,
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
            sensor_max: 1023.0,
        }
    }

    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }

    pub fn with_sensor_max(mut self, max: f32) -> Self {
        self.sensor_max = max;
        self
    }

    /// Demosaic weights [3, 4, 1, 1] — same as DemosaicBlock.
    pub fn demosaic_weights(&self) -> Vec<f32> {
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

    /// Compute fused weights = CCM × demosaic (3×3 @ 3×4 = 3×4).
    /// Returns a 12‑element vector in CHW order [3, 4, 1, 1].
    ///
    /// Call this at runtime when the CCM matrix changes, then set the
    /// result as the value of the `DemosaicCcm/w` extra input tensor.
    pub fn fuse_weights(&self, ccm_matrix: &[f32; 9]) -> Vec<f32> {
        let demo = self.demosaic_weights();     // [3, 4] row-major
        // ccm_matrix is [3, 3] row-major
        // fused[i, j] = Σₖ ccm[i, k] * demo[k, j]
        let mut fused = vec![0.0f32; 12];
        for i in 0..3 {
            for j in 0..4 {
                let mut sum = 0.0;
                for k in 0..3 {
                    sum += ccm_matrix[i * 3 + k] * demo[k * 4 + j];
                }
                fused[i * 4 + j] = sum;
            }
        }
        fused
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

    fn input_elem_type(&self) -> i32 { 5 } // INT16 — Bayer domain integer

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
        Some(Proto::value_info(&self.input_source, &dims, 5)) // INT16 input
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
            // INT16 Bayer → FLOAT (switch to float at CCM boundary)
            Proto::node(
                "Cast",
                &[&self.input_source],
                &[&format!("{}/input_f32", ns)],
                &[Proto::attribute_int("to", 1)],
            ),
            // Normalize: Div by sensor_max to get [0,1] range for Conv
            Proto::node(
                "Div",
                &[&format!("{}/input_f32", ns), &format!("{}/div_max", ns)],
                &[&format!("{}/normed", ns)],
                &[],
            ),
            // Single Conv1×1 with runtime-fused weights (demosaic × CCM)
            Proto::node(
                "Conv",
                &[&format!("{}/normed", ns), &format!("{}/w", ns), &format!("{}/b", ns)],
                &[&format!("{}/conv_out", ns)],
                &[
                    Proto::attribute_ints("kernel_shape", &[1, 1]),
                    Proto::attribute_ints("strides", &[1, 1]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                ],
            ),
            // Clip to [0, 1]
            Proto::node(
                "Clip",
                &[&format!("{}/conv_out", ns), &format!("{}/zero", ns), &format!("{}/one", ns)],
                &[&self.frame_tensor],
                &[],
            ),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            // Fused weights = demosaic @ identity_ccm = raw demosaic weights [3, 4, 1, 1].
            // At runtime, the controller pre-multiplies the CCM matrix and
            // overrides this tensor via the extra_inputs mechanism.
            Proto::tensor_proto_float(
                &format!("{}/w", ns),
                &[3, 4, 1, 1],
                &self.demosaic_weights(),
            ),
            // Bias [3]
            Proto::tensor_proto_float(
                &format!("{}/b", ns),
                &[3],
                &[0.0; 3],
            ),
            // Clip constants
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
            // Sensor max for normalization
            Proto::tensor_proto_float_scalar(&format!("{}/div_max", ns), self.sensor_max),
        ]
    }

    /// Expose the fused weight tensor and bias as dynamic extra inputs.
    /// The `zzz_` prefix ensures these sort AFTER the main frame input
    /// in MNN's alphabetical input ordering.
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![
            (format!("{}/w", self.tensor_ns()), 1, vec![3, 4, 1, 1]),
            (format!("{}/b", self.tensor_ns()), 1, vec![3]),
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
        assert_eq!(nodes.len(), 4, "Should produce 4 nodes: Cast, Div, Conv, Clip");
    }

    #[test]
    fn test_demosaic_ccm_initializers() {
        let block = DemosaicCcmBlock::new(0);
        let inits = block.initializers();
        assert_eq!(inits.len(), 5, "Should have 5 initializers (w, b, zero, one, div_max)");
    }

    #[test]
    fn test_demosaic_ccm_extra_inputs() {
        let block = DemosaicCcmBlock::new(2);
        let extra = block.extra_inputs();
        assert_eq!(extra.len(), 2, "Should expose 2 extra inputs");
        assert_eq!(extra[0].0, "DemosaicCcmBlock/w");
        assert_eq!(extra[1].0, "DemosaicCcmBlock/b");
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
    fn test_fuse_weights_identity() {
        // Fusing with identity CCM should produce the raw demosaic weights
        let block = DemosaicCcmBlock::new(0); // RGGB
        let identity: [f32; 9] = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ];
        let fused = block.fuse_weights(&identity);
        let raw = block.demosaic_weights();
        assert_eq!(fused.len(), raw.len());
        for (f, r) in fused.iter().zip(raw.iter()) {
            assert!((f - r).abs() < 1e-6, "Fused identity should match raw: {} vs {}", f, r);
        }
    }

    #[test]
    fn test_fuse_weights_diagonal() {
        // Diagonal CCM with different gains per channel
        let block = DemosaicCcmBlock::new(0);
        let diag: [f32; 9] = [
            2.0, 0.0, 0.0,
            0.0, 1.5, 0.0,
            0.0, 0.0, 0.8,
        ];
        let fused = block.fuse_weights(&diag);
        // R-channel weights should be doubled
        assert!((fused[0] - 2.0).abs() < 1e-6, "R ch0 fused: {}", fused[0]);
        // G-channel weights (average of ch1, ch2) should be 1.5×
        assert!((fused[5] - 0.75).abs() < 1e-6, "G ch1 fused: {}", fused[5]); // 1.5 * 0.5
        assert!((fused[6] - 0.75).abs() < 1e-6, "G ch2 fused: {}", fused[6]);
        // B-channel weight should be 0.8×
        assert!((fused[11] - 0.8).abs() < 1e-6, "B ch3 fused: {}", fused[11]);
    }

    #[test]
    fn test_fuse_weights_non_diagonal() {
        // Non-diagonal CCM: RGB channels mix
        let block = DemosaicCcmBlock::new(0);
        let ccm: [f32; 9] = [
            1.0, 0.1, 0.05,
            0.2, 1.0, 0.1,
            0.05, 0.1, 1.0,
        ];
        let fused = block.fuse_weights(&ccm);
        // R output from R input: ch0 weight = ccm[0,0]*1.0 + ccm[0,1]*0.0 + ccm[0,2]*0.0 = 1.0
        assert!((fused[0] - 1.0).abs() < 1e-6, "R from ch0: {}", fused[0]);
        // R output from G input (ch1): ccm[0,1]*0.5 + ccm[0,2]*0.0 = 0.1*0.5 = 0.05
        assert!((fused[1] - 0.05).abs() < 1e-6, "R from ch1: {}", fused[1]);
        // R output from G input (ch2): ccm[0,1]*0.5 = 0.1*0.5 = 0.05
        assert!((fused[2] - 0.05).abs() < 1e-6, "R from ch2: {}", fused[2]);
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
    fn test_demosaic_ccm_nodes_count() {
        // 1 Cast + 1 Div + 1 Conv + 1 Clip = 4 nodes
        let fused = DemosaicCcmBlock::new(2);
        assert_eq!(fused.nodes().len(), 4,
            "Cast+Div+Conv+Clip should produce 4 nodes, got {}", fused.nodes().len());
    }
}
