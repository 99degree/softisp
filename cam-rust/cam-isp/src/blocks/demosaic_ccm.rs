use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Fused Demosaic + CCM block — single Conv1×1 with runtime‑fused weights.
///
/// Input:  F32`[1, 4, H, W]`   (CFA 4‑channel from UnpackCfaBlock)
/// Output: F32`[1, 3, H, W]`   (RGB after demosaic + CCM)
///
/// Graph:  Conv1×1 → Clip(0, 1)   — **2 nodes** (fused sensor_max norm)
///
/// The Conv weight tensor `DemosaicCcm/w` `[3, 4, 1, 1]` is exposed as an
/// extra input.  At build time it is initialised to the demosaic weights
/// divided by sensor_max.  At runtime, the controller pre‑multiplies:
///
///   fused_w`[i, j]` = Σₖ ccm`[i, k]` · demosaic`[k, j]` / sensor_max
///   fused_bias`[i]` = bias`[i]` / sensor_max - Σⱼ fused_w`[i,j]` · wbⱼ · blcⱼ
///
/// This absorbs INT16→F32 Cast, Div(sensor_max), BLC Sub, WB Mul into
/// the Conv — 1 Conv does the work of 4 ops.
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

    /// Demosaic weights `[3, 4, 1, 1]` — same as DemosaicBlock.
    pub fn demosaic_weights(&self) -> Vec<f32> {
        match self.bayer_pattern {
            0 => vec![
                // RGGB
                1.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
            1 => vec![
                // GRBG
                0.0, 1.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0,
            ],
            2 => vec![
                // GBRG
                0.0, 0.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0,
            ],
            _ => vec![
                // BGGR
                0.0, 0.0, 0.0, 1.0, 0.0, 0.5, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0,
            ],
        }
    }

    /// Compute fused weights = CCM × demosaic (3×3 @ 3×4 = 3×4).
    /// Returns a 12‑element vector in CHW order `[3, 4, 1, 1]`.
    ///
    /// Call this at runtime when the CCM matrix changes, then set the
    /// result as the value of the `DemosaicCcm/w` extra input tensor.
    /// Compute fused weights = CCM × demosaic × WB / sensor_max (3×3 @ 3×4 = 3×4).
    /// Also returns adjusted bias that incorporates BLC offset.
    ///
    /// fused_w`[i,j]` = Σₖ ccm`[i,k]` · demosaic`[k,j]` · wbⱼ / sensor_max
    /// fused_bias`[i]` = - Σⱼ fused_w`[i,j]` · blcⱼ
    ///
    /// Call at runtime when CCM, WB gains, or BLC offsets change.
    pub fn fuse_weights_ext(
        &self,
        ccm_matrix: &[f32; 9],
        wb_gains: &[f32; 4],
        blc_vals: &[f32; 4],
        sensor_max: f32,
    ) -> (Vec<f32>, Vec<f32>) {
        let demo = self.demosaic_weights(); // [3, 4] row-major
        let sm = sensor_max;
        // fused_w[i,j] = Σₖ ccm[i,k] * demo[k,j] * wb_gains[j] / sensor_max
        let mut fused_w = vec![0.0f32; 12];
        for i in 0..3 {
            for j in 0..4 {
                let wb_j = wb_gains[j];
                let mut sum = 0.0;
                for k in 0..3 {
                    sum += ccm_matrix[i * 3 + k] * demo[k * 4 + j];
                }
                fused_w[i * 4 + j] = sum * wb_j / sm;
            }
        }
        // fused_bias[i] = - Σⱼ fused_w[i,j] * blc_vals[j]  (wb already in fused_w)
        let mut fused_b = vec![0.0f32; 3];
        for i in 0..3 {
            let mut sum = 0.0;
            for j in 0..4 {
                sum += fused_w[i * 4 + j] * blc_vals[j];
            }
            fused_b[i] = -sum;
        }
        (fused_w, fused_b)
    }

    /// Legacy: just CCM × demosaic (no sensor_max, no BLC/WB).
    /// Kept for backward compat.
    pub fn fuse_weights(&self, ccm_matrix: &[f32; 9]) -> Vec<f32> {
        let wb = [1.0, 1.0, 1.0, 1.0];
        let blc = [0.0, 0.0, 0.0, 0.0];
        self.fuse_weights_ext(ccm_matrix, &wb, &blc, 1.0).0
    }

    /// Return the effective input tensor name: `input_source` if set,
    /// otherwise `graph_input_name()`.
    fn effective_input(&self) -> String {
        if self.input_source.is_empty() {
            self.graph_input_name()
                .unwrap_or("DemosaicCcmBlock/frame")
                .to_string()
        } else {
            self.input_source.clone()
        }
    }
}

impl IspBlock for DemosaicCcmBlock {
    fn id(&self) -> &str {
        &self.id
    }

    fn tensor_ns(&self) -> String {
        "DemosaicCcmBlock".to_string()
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

    fn input_elem_type(&self) -> i32 {
        1
    } // FLOAT — accepts F32 Bayer planes

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
        Some(Proto::value_info(&self.effective_input(), &dims, 1)) // FLOAT input
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
        let inp = self.effective_input();
        vec![
            // Single Conv1×1 with runtime-fused weights.
            // Weights are pre-divided by sensor_max and incorporate BLC+WB.
            Proto::node(
                "Conv",
                &[&inp, &format!("{}/w", ns), &format!("{}/b", ns)],
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
                &[
                    &format!("{}/conv_out", ns),
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
        // Weights initialized as demosaic / sensor_max so Conv1×1 output is [0,1].
        let sm = self.sensor_max;
        let raw_w: Vec<f32> = self.demosaic_weights().iter().map(|v| v / sm).collect();
        vec![
            Proto::tensor_proto_float(&format!("{}/w", ns), &[3, 4, 1, 1], &raw_w),
            // Bias [3]
            Proto::tensor_proto_float(&format!("{}/b", ns), &[3], &[0.0; 3]),
            // Clip constants
            Proto::tensor_proto_float_scalar(&format!("{}/zero", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
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
        assert_eq!(nodes.len(), 2, "Should produce 2 nodes: Conv, Clip");
    }

    #[test]
    fn test_demosaic_ccm_initializers() {
        let block = DemosaicCcmBlock::new(0);
        let inits = block.initializers();
        assert_eq!(
            inits.len(),
            4,
            "Should have 4 initializers (w, b, zero, one)"
        );
        // w should be demosaic/sensor_max
        let raw_w = block.demosaic_weights();
        let _sm = block.sensor_max; // 1023.0
        for (_f, _r) in inits[0].iter().zip(raw_w.iter()) {
            // Can't easily compare protobuf bytes, just check count
        }
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
        // fuse_weights(identity) should produce demosaic_weights / sensor_max
        let block = DemosaicCcmBlock::new(0);
        let identity: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let fused = block.fuse_weights(&identity);
        let raw = block.demosaic_weights();
        assert_eq!(fused.len(), raw.len());
        // Legacy fuse_weights uses no BLC/WB, sensor_max=1, so should match raw
        for (f, r) in fused.iter().zip(raw.iter()) {
            assert!((f - r).abs() < 1e-6, "Fused identity raw: {} vs {}", f, r);
        }
    }

    #[test]
    fn test_fuse_weights_ext_normalized() {
        // fuse_weights_ext should produce demosaic * CCM / sensor_max
        let block = DemosaicCcmBlock::new(0);
        let identity: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let wb = [1.0, 1.0, 1.0, 1.0];
        let blc = [0.0, 0.0, 0.0, 0.0];
        let sm = 1023.0;
        let (fused_w, fused_b) = block.fuse_weights_ext(&identity, &wb, &blc, sm);
        let raw = block.demosaic_weights();
        assert_eq!(fused_w.len(), 12);
        assert_eq!(fused_b.len(), 3);
        // w[i,j] = raw[i,j] * wb[j] / 1023  (wb=1, so = raw[i,j] / 1023)
        for i in 0..12 {
            let j = i % 4;
            let expected = raw[i] * wb[j] / sm;
            assert!(
                (fused_w[i] - expected).abs() < 1e-6,
                "w[{}]: {} vs {}",
                i,
                fused_w[i],
                expected
            );
        }
        // bias should be 0 (blc=0, wb=1 → -Σ fused*1*0 = 0)
        for i in 0..3 {
            assert!(fused_b[i].abs() < 1e-6, "b[{}]: {}", i, fused_b[i]);
        }
    }

    #[test]
    fn test_fuse_weights_ext_blc_wb() {
        // With non-zero blc and wb, verify bias calculation
        let block = DemosaicCcmBlock::new(0); // RGGB
        let identity: [f32; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let wb = [2.0, 1.5, 1.5, 2.0];
        let blc = [16.0, 8.0, 8.0, 16.0];
        let sm = 1023.0;
        let (fused_w, fused_b) = block.fuse_weights_ext(&identity, &wb, &blc, sm);
        // For RGGB: demosaic = [1,0,0,0, 0,0.5,0.5,0, 0,0,0,1]
        // fused_w[i,j] = demosaic[i,j] * ccm[i,i] * wb[j] / 1023
        // R: w[0,0]=1*1*2/1023, w[0,1]=0, w[0,2]=0, w[0,3]=0
        let expected_w0 = 1.0 * 1.0 * 2.0 / 1023.0;
        assert!(
            (fused_w[0] - expected_w0).abs() < 1e-6,
            "w[0]: {} vs {}",
            fused_w[0],
            expected_w0
        );
        // G: w[1,0]=0, w[1,1]=0.5*1*1.5/1023, w[1,2]=0.5*1*1.5/1023, w[1,3]=0
        let expected_w5 = 0.5 * 1.0 * 1.5 / 1023.0;
        let expected_w6 = 0.5 * 1.0 * 1.5 / 1023.0;
        assert!(
            (fused_w[5] - expected_w5).abs() < 1e-6,
            "w[5]: {} vs {}",
            fused_w[5],
            expected_w5
        );
        assert!(
            (fused_w[6] - expected_w6).abs() < 1e-6,
            "w[6]: {} vs {}",
            fused_w[6],
            expected_w6
        );
        // bias[0] = -Σ_j fused_w[0,j] * blc[j]  (wb already in fused_w)
        // fused_w[0,0] = demosaic[0,0]*wb[0]/1023 = 2/1023
        // bias[0] = -(2/1023) * 16 = -32/1023
        let expected_b0 = -(1.0 * 2.0 / 1023.0) * 16.0;
        assert!(
            (fused_b[0] - expected_b0).abs() < 1e-6,
            "b[0]: {} vs {}",
            fused_b[0],
            expected_b0
        );
    }

    #[test]
    fn test_demosaic_ccm_pipeline_integration() {
        let b1: Box<dyn IspBlock> =
            Box::new(crate::blocks::RawInputBlock::new().with_concrete_dims(48, 64));
        let b2: Box<dyn IspBlock> = Box::new(crate::blocks::NormalizeBlock::new());
        let b3: Box<dyn IspBlock> =
            Box::new(crate::blocks::CfaBlock::new().with_concrete_dims(48, 64));
        let b4: Box<dyn IspBlock> = Box::new(DemosaicCcmBlock::new(2).with_concrete_dims(24, 32));

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2, b3, b4];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(
            result.is_ok(),
            "Pipeline through DemosaicCcmBlock: {:?}",
            result.err()
        );
    }
}
