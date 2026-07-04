//! BayerProcBlock — Flexible debayer block supporting multiple modes. //! //! Replaces UnpackCfaBlock + DemosaicCcmBlock with a unified, mode-flexible block. //!
use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;
#[cfg(test)] use crate::blocks::RawInputBlock;

/// Processing mode for the Bayer pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BayerMode {
    /// Direct SpaceToDepth: INT16 → INT16 CFA (no demosaicing)
    /// Used for raw CFA forwarding.
    Int16Direct,
    
    /// Built-in demosaic: INT16 → FP32 RGB
    /// Automatically selects demosaic weights based on bayer_pattern.
    #[default]
    BuiltinDemosaic,
    
    /// Full processing: INT16 → RGB with fused BLC/WB/CCM
    /// Optional parameters: blc_vals, wb_gains, ccm_matrix
    FullProc,
}

/// Bayer pattern indexing for demosaic weights.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BayerPattern {
    /// RGGB: R=TL, G=avg(TR,BL), B=BR
    #[default]
    Rggb = 0,
    
    /// GRBG: R=TR, G=avg(TL,BR), B=BL
    Grbg = 1,
    
    /// GBRG: R=BL, G=avg(TL,BR), B=TR
    Gbrg = 2,
    
    /// BGGR: R=BR, G=avg(TR,BL), B=TL
    Bggr = 3,
}

/// Simplified debayer block supporting multiple modes.
pub struct BayerProcBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub mode: BayerMode,
    pub bayer_pattern: BayerPattern,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
    pub sensor_max: f32,
    pub use_blc: bool,
    pub use_wb: bool,
    // CCM matrix for FullProc mode
    pub ccm_matrix: [f32; 9],
}

impl Default for BayerProcBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl BayerProcBlock {
    pub fn new() -> Self {
        Self {
            id: "bayer_proc".into(),
            prev: None,
            next: None,
            frame_tensor: "BayerProcBlock/frame".into(),
            input_source: String::new(),
            mode: BayerMode::BuiltinDemosaic,
            bayer_pattern: BayerPattern::Rggb,
            concrete_h: None,
            concrete_w: None,
            sensor_max: 1023.0,
            use_blc: false,
            use_wb: false,
            ccm_matrix: [
                1.0, 0.0, 0.0, 
                0.0, 1.0, 0.0, 
                0.0, 0.0, 1.0,
            ],
        }
    }
    
    pub fn with_mode(mut self, mode: BayerMode) -> Self {
        self.mode = mode;
        self
    }
    
    pub fn with_bayer_pattern(mut self, pattern: BayerPattern) -> Self {
        self.bayer_pattern = pattern;
        self
    }
    
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }
    
    pub fn with_sensor_max(mut self, sm: f32) -> Self {
        self.sensor_max = sm;
        self
    }
    
    pub fn with_blc(mut self, enable: bool) -> Self {
        self.use_blc = enable;
        self
    }
    
    pub fn with_wb(mut self, enable: bool) -> Self {
        self.use_wb = enable;
        self
    }
    
    pub fn with_ccm(mut self, ccm_matrix: [f32; 9]) -> Self {
        self.ccm_matrix = ccm_matrix;
        self
    }
    
    /// Demosaic weights [3, 4, 1, 1] for the selected pattern.
    fn demosaic_weights(&self) -> Vec<f32> {
        match self.bayer_pattern {
            BayerPattern::Rggb => vec![
                1.0, 0.0, 0.0, 0.0, // R from TL
                0.0, 0.5, 0.5, 0.0, // G from TR/BL
                0.0, 0.0, 0.0, 1.0, // B from BR
            ],
            BayerPattern::Grbg => vec![
                0.0, 1.0, 0.0, 0.0, // R from TR
                0.5, 0.0, 0.0, 0.5, // G from TL/BR
                0.0, 0.0, 1.0, 0.0, // B from BL
            ],
            BayerPattern::Gbrg => vec![
                0.0, 0.0, 1.0, 0.0, // R from BL
                0.5, 0.0, 0.0, 0.5, // G from TL/BR
                0.0, 1.0, 0.0, 0.0, // B from TR
            ],
            BayerPattern::Bggr => vec![
                0.0, 0.0, 0.0, 1.0, // R from BR
                0.0, 0.5, 0.5, 0.0, // G from TR/BL
                1.0, 0.0, 0.0, 0.0, // B from TL
            ],
        }
    }
    
    #[cfg(test)]
    /// Fused weights for FullProc mode.
    fn full_weights(&self, wb_gains: [f32; 4], blc_vals: [f32; 4]) -> (Vec<f32>, Vec<f32>) {
        let mut fused_w = self.demosaic_weights();
        // Multiply each weight by the WB gain
        for idx in 0..fused_w.len() {
            fused_w[idx] *= wb_gains[idx % 4] / self.sensor_max;
        }
        // Adjust weights by CCM
        let mut result_w = vec![0.0; 12];
        for i in 0..3 {
            for j in 0..4 {
                for k in 0..3 {
                    result_w[i * 4 + j] += fused_w[k * 4 + j] * self.ccm_matrix[i * 3 + k];
                }
            }
        }
        // Compute bias based on BLC values
        let mut fused_b = vec![0.0; 3];
        for i in 0..3 {
            for j in 0..4 {
                fused_b[i] -= result_w[i * 4 + j] * blc_vals[j];
            }
        }
        (result_w, fused_b)
    }
}

impl IspBlock for BayerProcBlock {
    fn id(&self) -> &str {
        &self.id
    }
    
    fn tensor_ns(&self) -> String {
        "BayerProcBlock".to_string()
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
        let mut tensors = vec![self.input_source.clone()];
        if self.mode == BayerMode::FullProc {
            tensors.push(format!("{}/wb_gains", self.id));
            if self.use_blc {
                tensors.push(format!("{}/blc_vals", self.id));
            }
        }
        tensors
    }
    
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }
    
    fn input_elem_type(&self) -> i32 {
        match self.mode {
            BayerMode::Int16Direct => 5, // INT16
            _ => 5,                    // INT16 (even for BuiltinDemosaic - built-in Cast)
        }
    }
    
    fn output_elem_type(&self) -> i32 {
        match self.mode {
            BayerMode::Int16Direct => 5, // INT16
            _ => 1,                    // FLOAT
        }
    }
    
    fn input_value_info(&self) -> Option<Vec<u8>> {
        let (h, w) = match (self.concrete_h, self.concrete_w) {
            (Some(h), Some(w)) => (h, w),
            _ => (48, 64), // Default fallback
        };
        let dims = vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(h),
            Proto::tensor_dim_value(w),
        ];
        Some(Proto::value_info(&self.input_source, &dims, self.input_elem_type()))
    }
    
    fn output_value_info(&self) -> Option<Vec<u8>> {
        let (h, w) = match (self.concrete_h, self.concrete_w) {
            (Some(h), Some(w)) => (h / 2, w / 2),
            _ => (24, 32), // Default fallback
        };
        let ch = match self.mode {
            BayerMode::Int16Direct => 4,
            _ => 3,
        };
        let dims = vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(ch),
            Proto::tensor_dim_value(h),
            Proto::tensor_dim_value(w),
        ];
        Some(Proto::value_info(&self.frame_tensor, &dims, self.output_elem_type()))
    }
    
    fn nodes(&self) -> Vec<Vec<u8>> {
        match self.mode {
            BayerMode::Int16Direct => {
                // NativeInt16: single SpaceToDepth node
                vec![
                    Proto::node(
                        "SpaceToDepth",
                        &[&self.input_source],
                        &[&self.frame_tensor],
                        &[Proto::attribute_int("blocksize", 2)],
                    )
                ]
            }
            BayerMode::BuiltinDemosaic => {
                // Cast[INT16→FP32] → SpaceToDepth → Demosaic[Conv1x1] → Clip
                vec![
                    Proto::node(
                        "Cast",
                        &[&self.input_source],
                        &[&format!("{}/input_f32", self.id)],
                        &[Proto::attribute_int("to", 1)], // to FLOAT
                    ),
                    Proto::node(
                        "SpaceToDepth",
                        &[&format!("{}/input_f32", self.id)],
                        &[&format!("{}/unpacked", self.id)],
                        &[Proto::attribute_int("blocksize", 2)],
                    ),
                    Proto::node(
                        "Conv",
                        &[&format!("{}/unpacked", self.id), &format!("{}/demosaic_w", self.id), &format!("{}/demosaic_b", self.id)],
                        &[&self.frame_tensor],
                        &[
                            Proto::attribute_ints("kernel_shape", &[1, 1]),
                            Proto::attribute_ints("strides", &[1, 1]),
                            Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                            Proto::attribute_int("group", 1),
                        ]
                    ),
                    Proto::node(
                        "Clip",
                        &[&self.frame_tensor, &format!("{}/clip_lo", self.id), &format!("{}/clip_hi", self.id)],
                        &[&self.frame_tensor],
                        &[],
                    )
                ]
            }
            BayerMode::FullProc => {
                // Full mode: extra inputs handled via initializers
                vec![
                    Proto::node(
                        "SpaceToDepthEx",
                        &[&self.input_source, &format!("{}/wb_gains", self.id), &format!("{}/blc_vals", self.id)],
                        &[&self.frame_tensor],
                        &[
                            Proto::attribute_int("blocksize", 2),
                            Proto::attribute_int("mode", 2), // FullProc
                            Proto::attribute_int("bayer_pattern", self.bayer_pattern as i64),
                        ]
                    )
                ]
            }
        }
    }
    
    fn initializers(&self) -> Vec<Vec<u8>> {
        let mut inits = vec![];
        
        // Max value for norm/denorm
        inits.push(Proto::tensor_proto_float_scalar(
            &format!("{}/sensor_max", self.id),
            self.sensor_max
        ));
        
        match self.mode {
            BayerMode::Int16Direct => {
                // No additional initializers for direct mode
            }
            BayerMode::BuiltinDemosaic => {
                // Demosaic weights [3, 4, 1, 1]
                inits.push(Proto::tensor_proto_float(
                    &format!("{}/demosaic_w", self.id),
                    &[3, 4, 1, 1],
                    &self.demosaic_weights()
                ));
                inits.push(Proto::tensor_proto_float_scalar(
                    &format!("{}/demosaic_b", self.id),
                    0.0
                ));
                // Clip range [0, 1]
                inits.push(Proto::tensor_proto_float_scalar(
                    &format!("{}/clip_lo", self.id),
                    0.0
                ));
                inits.push(Proto::tensor_proto_float_scalar(
                    &format!("{}/clip_hi", self.id),
                    1.0
                ));
            }
            BayerMode::FullProc => {
                // WB gains [1, 4, 1, 1] (defaults to all 1.0)
                inits.push(Proto::tensor_proto_float(
                    &format!("{}/wb_gains", self.id),
                    &[1, 4, 1, 1],
                    &[1.0, 1.0, 1.0, 1.0]
                ));
                
                // BLC values [1, 4, 1, 1] (defaults to [0, 0, 0, 0])
                inits.push(Proto::tensor_proto_float(
                    &format!("{}/blc_vals", self.id),
                    &[1, 4, 1, 1],
                    &[0.0, 0.0, 0.0, 0.0]
                ));
            }
        }
        inits
    }
    
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let mut extras = vec![
            (format!("{}/sensor_max", self.id), 1, vec![])
        ];
        
        match self.mode {
            BayerMode::Int16Direct => {
                // No extra inputs
            }
            BayerMode::BuiltinDemosaic => {
                // No runtime-modified extra inputs needed
            }
            BayerMode::FullProc => {
                // Let controller set:
                // - wb_gains shape [1, 4, 1, 1], type FLOAT
                // - blc_vals shape [1, 4, 1, 1], type FLOAT (optional)
                extras.push((format!("{}/wb_gains", self.id), 1, vec![1, 4, 1, 1]));
                if self.use_blc {
                    extras.push((format!("{}/blc_vals", self.id), 1, vec![1, 4, 1, 1]));
                }
            }
        }
        extras
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::GraphComposer;

    #[test]
    fn test_bayerproc_modes() {
        let patterns = [
            BayerPattern::Rggb,
            BayerPattern::Grbg,
            BayerPattern::Gbrg,
            BayerPattern::Bggr,
        ];
        
        for &pattern in &patterns {
            // Test BuiltinDemosaic mode
            let block = BayerProcBlock::new()
                .with_bayer_pattern(pattern)
                .with_concrete_dims(48, 64);
                assert_eq!(block.nodes().len(), 4, "BuiltinDemosaic: expected 4 nodes");
                assert_eq!(block.output_elem_type(), 1, "BuiltinDemosaic: F32 output");
                
            // Test Direct mode
            let direct = BayerProcBlock::new()
                .with_mode(BayerMode::Int16Direct)
                .with_bayer_pattern(pattern)
                .with_concrete_dims(48, 64);
                assert_eq!(direct.nodes().len(), 1, "Direct: expected 1 node");
                assert_eq!(direct.output_elem_type(), 5, "Direct: INT16 output");
                
            // Test Full mode
            let full = BayerProcBlock::new()
                .with_mode(BayerMode::FullProc)
                .with_bayer_pattern(pattern)
                .with_concrete_dims(48, 64);
                assert_eq!(full.nodes().len(), 1, "FullProc: expected 1 node");
                assert_eq!(full.output_elem_type(), 1, "FullProc: FLOAT output");
        }
    }
    
    #[test]
    fn test_bayer_pattern_weights() {
        let block_rggb = BayerProcBlock::new().with_bayer_pattern(BayerPattern::Rggb);
        let weights = block_rggb.demosaic_weights();
        assert_eq!(weights[0], 1.0, "RGGb: R weight");
        assert_eq!(weights[5], 0.5, "RGGb: G avg");
        
        let block_bayer_ids = [
            (BayerPattern::Grbg, 1),
            (BayerPattern::Gbrg, 2),
            (BayerPattern::Bggr, 3),
        ];
        
        for &(pattern, r_pos) in &block_bayer_ids {
            let block = BayerProcBlock::new().with_bayer_pattern(pattern);
            let weights = block.demosaic_weights();
            assert_eq!(weights[r_pos], 1.0, "Pattern {:?}: R weight", pattern);
        }
    }
    
    #[test]
    fn test_pipeline_integration() {
        let _b1: Box<dyn IspBlock> = Box::new(crate::blocks::RawInputBlock::new()
            .with_elem_type(5) // INT16
            .with_concrete_dims(48, 64));
        
        // Test each mode
        for &mode in &[
            BayerMode::Int16Direct,
            BayerMode::BuiltinDemosaic,
            BayerMode::FullProc,
        ] {
            let _b2: Box<dyn IspBlock> = Box::new(BayerProcBlock::new()
                .with_mode(mode)
                .with_concrete_dims(48, 64));
            
            let mut blocks: Vec<Box<dyn IspBlock>> = vec![];
            blocks.push(Box::new(RawInputBlock::new()
                .with_elem_type(5)
                .with_concrete_dims(48, 64)));
            blocks.push(Box::new(BayerProcBlock::new()
                .with_mode(mode)
                .with_concrete_dims(48, 64)));
            GraphComposer::wire_blocks(&mut blocks);
            
            let result = GraphComposer::compose_from_vec(
                &blocks.iter().map(|b| b.as_ref()).collect::<Vec<_>>(),
                &[],
                5 // CPU backend
            );
            
            assert!(result.is_ok(), "Mode {:?} should compose: {:?}", mode, result.err());
        }
    }
    
    #[test]
    fn test_weight_fusion() {
        let mut block = BayerProcBlock::new()
            .with_mode(BayerMode::FullProc)
            .with_bayer_pattern(BayerPattern::Rggb)
            .with_blc(true)
            .with_wb(true)
            .with_sensor_max(1023.0);
        
        let wb_gains = [1.2, 1.0, 1.0, 1.1];
        let blc_vals = [0.1 / 1023.0, 0.1 / 1023.0, 0.05 / 1023.0, 0.0 / 1023.0];
        // Note: wb_gains is [gR, gGr, gGb, gB] — scale R/B + keep G
        
        let ccm = [
            1.3, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.1,
        ];
        
        block.ccm_matrix = ccm;
        let (w, b) = block.full_weights(wb_gains, blc_vals);
        
        assert_eq!(w.len(), 12, "Fused weights shape [3,4,1,1] = 12");
        assert_eq!(b.len(), 3, "Fused bias [3]");
        
        // Check first RGB entry: R = scale × ccm
        let expected_r = 1.0 * wb_gains[0] / 1023.0 * 1.3;
        assert!((expected_r - w[0]).abs() < 0.0001, "R weight = {:.5} ≈ {:.5}", w[0], expected_r);
        // Manual check to verify the CCM × WB fusion
        let expected_b_bias = -(w[0] * blc_vals[0] + w[1] * blc_vals[1] + w[2] * blc_vals[2] + w[3] * blc_vals[3]);
        assert!((b[2] - expected_b_bias).abs() < 0.0001, "B location bias: {:.5} ≈ {:.5}", b[2], expected_b_bias);
    }
}