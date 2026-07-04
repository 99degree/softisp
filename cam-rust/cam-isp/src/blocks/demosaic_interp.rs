use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Demosaic interpolation block — full-resolution bilinear demosaicing.
///
/// Generates a Conv(4×4, stride=1, 1ch→3ch) pattern that IspChainFusion
/// converts to `isp.demosaic_interp` (bilinear SPIR-V shader).
///
/// Input:  `[1,1,H,W]` INT32 Bayer
/// Output: `[1,3,H,W]` F32 RGB at full resolution (no downscale)
///
/// For non-binned sensors where binning would lose too much resolution.
/// Only one of UnpackCfaBlock or DemosaicInterpBlock should be used.
pub struct DemosaicInterpBlock {
    id: String,
    prev: Option<Box<dyn IspBlock>>,
    next: Option<Box<dyn IspBlock>>,
    input_source: String,
    output_name: String,
    /// Sensor max value for normalization (default 1023 for 10-bit)
    sensor_max: f32,
}

impl Default for DemosaicInterpBlock {
    fn default() -> Self {
        Self {
            id: "DemosaicInterpBlock".to_string(),
            prev: None,
            next: None,
            input_source: String::new(),
            output_name: "DemosaicInterpBlock/frame".to_string(),
            sensor_max: 1023.0,
        }
    }
}

impl DemosaicInterpBlock {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_sensor_max(mut self, max: f32) -> Self {
        self.sensor_max = max;
        self
    }
}

impl IspBlock for DemosaicInterpBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "DemosaicInterpBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.output_name) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    fn input_tensors(&self) -> Vec<String> {
        if self.input_source.is_empty() { vec![] } else { vec![self.input_source.clone()] }
    }

    fn output_tensors(&self) -> Vec<String> { vec![self.output_name.clone()] }

    fn graph_input_name(&self) -> Option<&str> {
        if self.is_head() { Some(&self.output_name) } else { None }
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.output_name, &[
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(3),
            Proto::tensor_dim_param("H"),
            Proto::tensor_dim_param("W"),
        ], 1)) // FLOAT
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        if self.input_source.is_empty() {
            return vec![];
        }

        let ns = self.tensor_ns();

        // Conv(4×4, stride=1, pad=0, 1ch→3ch, group=1)
        // IspChainFusion R2b recognizes this pattern → isp.demosaic_interp
        vec![Proto::node(
            "Conv",
            &[
                &self.input_source,
                &format!("{ns}/conv_w"),
                &format!("{ns}/conv_b"),
            ],
            &[&self.output_name],
            &[
                Proto::attribute_ints("kernel_shape", &[4, 4]),
                Proto::attribute_ints("strides", &[1, 1]),
                Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                Proto::attribute_int("group", 1),
            ],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();

        // Conv weights: [3, 1, 4, 4] = 48 floats
        // Bilinear interpolation weights for RGGB Bayer pattern
        let mut weights = vec![0.0f32; 3 * 4 * 4]; // [OC, IC, KY, KX]

        // Channel 0 (R): weight R positions heavily, interpolate G and B
        for ky in 0..4 {
            for kx in 0..4 {
                let bx = kx % 2;
                let by = ky % 2;
                let idx = ky * 4 + kx;

                if bx == 0 && by == 0 {
                    // R position: weight 0.25
                    weights[idx] = 0.25;
                } else if (bx == 1 && by == 0) || (bx == 0 && by == 1) {
                    // Gr or Gb position: weight 0.125
                    weights[idx] = 0.125;
                } else {
                    // B position: weight 0.0625
                    weights[idx] = 0.0625;
                }
            }
        }

        // Channel 1 (G): weight Gr/Gb positions heavily
        for ky in 0..4 {
            for kx in 0..4 {
                let bx = kx % 2;
                let by = ky % 2;
                let idx = 16 + ky * 4 + kx;

                if (bx == 1 && by == 0) || (bx == 0 && by == 1) {
                    // G position: weight 0.25
                    weights[idx] = 0.25;
                } else {
                    // R or B position: weight 0.125
                    weights[idx] = 0.125;
                }
            }
        }

        // Channel 2 (B): weight B positions heavily, interpolate R and G
        for ky in 0..4 {
            for kx in 0..4 {
                let bx = kx % 2;
                let by = ky % 2;
                let idx = 2 * 16 + ky * 4 + kx;

                if bx == 1 && by == 1 {
                    // B position: weight 0.25
                    weights[idx] = 0.25;
                } else if (bx == 1 && by == 0) || (bx == 0 && by == 1) {
                    // G position: weight 0.125
                    weights[idx] = 0.125;
                } else {
                    // R position: weight 0.0625
                    weights[idx] = 0.0625;
                }
            }
        }

        // Bias: [3] zeros
        let bias = vec![0.0f32; 3];

        vec![
            Proto::tensor_proto_float(&format!("{ns}/conv_w"), &[3, 1, 4, 4], &weights),
            Proto::tensor_proto_float(&format!("{ns}/conv_b"), &[3], &bias),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_demosaic_interp_nodes() {
        let mut block = DemosaicInterpBlock::new();
        block.set_input_source("RawInputBlock/frame");
        let nodes = block.nodes();
        assert_eq!(nodes.len(), 1, "Single Conv node");
    }

    #[test]
    fn test_demosaic_interp_initializers() {
        let block = DemosaicInterpBlock::new();
        let inits = block.initializers();
        assert_eq!(inits.len(), 2, "Conv weights + bias");
    }

    #[test]
    fn test_demosaic_interp_id() {
        let block = DemosaicInterpBlock::new();
        assert_eq!(block.id(), "DemosaicInterpBlock");
    }

    #[test]
    fn test_demosaic_interp_tensor_ns() {
        let block = DemosaicInterpBlock::new();
        assert!(!block.tensor_ns().is_empty());
    }

    #[test]
    fn test_demosaic_interp_with_sensor_max() {
        let block = DemosaicInterpBlock::new().with_sensor_max(4095.0);
        let inits = block.initializers();
        assert_eq!(inits.len(), 2, "Conv weights + bias");
    }
}
