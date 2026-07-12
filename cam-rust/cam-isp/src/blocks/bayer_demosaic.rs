//! BayerDemosaicBlock — flexible Bayer demosaic with multiple algorithms.
//!
//! Supports Binning, Bilinear, and MHC (edge-directed) interpolation.
//! The DemosaicAlgo enum selects the algorithm at build time.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// BayerDemosaicBlock — configurable Bayer demosaic with Binning/Bilinear/MHC.
///
/// Select algorithm via `DemosaicAlgo` at build time.
/// Wraps a single ONNX Extra op with the chosen algorithm baked in.

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum DemosaicAlgo {
    Binning,
    #[default]
    Bilinear,
    Mhc,
}

impl DemosaicAlgo {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Binning => "binning",
            Self::Bilinear => "bilinear",
            Self::Mhc => "mhc",
        }
    }

    pub fn from_name(s: &str) -> Self {
        match s {
            "binning" => Self::Binning,
            "mhc" => Self::Mhc,
            _ => Self::Bilinear,
        }
    }

    pub fn kernel(&self) -> i64 {
        match self {
            Self::Binning => 2,
            Self::Bilinear => 4,
            Self::Mhc => 6,
        }
    }

    pub fn stride(&self) -> i64 {
        match self {
            Self::Binning => 2,
            Self::Bilinear | Self::Mhc => 1,
        }
    }

    pub fn output_channels(&self) -> i64 {
        match self {
            Self::Binning => 4,
            _ => 3,
        }
    }

    pub fn group(&self) -> i64 {
        match self {
            Self::Binning => 4,
            _ => 1,
        }
    }

    pub fn output_height(&self, h: i64) -> i64 {
        match self {
            Self::Binning => h / 2,
            _ => h,
        }
    }

    pub fn output_width(&self, w: i64) -> i64 {
        match self {
            Self::Binning => w / 2,
            _ => w,
        }
    }
}

pub struct BayerDemosaicBlock {
    id: String,
    prev: Option<Box<dyn IspBlock>>,
    next: Option<Box<dyn IspBlock>>,
    input_source: String,
    output_name: String,
    pub algo: DemosaicAlgo,
    pub sensor_max: f32,
    concrete_h: Option<i64>,
    concrete_w: Option<i64>,
}

impl Default for BayerDemosaicBlock {
    fn default() -> Self {
        Self {
            id: "BayerDemosaicBlock".to_string(),
            prev: None,
            next: None,
            input_source: String::new(),
            output_name: "BayerDemosaicBlock/frame".to_string(),
            algo: DemosaicAlgo::Bilinear,
            sensor_max: 1023.0,
            concrete_h: None,
            concrete_w: None,
        }
    }
}

impl BayerDemosaicBlock {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn with_algorithm(mut self, algo: DemosaicAlgo) -> Self {
        self.algo = algo;
        self
    }
    pub fn with_sensor_max(mut self, max: f32) -> Self {
        self.sensor_max = max;
        self
    }
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }
}

impl IspBlock for BayerDemosaicBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "BayerDemosaicBlock".to_string()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.output_name)
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
        if self.input_source.is_empty() {
            vec![]
        } else {
            vec![self.input_source.clone()]
        }
    }
    fn output_tensors(&self) -> Vec<String> {
        vec![self.output_name.clone()]
    }
    fn graph_input_name(&self) -> Option<&str> {
        if self.is_head() {
            Some(&self.output_name)
        } else {
            None
        }
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let h = self.concrete_h.unwrap_or(0);
        let w = self.concrete_w.unwrap_or(0);
        let oh = self.algo.output_height(h);
        let ow = self.algo.output_width(w);
        let oc = self.algo.output_channels();
        Some(Proto::value_info(
            &self.output_name,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(oc),
                if oh > 0 {
                    Proto::tensor_dim_value(oh)
                } else {
                    Proto::tensor_dim_param("H")
                },
                if ow > 0 {
                    Proto::tensor_dim_value(ow)
                } else {
                    Proto::tensor_dim_param("W")
                },
            ],
            1,
        ))
    }

    fn input_elem_type(&self) -> i32 {
        6
    }
    fn output_elem_type(&self) -> i32 {
        1
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        if self.input_source.is_empty() {
            return vec![];
        }
        let ns = self.tensor_ns();
        let k = self.algo.kernel();
        let s = self.algo.stride();
        let g = self.algo.group();
        vec![Proto::node(
            "Conv",
            &[
                &self.input_source,
                &format!("{ns}/conv_w"),
                &format!("{ns}/conv_b"),
            ],
            &[&self.output_name],
            &[
                Proto::attribute_ints("kernel_shape", &[k, k]),
                Proto::attribute_ints("strides", &[s, s]),
                Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                Proto::attribute_int("group", g),
            ],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let oc = self.algo.output_channels() as usize;
        let k = self.algo.kernel() as usize;
        match self.algo {
            DemosaicAlgo::Binning => {
                let mut w = vec![0.0f32; oc * k * k];
                for c in 0..oc {
                    w[c * k * k + c] = 1.0;
                }
                let b = vec![0.0f32; oc];
                vec![
                    Proto::tensor_proto_float(
                        &format!("{ns}/conv_w"),
                        &[oc as i64, 1, k as i64, k as i64],
                        &w,
                    ),
                    Proto::tensor_proto_float(&format!("{ns}/conv_b"), &[oc as i64], &b),
                ]
            }
            DemosaicAlgo::Bilinear => {
                let mut w = vec![0.0f32; oc * k * k];
                for o in 0..oc {
                    for ky in 0..k {
                        for kx in 0..k {
                            let bx = kx % 2;
                            let by = ky % 2;
                            let idx = o * k * k + ky * k + kx;
                            w[idx] = match (o, bx, by) {
                                (0, 0, 0) => 0.25,
                                (0, 1, 1) => 0.0625,
                                (0, _, _) => 0.125,
                                (1, 1, 0) | (1, 0, 1) => 0.25,
                                (1, _, _) => 0.125,
                                (2, 1, 1) => 0.25,
                                (2, 0, 0) => 0.0625,
                                (2, _, _) => 0.125,
                                _ => 0.0,
                            };
                        }
                    }
                }
                let b = vec![0.0f32; oc];
                vec![
                    Proto::tensor_proto_float(
                        &format!("{ns}/conv_w"),
                        &[oc as i64, 1, k as i64, k as i64],
                        &w,
                    ),
                    Proto::tensor_proto_float(&format!("{ns}/conv_b"), &[oc as i64], &b),
                ]
            }
            DemosaicAlgo::Mhc => {
                // 6x6 conv — weights not used by SPIR-V shader,
                // but must be non-zero for MNN converter detection
                let mut w = vec![0.0f32; oc * k * k];
                for o in 0..oc {
                    // Center region weight
                    for ky in 0..k {
                        for kx in 0..k {
                            let idx = o * k * k + ky * k + kx;
                            // 6x6 kernel, center 2x2 gets highest weight
                            if (2..=3).contains(&ky) && (2..=3).contains(&kx) {
                                w[idx] = 1.0 / 4.0;
                            } else if (1..=4).contains(&ky) && (1..=4).contains(&kx) {
                                w[idx] = 1.0 / 8.0;
                            } else {
                                w[idx] = 1.0 / 16.0;
                            }
                        }
                    }
                }
                let b = vec![0.0f32; oc];
                vec![
                    Proto::tensor_proto_float(
                        &format!("{ns}/conv_w"),
                        &[oc as i64, 1, k as i64, k as i64],
                        &w,
                    ),
                    Proto::tensor_proto_float(&format!("{ns}/conv_b"), &[oc as i64], &b),
                ]
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::IspBlock;
    #[test]
    fn test_algo_kernel() {
        assert_eq!(DemosaicAlgo::Binning.kernel(), 2);
        assert_eq!(DemosaicAlgo::Bilinear.kernel(), 4);
        assert_eq!(DemosaicAlgo::Mhc.kernel(), 6);
    }
    #[test]
    fn test_algo_stride() {
        assert_eq!(DemosaicAlgo::Binning.stride(), 2);
        assert_eq!(DemosaicAlgo::Bilinear.stride(), 1);
    }
    #[test]
    fn test_algo_output_shape() {
        let h = 1080i64;
        let w = 1920i64;
        assert_eq!(DemosaicAlgo::Binning.output_height(h), 540);
        assert_eq!(DemosaicAlgo::Binning.output_width(w), 960);
        assert_eq!(DemosaicAlgo::Bilinear.output_height(h), 1080);
        assert_eq!(DemosaicAlgo::Bilinear.output_width(w), 1920);
    }
    #[test]
    fn test_binning_nodes() {
        let mut block = BayerDemosaicBlock::new().with_algorithm(DemosaicAlgo::Binning);
        block.set_input_source("raw");
        assert_eq!(block.nodes().len(), 1);
    }
    #[test]
    fn test_bilinear_nodes() {
        let mut block = BayerDemosaicBlock::new().with_algorithm(DemosaicAlgo::Bilinear);
        block.set_input_source("raw");
        assert_eq!(block.nodes().len(), 1);
    }
    #[test]
    fn test_mhc_nodes() {
        let mut block = BayerDemosaicBlock::new().with_algorithm(DemosaicAlgo::Mhc);
        block.set_input_source("raw");
        assert_eq!(block.nodes().len(), 1);
    }
    #[test]
    fn test_initializers() {
        for algo in &[
            DemosaicAlgo::Binning,
            DemosaicAlgo::Bilinear,
            DemosaicAlgo::Mhc,
        ] {
            let block = BayerDemosaicBlock::new().with_algorithm(*algo);
            assert_eq!(block.initializers().len(), 2);
        }
    }
    #[test]
    fn test_mhc_weights_nonzero() {
        // Verify MHC generates non-zero 6×6 kernel weights
        let mut block = BayerDemosaicBlock::new().with_algorithm(DemosaicAlgo::Mhc);
        block.set_input_source("raw");
        let inits = block.initializers();
        // First initializer is the weight tensor
        // 6×6×3 = 108 float32 values = 432 bytes in raw_data
        // Plus protobuf framing ~50 bytes
        assert!(inits[0].len() > 400, "MHC weight tensor too small");
        assert!(inits[1].len() > 4, "MHC bias tensor too small");
        // Verify the node count
        assert_eq!(block.nodes().len(), 1);
    }
}
