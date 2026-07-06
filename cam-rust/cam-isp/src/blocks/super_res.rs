//! SuperResBlock — multi-frame super-resolution fusion.
//!
//! Merges N low-resolution frames into a single higher-quality frame.
//! Similar to HdrMergeBlock but for spatial super-resolution rather than
//! HDR exposure fusion. Uses contrast-weighted averaging: frames with
//! sharper detail (higher local contrast) get higher weights.
//!
//! Algorithm:
//!   1. Compute local contrast per frame: |Laplacian(frame)| or |frame - mean|
//!   2. Softmax-like weighting: weight_i = contrast_i / sum(contrasts)
//!   3. Merge: output = sum(frame_i * weight_i)
//!
//! ONNX subgraph (2-frame variant):
//!   1. diff0 = |frame0 - AvgPool(frame0)|
//!   2. diff1 = |frame1 - AvgPool(frame1)|
//!   3. w0 = diff0 / (diff0 + diff1 + eps)
//!   4. w1 = 1 - w0
//!   5. output = frame0 * w0 + frame1 * w1
//!
//! The `num_frames` parameter controls how many frames are fused (2-8).

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// SuperResBlock — multi-frame super-resolution via contrast-weighted fusion.
pub struct SuperResBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    /// Primary input (first frame).
    pub primary_source: String,
    /// Additional frame inputs.
    pub frame_sources: Vec<String>,
    /// Number of frames to fuse (2-8). Default: 2.
    pub num_frames: usize,
}

impl Default for SuperResBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl SuperResBlock {
    pub fn new() -> Self {
        Self {
            id: "super_res".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "SuperResBlock/frame".into(),
            primary_source: String::new(),
            frame_sources: Vec::new(),
            num_frames: 2,
        }
    }

    pub fn with_num_frames(mut self, n: usize) -> Self {
        self.num_frames = n.clamp(2, 8);
        self
    }

    pub fn add_frame(mut self, name: &str) -> Self {
        self.frame_sources.push(name.to_string());
        self
    }
}

impl IspBlock for SuperResBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "SuperRes".into() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.primary_source) }
    fn set_input_source(&mut self, name: &str) { self.primary_source = name.into(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev_block.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev_block = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next_block.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next_block = Some(block); }

    fn input_tensors(&self) -> Vec<String> {
        let mut inputs = vec![self.primary_source.clone()];
        inputs.extend(self.frame_sources.iter().cloned());
        inputs
    }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn graph_output_name(&self) -> Option<&str> { Some(&self.frame_tensor) }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.primary_source,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }
    fn output_value_info(&self) -> Option<Vec<u8>> { self.input_value_info() }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // For 2 frames: contrast-weighted fusion
        // For N frames: sequential pairwise fusion (frame0 ⊕ frame1, then result ⊕ frame2, ...)
        //
        // Pairwise fusion:
        //   diff_a = |a - AvgPool(a)|  (local contrast of frame a)
        //   diff_b = |b - AvgPool(b)|
        //   w_a = diff_a / (diff_a + diff_b + eps)
        //   w_b = 1 - w_a
        //   merged = a * w_a + b * w_b

        let eps_name = format!("{}/eps", ns);
        let mut current = self.primary_source.clone();

        // If only primary_source (no additional frames), return identity
        if self.frame_sources.is_empty() {
            nodes.push(Proto::node("Identity", &[&current], &[&self.frame_tensor], &[]));
            return nodes;
        }

        // Sequential pairwise fusion
        for (i, extra_frame) in self.frame_sources.iter().enumerate() {
            let pair_ns = format!("{}/p{}", ns, i);

            // Contrast of current: |current - AvgPool(current)|
            let mean_a = format!("{}/mean_a", pair_ns);
            let diff_a_raw = format!("{}/diff_a_raw", pair_ns);
            let diff_a = format!("{}/diff_a", pair_ns);

            nodes.push(Proto::node("AveragePool",
                &[&current],
                &[&mean_a],
                &[Proto::attribute_ints("kernel_shape", &[3, 3]),
                  Proto::attribute_ints("pads", &[1, 1, 1, 1]),
                  Proto::attribute_ints("strides", &[1, 1])]));
            nodes.push(Proto::node("Sub", &[&current, &mean_a], &[&diff_a_raw], &[]));
            nodes.push(Proto::node("Abs", &[&diff_a_raw], &[&diff_a], &[]));

            // Contrast of extra: |extra - AvgPool(extra)|
            let mean_b = format!("{}/mean_b", pair_ns);
            let diff_b_raw = format!("{}/diff_b_raw", pair_ns);
            let diff_b = format!("{}/diff_b", pair_ns);

            nodes.push(Proto::node("AveragePool",
                &[extra_frame],
                &[&mean_b],
                &[Proto::attribute_ints("kernel_shape", &[3, 3]),
                  Proto::attribute_ints("pads", &[1, 1, 1, 1]),
                  Proto::attribute_ints("strides", &[1, 1])]));
            nodes.push(Proto::node("Sub", &[extra_frame, &mean_b], &[&diff_b_raw], &[]));
            nodes.push(Proto::node("Abs", &[&diff_b_raw], &[&diff_b], &[]));

            // w_a = diff_a / (diff_a + diff_b + eps)
            let sum_diff = format!("{}/sum_diff", pair_ns);
            let sum_eps = format!("{}/sum_eps", pair_ns);
            let w_a = format!("{}/w_a", pair_ns);

            nodes.push(Proto::node("Add", &[&diff_a, &diff_b], &[&sum_diff], &[]));
            nodes.push(Proto::node("Add", &[&sum_diff, &eps_name], &[&sum_eps], &[]));
            nodes.push(Proto::node("Div", &[&diff_a, &sum_eps], &[&w_a], &[]));

            // w_b = 1 - w_a
            let one = format!("{}/one", pair_ns);
            let w_b = format!("{}/w_b", pair_ns);
            nodes.push(Proto::node("Sub", &[&one, &w_a], &[&w_b], &[]));

            // merged = current * w_a + extra * w_b
            let ca = format!("{}/ca", pair_ns);
            let cb = format!("{}/cb", pair_ns);
            let merged = format!("{}/merged", pair_ns);

            nodes.push(Proto::node("Mul", &[&current, &w_a], &[&ca], &[]));
            nodes.push(Proto::node("Mul", &[extra_frame, &w_b], &[&cb], &[]));
            nodes.push(Proto::node("Add", &[&ca, &cb], &[&merged], &[]));

            current = merged;
        }

        // Rename final to output
        if current != self.frame_tensor {
            nodes.push(Proto::node("Identity", &[&current], &[&self.frame_tensor], &[]));
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float_scalar(&format!("{}/eps", ns), 1e-6),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        self.frame_sources.iter().map(|s| {
            (s.clone(), 1, vec![1, 3, -1, -1])
        }).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_super_res_id() {
        assert_eq!(SuperResBlock::new().id(), "super_res");
    }

    #[test]
    fn test_super_res_with_frames() {
        let b = SuperResBlock::new().with_num_frames(4);
        assert_eq!(b.num_frames, 4);
    }

    #[test]
    fn test_super_res_clamps_frames() {
        let b = SuperResBlock::new().with_num_frames(20);
        assert_eq!(b.num_frames, 8, "should clamp to max 8");
    }

    #[test]
    fn test_super_res_two_frame_emit() {
        let mut b = SuperResBlock::new().with_num_frames(2);
        b.set_input_source("frame0");
        b = b.add_frame("frame1");
        let nodes = b.nodes();
        // Per pair: AvgPool + Sub + Abs + AvgPool + Sub + Abs + Add + Add + Div + Sub + Mul + Mul + Add = 13
        assert!(nodes.len() >= 10, "need >= 10 nodes for 2-frame, got {}", nodes.len());
    }

    #[test]
    fn test_super_res_three_frame_emit() {
        let mut b = SuperResBlock::new().with_num_frames(3);
        b.set_input_source("frame0");
        b = b.add_frame("frame1");
        b = b.add_frame("frame2");
        let nodes = b.nodes();
        // 2 pairs × 13 nodes + 1 Identity = 27
        assert!(nodes.len() >= 20, "need >= 20 nodes for 3-frame, got {}", nodes.len());
    }

    #[test]
    fn test_super_res_no_extra_frames() {
        let mut b = SuperResBlock::new();
        b.set_input_source("frame0");
        let nodes = b.nodes();
        assert_eq!(nodes.len(), 1, "no extra frames should be Identity");
    }

    #[test]
    fn test_super_res_has_input_output() {
        let b = SuperResBlock::new();
        assert_eq!(b.output_tensors().len(), 1);
    }

    #[test]
    fn test_super_res_tensor_ns() {
        let b = SuperResBlock::new();
        assert_eq!(b.tensor_ns(), "SuperRes");
    }

    #[test]
    fn test_super_res_extra_inputs() {
        let mut b = SuperResBlock::new().with_num_frames(3);
        b.set_input_source("frame0");
        b = b.add_frame("frame1");
        b = b.add_frame("frame2");
        let extras = b.extra_inputs();
        assert_eq!(extras.len(), 2, "should have 2 extra frame inputs");
    }

    #[test]
    fn test_super_res_graph_output() {
        let b = SuperResBlock::new();
        assert!(b.graph_output_name().is_some());
    }
}
