//! HdrMergeBlock — multi-exposure HDR merge.
//!
//! Takes 3 exposures (under/neutral/over) and blends them using
//! luminance-based weight maps. Output is a single HDR frame.
//!
//! ONNX subgraph:
//!   1. Luminance = 0.299*R + 0.587*G + 0.114*B (Conv group=3, 3→3)
//!   2. ReduceSum → [1,1,H,W], clip to [0,1]
//!   3. Weight over   = clip(lum * 2, 0, 1)      → dark areas low weight
//!   4. Weight under  = 1 - w_over                 → bright areas low weight
//!   5. Weight neutral = 1 - |lum - 0.5| * 2      → mid-range high weight
//!   6. Broadcast 1ch→3ch via Conv(1×1, 1→3)
//!   7. Merged = under*w_under + neutral*w_neutral + over*w_over
//!
//! Inputs (graph inputs, not initializers):
//!   - neutral_exposure: [1,3,H,W] float — main pipeline input
//!   - under_exposure:   [1,3,H,W] float — short exposure
//!   - over_exposure:    [1,3,H,W] float — long exposure

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

/// HdrMergeBlock — multi-exposure HDR fusion.
///
/// Merges multiple exposures using exposure-weighted blending.
/// Inputs: [overexposed, normal, underexposed] as separate channels.
pub struct HdrMergeBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub neutral_source: String,
    pub under_source: String,
    pub over_source: String,
    pub in_h: Option<i64>,
    pub in_w: Option<i64>,
}

impl HdrMergeBlock {
    pub fn new() -> Self {
        Self {
            id: "hdr_merge".into(),
            prev: None,
            next: None,
            frame_tensor: "HdrMergeBlock/frame".into(),
            neutral_source: String::new(),
            under_source: "HdrMergeBlock/under".into(),
            over_source: "HdrMergeBlock/over".into(),
            in_h: None,
            in_w: None,
        }
    }

    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.in_h = Some(h);
        self.in_w = Some(w);
        self
    }

    fn chw_shape(&self) -> Vec<Vec<u8>> {
        match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(h), Proto::tensor_dim_value(w)],
            _ => vec![
                Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
                Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")],
        }
    }
}

impl IspBlock for HdrMergeBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "HdrMergeBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.neutral_source) }
    fn set_input_source(&mut self, name: &str) { self.neutral_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.neutral_source.clone(), self.under_source.clone(), self.over_source.clone()]
    }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.neutral_source, &self.chw_shape(), 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(&self.frame_tensor, &self.chw_shape(), 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // Luminance from neutral: Conv group=3 [0.299, 0.587, 0.114]
        let lum_3ch = format!("{}/lum_3ch", ns);
        let lum_1ch = format!("{}/lum_1ch", ns);
        let lum_clip_min = format!("{}/lum_clip_min", ns);
        let lum_clip_max = format!("{}/lum_clip_max", ns);

        nodes.push(Proto::node("Conv",
            &[&self.neutral_source, &format!("{}/lum_w", ns), &format!("{}/lum_b", ns)],
            &[&lum_3ch],
            &[Proto::attribute_ints("kernel_shape", &[1, 1]),
              Proto::attribute_int("group", 3)]));
        nodes.push(Proto::node("ReduceSum", &[&lum_3ch], &[&lum_1ch],
            &[Proto::attribute_ints("axes", &[1]), Proto::attribute_int("keepdims", 1)]));
        nodes.push(Proto::node("Clip", &[&lum_1ch, &lum_clip_min, &lum_clip_max],
            &[&lum_1ch], &[]));

        // w_over = clip(lum * 2, 0, 1)
        let lum_x2 = format!("{}/lum_x2", ns);
        let w_over = format!("{}/w_over", ns);
        let scale_2 = format!("{}/scale_2", ns);
        let w_clip_min = format!("{}/w_clip_min", ns);
        let w_clip_max = format!("{}/w_clip_max", ns);

        nodes.push(Proto::node("Mul", &[&lum_1ch, &scale_2], &[&lum_x2], &[]));
        nodes.push(Proto::node("Clip", &[&lum_x2, &w_clip_min, &w_clip_max],
            &[&w_over], &[]));

        // w_under = 1 - w_over
        let one = format!("{}/one", ns);
        let w_under = format!("{}/w_under", ns);
        nodes.push(Proto::node("Sub", &[&one, &w_over], &[&w_under], &[]));

        // w_neutral = 1 - |lum - 0.5| * 2
        let half = format!("{}/half", ns);
        let lum_diff = format!("{}/lum_diff", ns);
        let lum_abs = format!("{}/lum_abs", ns);
        let lum_abs_x2 = format!("{}/lum_abs_x2", ns);
        let w_neutral = format!("{}/w_neutral", ns);

        nodes.push(Proto::node("Sub", &[&lum_1ch, &half], &[&lum_diff], &[]));
        nodes.push(Proto::node("Abs", &[&lum_diff], &[&lum_abs], &[]));
        nodes.push(Proto::node("Mul", &[&lum_abs, &scale_2], &[&lum_abs_x2], &[]));
        nodes.push(Proto::node("Sub", &[&one, &lum_abs_x2], &[&w_neutral], &[]));

        // Broadcast 1ch → 3ch via Conv(1×1, 1→3, weights=[1,1,1])
        let expand_w = format!("{}/expand_w", ns);
        let expand_b = format!("{}/expand_b", ns);
        let w_over_3 = format!("{}/w_over_3", ns);
        let w_under_3 = format!("{}/w_under_3", ns);
        let w_neutral_3 = format!("{}/w_neutral_3", ns);

        nodes.push(Proto::node("Conv", &[&w_over, &expand_w, &expand_b], &[&w_over_3],
            &[Proto::attribute_ints("kernel_shape", &[1, 1]),
              Proto::attribute_int("group", 1)]));
        nodes.push(Proto::node("Conv", &[&w_under, &expand_w, &expand_b], &[&w_under_3],
            &[Proto::attribute_ints("kernel_shape", &[1, 1]),
              Proto::attribute_int("group", 1)]));
        nodes.push(Proto::node("Conv", &[&w_neutral, &expand_w, &expand_b], &[&w_neutral_3],
            &[Proto::attribute_ints("kernel_shape", &[1, 1]),
              Proto::attribute_int("group", 1)]));

        // Blend: merged = under*w_under + neutral*w_neutral + over*w_over
        let under_x_w = format!("{}/under_x_w", ns);
        let neutral_x_w = format!("{}/neutral_x_w", ns);
        let over_x_w = format!("{}/over_x_w", ns);
        let sum1 = format!("{}/sum1", ns);

        nodes.push(Proto::node("Mul", &[&self.under_source, &w_under_3], &[&under_x_w], &[]));
        nodes.push(Proto::node("Mul", &[&self.neutral_source, &w_neutral_3], &[&neutral_x_w], &[]));
        nodes.push(Proto::node("Mul", &[&self.over_source, &w_over_3], &[&over_x_w], &[]));
        nodes.push(Proto::node("Add", &[&under_x_w, &neutral_x_w], &[&sum1], &[]));
        nodes.push(Proto::node("Add", &[&sum1, &over_x_w], &[&self.frame_tensor], &[]));

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            // Luminance weights [3,1,1,1] for group=3 Conv
            Proto::tensor_proto_float(&format!("{}/lum_w", ns), &[3, 1, 1, 1],
                &[0.299, 0.587, 0.114]),
            Proto::tensor_proto_float(&format!("{}/lum_b", ns), &[3], &[0.0, 0.0, 0.0]),
            // Constants
            Proto::tensor_proto_float_scalar(&format!("{}/lum_clip_min", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/lum_clip_max", ns), 1.0),
            Proto::tensor_proto_float_scalar(&format!("{}/scale_2", ns), 2.0),
            Proto::tensor_proto_float_scalar(&format!("{}/w_clip_min", ns), 0.0),
            Proto::tensor_proto_float_scalar(&format!("{}/w_clip_max", ns), 1.0),
            Proto::tensor_proto_float_scalar(&format!("{}/one", ns), 1.0),
            Proto::tensor_proto_float_scalar(&format!("{}/half", ns), 0.5),
            // Expand weights: 1ch → 3ch Conv(1×1)
            Proto::tensor_proto_float(&format!("{}/expand_w", ns), &[3, 1, 1, 1],
                &[1.0, 1.0, 1.0]),
            Proto::tensor_proto_float(&format!("{}/expand_b", ns), &[3], &[0.0, 0.0, 0.0]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![
            (self.under_source.clone(), 1, vec![1, 3, -1, -1]),
            (self.over_source.clone(), 1, vec![1, 3, -1, -1]),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hdr_merge_node_count() {
        let block = HdrMergeBlock::new().with_concrete_dims(1080, 1920);

        let nodes = block.nodes();
        let inits = block.initializers();

        // 18 nodes: Conv(lum) + ReduceSum + Clip + Mul(scale) + Clip(w_over) +
        //   Sub(w_under) + Sub(lum_diff) + Abs + Mul(scale) + Sub(w_neutral) +
        //   3× Conv(expand) + 3× Mul(blend) + 2× Add
        assert_eq!(nodes.len(), 18, "expected 18 nodes, got {}", nodes.len());
        assert_eq!(inits.len(), 11, "expected 11 initializers, got {}", inits.len());

        // Extra inputs: under + over
        let extra = block.extra_inputs();
        assert_eq!(extra.len(), 2);
        assert_eq!(extra[0].0, "HdrMergeBlock/under");
        assert_eq!(extra[1].0, "HdrMergeBlock/over");

        println!("HdrMergeBlock: {} nodes, {} initializers, {} extra inputs",
            nodes.len(), inits.len(), extra.len());
    }

    #[test]
    fn test_hdr_merge_has_three_inputs() {
        let block = HdrMergeBlock::new();
        // neutral + under + over
        assert_eq!(block.input_tensors().len(), 3, "must have 3 inputs");
    }

    #[test]
    fn test_hdr_merge_has_one_output() {
        let block = HdrMergeBlock::new();
        assert_eq!(block.output_tensors().len(), 1, "must have 1 output");
    }

    #[test]
    fn test_hdr_merge_emits_conv_for_luminance() {
        let block = HdrMergeBlock::new();
        let nodes = block.nodes();
        let has_conv = nodes.iter().any(|n| {
            let s = String::from_utf8_lossy(n);
            s.contains("Conv")
        });
        assert!(has_conv, "must emit Conv for luminance extraction");
    }

    #[test]
    fn test_hdr_merge_weight_computation() {
        let block = HdrMergeBlock::new();
        let nodes = block.nodes();
        // Weight computation: Clip, Sub, Abs, Mul
        let has_clip = nodes.iter().any(|n| {
            let s = String::from_utf8_lossy(n);
            s.contains("Clip")
        });
        let has_abs = nodes.iter().any(|n| {
            let s = String::from_utf8_lossy(n);
            s.contains("Abs")
        });
        assert!(has_clip, "must emit Clip for weight clamping");
        assert!(has_abs, "must emit Abs for neutral weight");
    }

    #[test]
    fn test_hdr_merge_blends_three_exposures() {
        let block = HdrMergeBlock::new();
        let nodes = block.nodes();
        // 3× Mul for exposure blending + 2× Add for combining
        let mul_count = nodes.iter().filter(|n| {
            let s = String::from_utf8_lossy(n);
            s.contains("Mul")
        }).count();
        let add_count = nodes.iter().filter(|n| {
            let s = String::from_utf8_lossy(n);
            s.contains("Add")
        }).count();
        assert!(mul_count >= 3, "must have 3+ Mul for blending, got {}", mul_count);
        assert!(add_count >= 2, "must have 2+ Add for combining, got {}", add_count);
    }
}

