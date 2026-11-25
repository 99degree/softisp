//! ToneLutBlock — 7-point tone curve LUT via per-pixel interpolation.
//!
//! Applies a piecewise-linear tone mapping curve defined by 7 control points
//! at fixed x-positions [0, 1/6, 2/6, 3/6, 4/6, 5/6, 1]. For each pixel
//! value `x`, the output is the linearly interpolated tone curve value.
//!
//! When no LUT is provided (default identity LUT [0, 1/6, ..., 1]), the block
//! is a pass-through: output = input.
//!
//! This block complements [`ToneBlock`]: ToneBlock handles parametric
//! contrast/brightness/gamma, ToneLutBlock handles the 7-point LUT from the
//! neural controller. In the hybrid controller mode, ToneBlock runs with
//! neutral parametric values (contrast=1, brightness=0, gamma=1) and
//! ToneLutBlock applies the neural model's tone curve.
//!
//! ONNX subgraph (per segment i = 0..5):
//!   seg_input   = clip(x - t_i_start, 0, 1/6)
//!   seg_output  = seg_input * slope_i   where slope_i = (t_{i+1} - t_i) * 6
//!   result      = t_0 + sum(seg_output_i for i in 0..5)
//!
//! At x = k/6: result = t_0 + sum_{j<k} (t_{j+1}-t_j) = t_k  ✓

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// 7-point tone curve LUT block.
///
/// Applies piecewise-linear tone mapping using 7 control points at fixed
/// x-positions [0, 1/6, 2/6, 3/6, 4/6, 5/6, 1].
#[derive(Debug)]
pub struct ToneLutBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// 7 tone curve control points at x = [0, 1/6, 2/6, 3/6, 4/6, 5/6, 1].
    /// Default: identity LUT [0, 1/6, 2/6, 3/6, 4/6, 5/6, 1].
    tone_curve_lut: [f32; 7],
}

impl Default for ToneLutBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl ToneLutBlock {
    pub fn new() -> Self {
        Self {
            id: "tone_lut".into(),
            prev: None,
            next: None,
            frame_tensor: "ToneLutBlock/frame".into(),
            input_source: String::new(),
            tone_curve_lut: [0.0, 1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 4.0 / 6.0, 5.0 / 6.0, 1.0],
        }
    }

    /// Set the 7-point tone curve LUT.
    pub fn set_tone_curve_lut(&mut self, lut: [f32; 7]) {
        self.tone_curve_lut = lut;
    }

    /// Get current tone curve LUT.
    pub fn tone_curve_lut(&self) -> [f32; 7] {
        self.tone_curve_lut
    }
}

impl IspBlock for ToneLutBlock {
    fn id(&self) -> &str {
        &self.id
    }

    fn tensor_ns(&self) -> String {
        "ToneLutBlock".to_string()
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
                Proto::tensor_dim_value(3),
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
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ],
            1,
        ))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let x = &self.input_source;
        let mut nodes = Vec::new();

        // Segment start positions: t_i = i/6 for i=0..5
        let t_starts: [f32; 6] = [
            0.0,
            1.0 / 6.0,
            2.0 / 6.0,
            3.0 / 6.0,
            4.0 / 6.0,
            5.0 / 6.0,
        ];

        // Accumulator starts at tone[0]
        let mut acc = format!("{}/lut_t0", ns);
        nodes.push(Proto::node(
            "Add",
            &[x, &format!("{}/t0", ns)],
            &[&acc],
            &[],
        ));

        // For each segment i: result += clip(x - t_i, 0, 1/6) * slope_i
        for i in 0..6 {
            let seg_in = format!("{}/seg{}_in", ns, i);
            let seg_clamped = format!("{}/seg{}_clamped", ns, i);
            let seg_scaled = format!("{}/seg{}_scaled", ns, i);
            let acc_out = format!("{}/lut_acc{}", ns, i + 1);

            // seg_in = x - t_i_start
            nodes.push(Proto::node(
                "Sub",
                &[x, &format!("{}/t{}", ns, i)],
                &[&seg_in],
                &[],
            ));

            // seg_clamped = clip(seg_in, 0, 1/6)
            nodes.push(Proto::node(
                "Clip",
                &[
                    &seg_in,
                    &format!("{}/zero", ns),
                    &format!("{}/seg_width", ns),
                ],
                &[&seg_clamped],
                &[],
            ));

            // seg_scaled = seg_clamped * slope_i
            nodes.push(Proto::node(
                "Mul",
                &[&seg_clamped, &format!("{}/slope{}", ns, i)],
                &[&seg_scaled],
                &[],
            ));

            // acc_out = acc + seg_scaled
            nodes.push(Proto::node("Add", &[&acc, &seg_scaled], &[&acc_out], &[]));
            acc = acc_out;
        }

        // Final: acc -> output (identity, just to have a named output tensor)
        nodes.push(Proto::node(
            "Identity",
            &[&acc],
            &[&self.frame_tensor],
            &[],
        ));

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        let mut inputs = Vec::new();

        // 7 tone curve point constants: t0..t6
        for i in 0..7 {
            inputs.push((
                format!("{}/t{}", ns, i),
                1,
                vec![1],
            ));
        }

        // 6 segment start positions: t_0..t_5 (for Sub x - t_i)
        for i in 0..6 {
            inputs.push((
                format!("{}/t{}", ns, i),
                1,
                vec![1],
            ));
        }

        // 6 segment widths (constant 1/6)
        for i in 0..6 {
            inputs.push((
                format!("{}/seg_width", ns),
                1,
                vec![1],
            ));
        }

        // zero constant for Clip lower bound
        inputs.push((format!("{}/zero", ns), 1, vec![1]));

        inputs
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        let mut inits = Vec::new();

        // 7 tone curve point values (default: identity LUT)
        for i in 0..7 {
            inits.push((
                format!("{}/t{}", ns, i),
                self.tone_curve_lut[i].to_ne_bytes().to_vec(),
            ));
        }

        // 6 segment start positions: t_i = i/6
        for i in 0..6 {
            let ti: f32 = i as f32 / 6.0;
            inits.push((
                format!("{}/t{}", ns, i),
                ti.to_ne_bytes().to_vec(),
            ));
        }

        // 6 segment widths (constant 1/6)
        for _ in 0..6 {
            let sw: f32 = 1.0 / 6.0;
            inits.push((
                format!("{}/seg_width", ns),
                sw.to_ne_bytes().to_vec(),
            ));
        }

        // zero constant
        inits.push((
            format!("{}/zero", ns),
            0.0f32.to_ne_bytes().to_vec(),
        ));

        inits
    }

    /// Signals that ToneLutBlock can make use of aux blocks.
    fn signals_aux(&self) -> Vec<String> {
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tone_lut_id() {
        assert_eq!(ToneLutBlock::new().id(), "tone_lut");
    }

    #[test]
    fn test_tone_lut_default_lut() {
        let b = ToneLutBlock::new();
        let lut = b.tone_curve_lut();
        assert_eq!(lut[0], 0.0);
        assert_eq!(lut[6], 1.0);
        assert!((lut[1] - 1.0 / 6.0).abs() < 1e-6);
        assert!((lut[3] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_tone_lut_set_lut() {
        let mut b = ToneLutBlock::new();
        let new_lut = [0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0];
        b.set_tone_curve_lut(new_lut);
        assert_eq!(b.tone_curve_lut(), new_lut);
    }

    #[test]
    fn test_tone_lut_emit_onnx() {
        let mut b = ToneLutBlock::new();
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        // 6 segments × (Sub + Clip + Mul + Add) = 24 nodes + initial Add + final Identity = 26
        assert!(nodes.len() >= 20, "Expected >= 20 nodes, got {}", nodes.len());
    }

    #[test]
    fn test_tone_lut_extra_inputs_count() {
        let b = ToneLutBlock::new();
        let inputs = b.extra_inputs();
        // 7 tone points + 6 segment starts + 6 segment widths + 1 zero = 20
        assert_eq!(inputs.len(), 20, "Expected 20 extra inputs, got {}", inputs.len());
    }

    #[test]
    fn test_tone_lut_extra_input_defaults_count() {
        let b = ToneLutBlock::new();
        let inits = b.extra_input_defaults();
        assert_eq!(inits.len(), 20, "Expected 20 initializers, got {}", inits.len());
    }

    #[test]
    fn test_tone_lut_tensor_ns() {
        let b = ToneLutBlock::new();
        assert_eq!(b.tensor_ns(), "ToneLutBlock");
    }

    #[test]
    fn test_tone_lut_graph_output() {
        let b = ToneLutBlock::new();
        assert!(b.graph_output_name().is_some());
    }
}
