//! ToneLutBlock — 7-point tone curve LUT + parametric tone via per-pixel interpolation.
//!
//! Applies a piecewise-linear tone mapping curve defined by 7 control points
//! at fixed x-positions [0, 1/6, 2/6, 3/6, 4/6, 5/6, 1], followed by parametric
//! contrast/brightness/gamma.
//!
//! The block can also apply parametric contrast/brightness/gamma, but in the
//! hybrid pipeline it runs directly after [`ToneBlock`] with identity defaults
//! for those three: the rule-based controller sets them there once, and the
//! neural controller supplies only the 7-point LUT.
//!
//! ONNX subgraph:
//! 1. Piecewise-linear LUT from 7 control points at x = [0, 1/6, ..., 1].
//!    For segment i (0..5), `seg_out = clip(x - t_i, 0, seg_width) * slope_i`
//!    where `slope_i = (y_{i+1} - y_i) / seg_width`; `lut_out` sums those
//!    segments onto `y_0`.
//! 2. Apply parametric: `lut_out * contrast + brightness`.
//! 3. Apply gamma: `result ^ (1/gamma)`.
//!
//! At x = k/6: LUT result = y_0 + sum_{j<k} (y_{j+1} - y_j) = y_k  ✓
//!
//! The neural controller computes the 7 LUT values; in the hybrid pipeline this
//! block applies them per-pixel after the parametric tone stage.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// 7-point tone curve LUT block with parametric contrast/brightness/gamma.
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
    /// Contrast factor (1.0 = neutral)
    contrast: f32,
    /// Brightness offset (-1.0 to 1.0)
    brightness: f32,
    /// Gamma (1.0 = linear, 2.2 = sRGB-like)
    gamma: f32,
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
            tone_curve_lut: [
                0.0,
                1.0 / 6.0,
                2.0 / 6.0,
                3.0 / 6.0,
                4.0 / 6.0,
                5.0 / 6.0,
                1.0,
            ],
            contrast: 1.0,
            brightness: 0.0,
            gamma: 1.0,
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

    /// Set contrast (1.0 = neutral, >1.0 = more contrast).
    pub fn set_contrast(&mut self, contrast: f32) {
        self.contrast = contrast;
    }

    /// Set brightness offset (-1.0 to 1.0).
    pub fn set_brightness(&mut self, brightness: f32) {
        self.brightness = brightness;
    }

    /// Set gamma (1.0 = linear).
    pub fn set_gamma(&mut self, gamma: f32) {
        self.gamma = gamma;
    }

    /// Get current contrast.
    pub fn contrast(&self) -> f32 {
        self.contrast
    }

    /// Get current brightness.
    pub fn brightness(&self) -> f32 {
        self.brightness
    }

    /// Get current gamma.
    pub fn gamma(&self) -> f32 {
        self.gamma
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
        // Slopes are derived in the graph from y_i and the shared seg_width.

        // Accumulator starts at y_0 (tone_curve_lut[0])
        let mut acc = format!("{}/lut_y0", ns);
        nodes.push(Proto::node(
            "Identity",
            &[&format!("{}/y0", ns)],
            &[&acc],
            &[],
        ));

        // For each segment i: acc += clip(x - t_i, 0, 1/6) * slope_i
        for i in 0..6 {
            let seg_in = format!("{}/seg{}_in", ns, i);
            let seg_clamped = format!("{}/seg{}_clamped", ns, i);
            let seg_scaled = format!("{}/seg{}_scaled", ns, i);
            let acc_out = format!("{}/lut_acc{}", ns, i + 1);

            // slope_i = (y_{i+1} - y_i) / seg_width
            let diff = format!("{}/seg{}_diff", ns, i);
            let slope = format!("{}/seg{}_slope", ns, i);
            nodes.push(Proto::node(
                "Sub",
                &[&format!("{}/y{}", ns, i + 1), &format!("{}/y{}", ns, i)],
                &[&diff],
                &[],
            ));
            nodes.push(Proto::node(
                "Div",
                &[&diff, &format!("{}/seg_width", ns)],
                &[&slope],
                &[],
            ));

            // seg_in = x - t_i
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
                &[&seg_clamped, &slope],
                &[&seg_scaled],
                &[],
            ));

            // acc_out = acc + seg_scaled
            nodes.push(Proto::node("Add", &[&acc, &seg_scaled], &[&acc_out], &[]));
            acc = acc_out;
        }

        // Apply parametric contrast: acc * contrast
        let contrasted = format!("{}/lut_contrasted", ns);
        nodes.push(Proto::node(
            "Mul",
            &[&acc, &format!("{}/contrast", ns)],
            &[&contrasted],
            &[],
        ));

        // Apply brightness: contrasted + brightness
        let brightened = format!("{}/lut_brightened", ns);
        nodes.push(Proto::node(
            "Add",
            &[&contrasted, &format!("{}/brightness", ns)],
            &[&brightened],
            &[],
        ));

        // Clip to [0, 1]
        let clipped = format!("{}/lut_clipped", ns);
        nodes.push(Proto::node(
            "Clip",
            &[&brightened, &format!("{}/zero", ns), &format!("{}/one", ns)],
            &[&clipped],
            &[],
        ));

        // Apply gamma: clipped ^ (1/gamma) -> pow(clipped, inv_gamma)
        let gamma_corrected = format!("{}/lut_gamma", ns);
        nodes.push(Proto::node(
            "Pow",
            &[&clipped, &format!("{}/inv_gamma", ns)],
            &[&gamma_corrected],
            &[],
        ));

        // Final: gamma_corrected -> output
        nodes.push(Proto::node(
            "Identity",
            &[&gamma_corrected],
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

        // 7 tone curve point values: y_0..y_6
        for i in 0..7 {
            inputs.push((format!("{}/y{}", ns, i), 1, vec![1]));
        }

        // 6 segment start positions: t_i = i/6 (for Sub x - t_i)
        for i in 0..6 {
            inputs.push((format!("{}/t{}", ns, i), 1, vec![1]));
        }

        // Shared segment width (1/6); slopes are derived from y_i in the graph.
        inputs.push((format!("{}/seg_width", ns), 1, vec![1]));

        // zero constant for Clip lower bound
        inputs.push((format!("{}/zero", ns), 1, vec![1]));

        // one constant for Clip upper bound
        inputs.push((format!("{}/one", ns), 1, vec![1]));

        // Contrast, brightness, inv_gamma (parametric tone)
        inputs.push((format!("{}/contrast", ns), 1, vec![1]));
        inputs.push((format!("{}/brightness", ns), 1, vec![1]));
        inputs.push((format!("{}/inv_gamma", ns), 1, vec![1]));

        inputs
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        let mut inits = Vec::new();

        // 7 tone curve point values (default: identity LUT)
        for i in 0..7 {
            inits.push((
                format!("{}/y{}", ns, i),
                self.tone_curve_lut[i].to_ne_bytes().to_vec(),
            ));
        }

        // 6 segment start positions: t_i = i/6
        for i in 0..6 {
            let ti: f32 = i as f32 / 6.0;
            inits.push((format!("{}/t{}", ns, i), ti.to_ne_bytes().to_vec()));
        }

        // Shared segment width (1/6)
        inits.push((
            format!("{}/seg_width", ns),
            (1.0f32 / 6.0).to_ne_bytes().to_vec(),
        ));

        // zero constant
        inits.push((format!("{}/zero", ns), 0.0f32.to_ne_bytes().to_vec()));

        // one constant
        inits.push((format!("{}/one", ns), 1.0f32.to_ne_bytes().to_vec()));

        // Contrast, brightness, inv_gamma defaults
        inits.push((
            format!("{}/contrast", ns),
            self.contrast.to_ne_bytes().to_vec(),
        ));
        inits.push((
            format!("{}/brightness", ns),
            self.brightness.to_ne_bytes().to_vec(),
        ));
        inits.push((
            format!("{}/inv_gamma", ns),
            (1.0 / self.gamma.max(1e-4)).to_ne_bytes().to_vec(),
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
        // initial Identity (y0) = 1
        // 6 segments × (diff Sub + slope Div + Sub + Clip + Mul + Add) = 36
        // contrast Mul + brightness Add + Clip + Pow + final Identity = 5
        assert_eq!(nodes.len(), 42, "Expected 42 nodes, got {}", nodes.len());
    }

    #[test]
    fn test_tone_lut_extra_inputs_count() {
        let b = ToneLutBlock::new();
        let inputs = b.extra_inputs();
        // 7 tone points + 6 segment starts + 1 segment width
        // + zero + one + contrast + brightness + inv_gamma = 19
        assert_eq!(
            inputs.len(),
            19,
            "Expected 19 extra inputs, got {}",
            inputs.len()
        );
        let names: std::collections::HashSet<_> =
            inputs.iter().map(|(name, _, _)| name.as_str()).collect();
        assert_eq!(names.len(), inputs.len(), "duplicate ToneLutBlock input");
        for i in 0..7 {
            assert!(names.contains(format!("ToneLutBlock/y{}", i).as_str()));
        }
        assert!(!names.iter().any(|n| n.contains("/slope")));
    }

    #[test]
    fn test_tone_lut_extra_input_defaults_count() {
        let b = ToneLutBlock::new();
        let inits = b.extra_input_defaults();
        assert_eq!(
            inits.len(),
            19,
            "Expected 19 initializers, got {}",
            inits.len()
        );
        let names: std::collections::HashSet<_> =
            inits.iter().map(|(name, _)| name.as_str()).collect();
        assert_eq!(
            names.len(),
            inits.len(),
            "duplicate ToneLutBlock initializer"
        );
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
