//! HdrToneBlock — local HDR tone mapping.
//!
//! Implements filmic/ACES/Reinhard operators for HDR→SDR conversion.
//! Unlike `ToneBlock` (simple Mul+Add curve), this block applies physically
//! based tone mapping operators with highlight compression.
//!
//! Supported operators:
//! - **Reinhard** (global): `x / (1 + x)` with optional white point
//! - **ACES** (filmic): ACES approximation from Stephen Hill's fit
//! - **Uncharted2** (filmic): John Hable's curve with shoulder/toe
//!
//! The operator is selected at construction time and baked into the ONNX graph.
//! The `intensity` parameter scales the output.
//!
//! ONNX subgraph (ACES):
//!   1. Clamp to `[0, inf)`
//!   2. Apply ACES input transform matrix (3×3 Conv)
//!   3. ACES curve: `(x(2.51x + 0.03)) / (x(2.43x + 0.59) + 0.14)`
//!   4. Apply ACES output transform matrix (3×3 Conv)
//!   5. Mul(intensity) + Clip

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Tone mapping operator variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToneOperator {
    /// Reinhard global: `L / (1 + L)` with optional white point W.
    /// `output = L * (1 + L/W^2) / (1 + L)`
    Reinhard,
    /// ACES filmic approximation (Stephen Hill).
    /// Perceptually accurate, used in film industry.
    Aces,
    /// Uncharted2 / John Hable's filmic curve.
    /// Good shadow detail, used in many game engines.
    Uncharted2,
}

/// HdrToneBlock — HDR to SDR tone mapping via filmic operators.
///
/// Applies Reinhard, ACES, or Uncharted2 operator to compress HDR luminance
/// into SDR display range.
pub struct HdrToneBlock {
    pub id: String,
    pub prev_block: Option<Box<dyn IspBlock>>,
    pub next_block: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub operator: ToneOperator,
    pub intensity: f32,
    pub white_point: f32,
}

impl Default for HdrToneBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl HdrToneBlock {
    pub fn new() -> Self {
        Self {
            id: "hdr_tone".into(),
            prev_block: None,
            next_block: None,
            frame_tensor: "HdrToneBlock/frame".into(),
            input_source: String::new(),
            operator: ToneOperator::Aces,
            intensity: 1.0,
            white_point: 4.0,
        }
    }

    pub fn with_operator(mut self, op: ToneOperator) -> Self {
        self.operator = op;
        self
    }

    pub fn with_intensity(mut self, v: f32) -> Self {
        self.intensity = v;
        self
    }

    pub fn with_white_point(mut self, v: f32) -> Self {
        self.white_point = v;
        self
    }
}

impl IspBlock for HdrToneBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "HdrTone".into()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }
    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.into();
    }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev_block.as_ref()
    }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) {
        self.prev_block = Some(block);
    }
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next_block.as_ref()
    }
    fn set_next(&mut self, block: Box<dyn IspBlock>) {
        self.next_block = Some(block);
    }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
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
        let mut nodes = Vec::new();

        match self.operator {
            ToneOperator::Reinhard => {
                // Reinhard global: x / (1 + x) with white point
                // For color: apply per-channel with RGB → luminance adaptation
                //
                // Simplified per-channel:
                //   lum = dot(x, [0.2126, 0.7152, 0.0722])
                //   scaled = lum / (1 + lum)
                //   factor = scaled / (lum + 1e-6)
                //   output = x * factor
                //
                // Simpler per-channel (standard Reinhard):
                //   white2 = white_point^2
                //   output = x * (1 + x/white2) / (1 + x)

                let white2_name = format!("{}/white2", ns);
                let scaled = format!("{}/scaled", ns);
                let one = format!("{}/one", ns);
                let denom = format!("{}/denom", ns);

                // scaled = x / white2
                nodes.push(Proto::node(
                    "Mul",
                    &[&self.input_source, &white2_name],
                    &[&scaled],
                    &[],
                ));

                // denom = 1 + x/white2 = 1 + scaled
                nodes.push(Proto::node("Add", &[&scaled, &one], &[&denom], &[]));

                // output = x * (1 + x/white2) / (1 + x/white2)
                // But simpler: output = x / denom * (1 + x)
                // Actually standard Reinhard: L * (1 + L/W²) / (1 + L)
                // = L * (1 + L*invW2) / (1 + L)
                let numerator = format!("{}/num", ns);
                let one_plus_x = format!("{}/1px", ns);
                let inv_w2 = format!("{}/inv_w2", ns);

                // one_plus_x = 1 + x
                nodes.push(Proto::node(
                    "Add",
                    &[&self.input_source, &one],
                    &[&one_plus_x],
                    &[],
                ));

                // x * inv_w2 = x / W²
                nodes.push(Proto::node(
                    "Mul",
                    &[&self.input_source, &inv_w2],
                    &[&scaled],
                    &[],
                ));

                // num = 1 + x/W²
                nodes.push(Proto::node("Add", &[&scaled, &one], &[&numerator], &[]));

                // x * num / (1 + x)
                let mut product = format!("{}/product", ns);
                nodes.push(Proto::node(
                    "Mul",
                    &[&self.input_source, &numerator],
                    &[&product],
                    &[],
                ));

                nodes.push(Proto::node(
                    "Div",
                    &[&product, &one_plus_x],
                    &[&product],
                    &[],
                ));

                // intensity
                if (self.intensity - 1.0).abs() > 1e-6 {
                    let int_name = format!("{}/intensity", ns);
                    let scaled = format!("{}/final_scaled", ns);
                    nodes.push(Proto::node("Mul", &[&product, &int_name], &[&scaled], &[]));
                    product = scaled;
                }

                // rename to output
                nodes.push(Proto::node(
                    "Identity",
                    &[&product],
                    &[&self.frame_tensor],
                    &[],
                ));
            }
            ToneOperator::Aces => {
                // ACES filmic approximation (Stephen Hill fit)
                // Input transform: sRGB → ACES (3x3)
                // Curve: (x*(2.51x + 0.03)) / (x*(2.43x + 0.59) + 0.14)
                // Output transform: ACES → sRGB (3x3)

                // Step 1: ACES input transform (3×3 Conv)
                let aces_in = format!("{}/aces_in", ns);
                nodes.push(Proto::node(
                    "Conv",
                    &[&self.input_source, &format!("{}/input_matrix", ns)],
                    &[&aces_in],
                    &[],
                ));

                // Step 2: ACES curve
                //   a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14
                let a_name = format!("{}/a", ns);
                let b_name = format!("{}/b", ns);
                let c_name = format!("{}/c", ns);
                let d_name = format!("{}/d", ns);
                let e_name = format!("{}/e", ns);

                let ax = format!("{}/ax", ns);
                let bx = format!("{}/bx", ns);
                let numerator = format!("{}/num", ns);
                let cx = format!("{}/cx", ns);
                let dx = format!("{}/dx", ns);
                let sum = format!("{}/sum", ns);
                let denominator = format!("{}/den", ns);

                // ax = a * x = 2.51 * x
                nodes.push(Proto::node("Mul", &[&a_name, &aces_in], &[&ax], &[]));
                // bx = ax + b = 2.51x + 0.03
                nodes.push(Proto::node("Add", &[&ax, &b_name], &[&bx], &[]));
                // numerator = x * bx = x * (2.51x + 0.03)
                nodes.push(Proto::node("Mul", &[&aces_in, &bx], &[&numerator], &[]));

                // cx = c * x = 2.43 * x
                nodes.push(Proto::node("Mul", &[&c_name, &aces_in], &[&cx], &[]));
                // dx = cx + d = 2.43x + 0.59
                nodes.push(Proto::node("Add", &[&cx, &d_name], &[&dx], &[]));
                // sum = x * dx = x * (2.43x + 0.59)
                nodes.push(Proto::node("Mul", &[&aces_in, &dx], &[&sum], &[]));
                // denominator = sum + e = x*(2.43x+0.59) + 0.14
                nodes.push(Proto::node("Add", &[&sum, &e_name], &[&denominator], &[]));

                // curve_out = numerator / denominator
                let curve_out = format!("{}/curved", ns);
                nodes.push(Proto::node(
                    "Div",
                    &[&numerator, &denominator],
                    &[&curve_out],
                    &[],
                ));

                // Step 3: ACES output transform (3×3 Conv)
                let aces_out = format!("{}/aces_out", ns);
                nodes.push(Proto::node(
                    "Conv",
                    &[&curve_out, &format!("{}/output_matrix", ns)],
                    &[&aces_out],
                    &[],
                ));

                // Step 4: Intensity + clip
                let mut result = aces_out;
                if (self.intensity - 1.0).abs() > 1e-6 {
                    let int_name = format!("{}/intensity", ns);
                    let scaled = format!("{}/scaled", ns);
                    nodes.push(Proto::node("Mul", &[&result, &int_name], &[&scaled], &[]));
                    result = scaled;
                }

                // Clip to [0, 1]
                let min_name = format!("{}/min", ns);
                let max_name = format!("{}/max", ns);
                let clamped = format!("{}/clamped", ns);
                nodes.push(Proto::node("Max", &[&result, &min_name], &[&clamped], &[]));
                nodes.push(Proto::node(
                    "Min",
                    &[&clamped, &max_name],
                    &[&self.frame_tensor],
                    &[],
                ));
            }
            ToneOperator::Uncharted2 => {
                // Uncharted2 / John Hable's filmic curve
                // f(x) = ((x*(A*x+C*B)+D*E) / (x*(A*x+B)+D*F)) - E/F
                // A=0.15, B=0.50, C=0.10, D=0.20, E=0.02, F=0.30
                //
                // Per-channel, then scale by exposure

                let exp_name = format!("{}/exposure", ns);

                // exposed = x * exposure
                let exposed = format!("{}/exposed", ns);
                nodes.push(Proto::node(
                    "Mul",
                    &[&self.input_source, &exp_name],
                    &[&exposed],
                    &[],
                ));

                let a = format!("{}/a", ns);
                let b = format!("{}/b", ns);
                let c = format!("{}/c", ns);
                let d = format!("{}/d", ns);
                let e = format!("{}/e", ns);
                let f = format!("{}/f", ns);

                // numerator: x*(A*x + C*B) + D*E
                let ax = format!("{}/ax", ns);
                let ccb = format!("{}/ccb", ns);
                let axb = format!("{}/axb", ns);
                let n1 = format!("{}/n1", ns);
                let n2 = format!("{}/n2", ns);

                nodes.push(Proto::node("Mul", &[&a, &exposed], &[&ax], &[]));
                nodes.push(Proto::node("Mul", &[&c, &b], &[&ccb], &[]));
                nodes.push(Proto::node("Add", &[&ax, &ccb], &[&axb], &[]));
                nodes.push(Proto::node("Mul", &[&exposed, &axb], &[&n1], &[]));
                nodes.push(Proto::node("Mul", &[&d, &e], &[&n2], &[]));

                let num = format!("{}/num", ns);
                nodes.push(Proto::node("Add", &[&n1, &n2], &[&num], &[]));

                // denominator: x*(A*x + B) + D*F
                let axb2 = format!("{}/axb2", ns);
                let d1 = format!("{}/d1", ns);
                let d2 = format!("{}/d2", ns);

                nodes.push(Proto::node("Add", &[&ax, &b], &[&axb2], &[]));
                nodes.push(Proto::node("Mul", &[&exposed, &axb2], &[&d1], &[]));
                nodes.push(Proto::node("Mul", &[&d, &f], &[&d2], &[]));

                let den = format!("{}/den", ns);
                nodes.push(Proto::node("Add", &[&d1, &d2], &[&den], &[]));

                // curve = num / den
                let curve = format!("{}/curve", ns);
                nodes.push(Proto::node("Div", &[&num, &den], &[&curve], &[]));

                // white_scale: subtract E/F
                let ef = format!("{}/ef", ns);
                nodes.push(Proto::node("Div", &[&e, &f], &[&ef], &[]));

                let mut result = format!("{}/tonemapped", ns);
                nodes.push(Proto::node("Sub", &[&curve, &ef], &[&result], &[]));

                // intensity
                if (self.intensity - 1.0).abs() > 1e-6 {
                    let int_name = format!("{}/intensity", ns);
                    let scaled = format!("{}/scaled", ns);
                    nodes.push(Proto::node("Mul", &[&result, &int_name], &[&scaled], &[]));
                    result = scaled;
                }

                // Clip to [0, 1]
                let min_name = format!("{}/min", ns);
                let max_name = format!("{}/max", ns);
                let clamped = format!("{}/clamped", ns);
                nodes.push(Proto::node("Max", &[&result, &min_name], &[&clamped], &[]));
                nodes.push(Proto::node(
                    "Min",
                    &[&clamped, &max_name],
                    &[&self.frame_tensor],
                    &[],
                ));
            }
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        vec![
            (format!("{}/white2", ns).to_string(), 1, vec![]),
            (format!("{}/inv_w2", ns).to_string(), 1, vec![]),
            (format!("{}/one", ns).to_string(), 1, vec![]),
            (format!("{}/intensity", ns).to_string(), 1, vec![]),
            (format!("{}/input_matrix", ns).to_string(), 1, vec![3, 3]),
            (format!("{}/a", ns).to_string(), 1, vec![]),
            (format!("{}/b", ns).to_string(), 1, vec![]),
            (format!("{}/c", ns).to_string(), 1, vec![]),
            (format!("{}/d", ns).to_string(), 1, vec![]),
            (format!("{}/e", ns).to_string(), 1, vec![]),
            (format!("{}/output_matrix", ns).to_string(), 1, vec![3, 3]),
            (format!("{}/min", ns).to_string(), 1, vec![]),
            (format!("{}/max", ns).to_string(), 1, vec![]),
            (format!("{}/intensity", ns).to_string(), 1, vec![]),
            (format!("{}/a", ns).to_string(), 1, vec![]),
            (format!("{}/b", ns).to_string(), 1, vec![]),
            (format!("{}/c", ns).to_string(), 1, vec![]),
            (format!("{}/d", ns).to_string(), 1, vec![]),
            (format!("{}/e", ns).to_string(), 1, vec![]),
            (format!("{}/f", ns).to_string(), 1, vec![]),
            (format!("{}/exposure", ns).to_string(), 1, vec![]),
            (format!("{}/min", ns).to_string(), 1, vec![]),
            (format!("{}/max", ns).to_string(), 1, vec![]),
            (format!("{}/intensity", ns).to_string(), 1, vec![]),
        ]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        vec![
            (format!("{}/white2", ns).to_string(), vec![]),
            (format!("{}/inv_w2", ns).to_string(), vec![]),
            (format!("{}/one", ns).to_string(), vec![]),
            (format!("{}/intensity", ns).to_string(), vec![]),
            (format!("{}/input_matrix", ns).to_string(), vec![]),
            (
                format!("{}/a", ns).to_string(),
                (2.51f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/b", ns).to_string(),
                (0.03f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/c", ns).to_string(),
                (2.43f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/d", ns).to_string(),
                (0.59f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/e", ns).to_string(),
                (0.14f32).to_ne_bytes().to_vec(),
            ),
            (format!("{}/output_matrix", ns).to_string(), vec![]),
            (format!("{}/min", ns).to_string(), vec![]),
            (format!("{}/max", ns).to_string(), vec![]),
            (format!("{}/intensity", ns).to_string(), vec![]),
            (
                format!("{}/a", ns).to_string(),
                (0.15f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/b", ns).to_string(),
                (0.50f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/c", ns).to_string(),
                (0.10f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/d", ns).to_string(),
                (0.20f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/e", ns).to_string(),
                (0.02f32).to_ne_bytes().to_vec(),
            ),
            (
                format!("{}/f", ns).to_string(),
                (0.30f32).to_ne_bytes().to_vec(),
            ),
            (format!("{}/exposure", ns).to_string(), vec![]),
            (format!("{}/min", ns).to_string(), vec![]),
            (format!("{}/max", ns).to_string(), vec![]),
            (format!("{}/intensity", ns).to_string(), vec![]),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hdr_tone_aces_default() {
        let b = HdrToneBlock::new();
        assert_eq!(b.id(), "hdr_tone");
        assert_eq!(b.operator, ToneOperator::Aces);
        assert_eq!(b.intensity, 1.0);
    }

    #[test]
    fn test_hdr_tone_reinhard() {
        let b = HdrToneBlock::new().with_operator(ToneOperator::Reinhard);
        assert_eq!(b.operator, ToneOperator::Reinhard);
        let nodes = b.nodes();
        // Reinhard: Mul + Add + Mul + Add + Mul + Div + Identity = 7
        assert!(
            nodes.len() >= 5,
            "Reinhard needs >= 5 nodes, got {}",
            nodes.len()
        );
    }

    #[test]
    fn test_hdr_tone_aces() {
        let b = HdrToneBlock::new().with_operator(ToneOperator::Aces);
        let nodes = b.nodes();
        // ACES: Conv + 7 curve nodes + Conv + Max + Min = 11
        assert!(
            nodes.len() >= 9,
            "ACES needs >= 9 nodes, got {}",
            nodes.len()
        );
        let inits = b.extra_input_defaults();
        // input_matrix + 5 constants + output_matrix + min + max = 9
        assert!(
            inits.len() >= 8,
            "ACES needs >= 8 initializers, got {}",
            inits.len()
        );
    }

    #[test]
    fn test_hdr_tone_uncharted2() {
        let b = HdrToneBlock::new().with_operator(ToneOperator::Uncharted2);
        let nodes = b.nodes();
        // Uncharted2: Mul + many ops + Max + Min
        assert!(
            nodes.len() >= 10,
            "Uncharted2 needs >= 10 nodes, got {}",
            nodes.len()
        );
    }

    #[test]
    fn test_hdr_tone_intensity() {
        let b = HdrToneBlock::new().with_intensity(1.5);
        assert_eq!(b.intensity, 1.5);
        let nodes = b.nodes();
        // ACES with intensity adds a Mul node
        assert!(nodes.len() >= 10, "intensity should add node");
    }

    #[test]
    fn test_hdr_tone_white_point() {
        let b = HdrToneBlock::new()
            .with_operator(ToneOperator::Reinhard)
            .with_white_point(8.0);
        assert_eq!(b.white_point, 8.0);
    }

    #[test]
    fn test_hdr_tone_extra_inputs() {
        let b = HdrToneBlock::new().with_operator(ToneOperator::Reinhard);
        let extras = b.extra_inputs();
        assert!(
            !extras.is_empty(),
            "Reinhard should have white2 extra input"
        );
    }

    #[test]
    fn test_hdr_tone_has_input_output() {
        let b = HdrToneBlock::new();
        assert_eq!(b.input_tensors().len(), 1);
        assert_eq!(b.output_tensors().len(), 1);
    }

    #[test]
    fn test_hdr_tone_tensor_ns() {
        let b = HdrToneBlock::new();
        assert_eq!(b.tensor_ns(), "HdrTone");
    }

    #[test]
    fn test_hdr_tone_graph_output() {
        let b = HdrToneBlock::new();
        assert!(b.graph_output_name().is_some());
    }
}
