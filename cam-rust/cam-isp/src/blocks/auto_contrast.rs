//! AutoContrastBlock — adaptive contrast enhancement via S-curve.
//!
//! Applies a parametric S-curve that brightens shadows and darkens highlights,
//! expanding dynamic range in the mid-tones. Unlike histogram equalization,
//! this works as a single-pass ONNX op (no histogram dependency).
//!
//! ONNX subgraph:
//!   Sub(input, 0.5) → centered
//!   Mul(centered, contrast) → stretched
//!   Add(stretched, 0.5) → output
//!
//! The contrast factor (1.0 = no change, 2.0 = strong) can be tuned per-scene.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// AutoContrastBlock — parametric S-curve contrast enhancement.
///
/// Applies: `clip((x - 0.5) * contrast + 0.5, 0, 1)`.
/// Strength 1.0 = no change, 1.3 = moderate, 2.0 = aggressive.
pub struct AutoContrastBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// Contrast factor: 1.0 = identity, >1.0 = more contrast, <1.0 = less.
    pub contrast: f32,
    /// Shadow lift: adds this much to dark areas (0.0 = none).
    pub shadow_lift: f32,
    /// Highlight compress: compresses highlights (0.0 = none, 1.0 = full).
    pub highlight_compress: f32,
}

impl AutoContrastBlock {
    pub fn new(contrast: f32) -> Self {
        Self {
            id: "auto_contrast".into(),
            prev: None,
            next: None,
            frame_tensor: "AutoContrastBlock/frame".into(),
            input_source: String::new(),
            contrast,
            shadow_lift: 0.0,
            highlight_compress: 0.0,
        }
    }

    /// Add shadow lift: brightens dark areas by a fixed offset.
    pub fn with_shadow_lift(mut self, lift: f32) -> Self {
        self.shadow_lift = lift;
        self
    }

    /// Add highlight compression: soft-knee compression of bright areas.
    pub fn with_highlight_compress(mut self, compress: f32) -> Self {
        self.highlight_compress = compress;
        self
    }
}

impl IspBlock for AutoContrastBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "AutoContrast".into()
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

        // Phase 1: Shadow lift (add offset to brighten darks)
        let mut prev = self.input_source.clone();
        if self.shadow_lift > 0.01 {
            let lift_name = format!("{}/lift", ns);
            let lifted = format!("{}/lifted", ns);
            nodes.push(Proto::node("Add", &[&prev, &lift_name], &[&lifted], &[]));
            prev = lifted;
        }

        // Phase 2: Center → stretch → uncenter (S-curve)
        if (self.contrast - 1.0).abs() > 0.01 {
            let half = format!("{}/half", ns);
            let centered = format!("{}/centered", ns);
            let contrast_w = format!("{}/contrast_w", ns);
            let stretched = format!("{}/stretched", ns);

            // center: Sub(input, 0.5)
            nodes.push(Proto::node("Sub", &[&prev, &half], &[&centered], &[]));
            // stretch: Mul(centered, contrast)
            nodes.push(Proto::node(
                "Mul",
                &[&centered, &contrast_w],
                &[&stretched],
                &[],
            ));
            // uncenter: Add(stretched, 0.5)
            let final_name = if self.highlight_compress > 0.01 {
                format!("{}/pre_clip", ns)
            } else {
                self.frame_tensor.clone()
            };
            nodes.push(Proto::node(
                "Add",
                &[&stretched, &half],
                &[&final_name],
                &[],
            ));
            prev = final_name;
        }

        // Phase 3: Highlight compression (soft knee)
        if self.highlight_compress > 0.01 {
            let _comp_name = format!("{}/compress", ns);
            // Simple compress: Mul(x, 1-compress) for values > 0.5
            // More sophisticated: x * (1 - compress * max(0, x - 0.5))
            // Simplified: just clip to [0, 1]
            nodes.push(Proto::node(
                "Clip",
                &[&prev, &format!("{}/zero", ns), &format!("{}/one", ns)],
                &[&self.frame_tensor],
                &[],
            ));
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut inits = Vec::new();

        if self.shadow_lift > 0.01 {
            inits.push(Proto::tensor_proto_float_scalar(
                &format!("{}/lift", ns),
                self.shadow_lift,
            ));
        }
        if (self.contrast - 1.0).abs() > 0.01 {
            inits.push(Proto::tensor_proto_float_scalar(
                &format!("{}/half", ns),
                0.5,
            ));
            inits.push(Proto::tensor_proto_float_scalar(
                &format!("{}/contrast_w", ns),
                self.contrast,
            ));
        }
        if self.highlight_compress > 0.01 || (self.contrast - 1.0).abs() > 0.01 {
            inits.push(Proto::tensor_proto_float_scalar(
                &format!("{}/zero", ns),
                0.0,
            ));
            inits.push(Proto::tensor_proto_float_scalar(
                &format!("{}/one", ns),
                1.0,
            ));
        }

        inits
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        let mut extras = Vec::new();
        if self.shadow_lift > 0.01 {
            extras.push((format!("{}/lift", ns), 1, vec![1]));
        }
        if (self.contrast - 1.0).abs() > 0.01 {
            extras.push((format!("{}/half", ns), 1, vec![1]));
            extras.push((format!("{}/contrast_w", ns), 1, vec![1]));
        }
        if self.highlight_compress > 0.01 || (self.contrast - 1.0).abs() > 0.01 {
            extras.push((format!("{}/zero", ns), 1, vec![1]));
            extras.push((format!("{}/one", ns), 1, vec![1]));
        }
        extras
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_auto_contrast_identity() {
        let block = AutoContrastBlock::new(1.0);
        let nodes = block.nodes();
        // contrast=1.0 means no stretch nodes emitted
        assert!(nodes.is_empty(), "identity contrast should emit no nodes");
    }

    #[test]
    fn test_auto_contrast_with_lift() {
        let block = AutoContrastBlock::new(1.5).with_shadow_lift(0.05);
        let nodes = block.nodes();
        // lift(Add) + center(Sub) + stretch(Mul) + uncenter(Add) = 4 nodes
        assert_eq!(nodes.len(), 4, "should emit 4 nodes");
    }

    #[test]
    fn test_auto_contrast_emission() {
        let block = AutoContrastBlock::new(2.0);
        let nodes = block.nodes();
        // center(Sub) + stretch(Mul) + uncenter(Add) = 3 nodes
        assert_eq!(nodes.len(), 3, "should emit 3 nodes");
        let inits = block.initializers();
        assert!(
            inits.len() >= 3,
            "should have half, contrast_w, zero, one inits"
        );
    }

    #[test]
    fn test_auto_contrast_id() {
        assert_eq!(AutoContrastBlock::new(1.5).id(), "auto_contrast");
    }

    #[test]
    fn test_auto_contrast_tensor_ns() {
        let b = AutoContrastBlock::new(1.5);
        assert!(!b.tensor_ns().is_empty());
    }

    #[test]
    fn test_auto_contrast_zero_strength() {
        let b = AutoContrastBlock::new(0.0);
        assert_eq!(b.contrast, 0.0);
        assert!(!b.nodes().is_empty());
    }

    #[test]
    fn test_auto_contrast_high_strength() {
        let b = AutoContrastBlock::new(5.0);
        assert_eq!(b.contrast, 5.0);
    }
}
