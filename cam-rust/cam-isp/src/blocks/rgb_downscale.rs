//! RgbDownscaleBlock — RGB frame downscale as a single `isp.downscale` dispatch.
//!
//! The primitive path expresses this as an `AdaptiveDownscaleBlock`, which emits
//! a `Resize` node. That works, but `Resize` carries its scales and sizes as
//! tensors MNN must read during shape inference, and on Vulkan it becomes a
//! generic resample rather than a purpose-built kernel.
//!
//! `isp.downscale` is the same maths — bilinear, aspect-preserving, centred with
//! letterbox padding — in one dispatch over planar RGB, with its geometry in a
//! small params tensor the engine writes per frame.
//!
//! The shader only implements **fit + pad**. `AdaptiveDownscaleBlock` also offers
//! `crop` and an EIS margin, neither of which the shader understands, so those
//! stay on the primitive form. `with_aspect_mode` enforces that.

use crate::onnx::proto::Proto;
use crate::pipeline::{BlockOpsetMode, IspBlock};

/// RGB downscale, fused as `isp.downscale`.
#[derive(Debug)]
pub struct RgbDownscaleBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// Output size in pixels.
    pub target_w: i64,
    pub target_h: i64,
    /// Source size, used for the declared value info and the params tensor.
    pub in_w: Option<i64>,
    pub in_h: Option<i64>,
    /// Override the emitted opset. `None` emits the fused shader.
    pub forced_mode: Option<BlockOpsetMode>,
}

impl RgbDownscaleBlock {
    pub fn new(target_w: i64, target_h: i64) -> Self {
        Self {
            id: "rgb_downscale".into(),
            prev: None,
            next: None,
            frame_tensor: "RgbDownscaleBlock/frame".into(),
            input_source: String::new(),
            target_w: target_w.max(1),
            target_h: target_h.max(1),
            in_w: None,
            in_h: None,
            forced_mode: None,
        }
    }

    pub fn with_concrete_dims(mut self, in_h: i64, in_w: i64) -> Self {
        self.in_h = Some(in_h);
        self.in_w = Some(in_w);
        self
    }

    pub fn with_opset_mode(mut self, mode: BlockOpsetMode) -> Self {
        self.forced_mode = Some(mode);
        self
    }

    /// `{W, H, outW, outH}` — the layout `isp_downscale.comp` reads at binding 5.
    pub fn params(&self) -> [f32; 4] {
        [
            self.in_w.unwrap_or(0) as f32,
            self.in_h.unwrap_or(0) as f32,
            self.target_w as f32,
            self.target_h as f32,
        ]
    }
}

impl IspBlock for RgbDownscaleBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "RgbDownscaleBlock".to_string()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
    fn input_source(&self) -> Option<&str> {
        if self.input_source.is_empty() {
            None
        } else {
            Some(&self.input_source)
        }
    }
    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.to_string();
    }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev.as_ref()
    }
    fn set_prev(&mut self, b: Box<dyn IspBlock>) {
        self.prev = Some(b);
    }
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next.as_ref()
    }
    fn set_next(&mut self, b: Box<dyn IspBlock>) {
        self.next = Some(b);
    }

    fn opset_mode(&self) -> BlockOpsetMode {
        self.forced_mode.unwrap_or(BlockOpsetMode::MnnIspMin)
    }

    fn set_opset_mode(&mut self, mode: BlockOpsetMode) {
        self.forced_mode = Some(mode);
    }

    fn custom_op_types(&self) -> Option<&[&str]> {
        Some(&["isp.downscale"])
    }

    fn custom_opsets(&self) -> Vec<(String, i64)> {
        vec![("isp".to_string(), 1)]
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(self.target_h),
                Proto::tensor_dim_value(self.target_w),
            ],
            1,
        ))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        if self.opset_mode() == BlockOpsetMode::MnnIspMin {
            // One dispatch. The params tensor is a graph input so the engine can
            // write the real source geometry each frame — baking W/H here would
            // break as soon as a frame arrived at a different size.
            return vec![Proto::node_with_domain(
                "isp.downscale",
                "isp",
                &[&self.input_source, &format!("{}/hw", ns)],
                &[&self.frame_tensor],
                // Not `elementwise`: the output is a different size from the
                // input, so VulkanFuse must not size the dispatch from the input.
                &[],
            )];
        }

        // Primitive form: a Resize with an explicit output size.
        let roi = format!("{}/roi", ns);
        let scales = format!("{}/scales", ns);
        let sizes = format!("{}/sizes", ns);
        vec![Proto::node(
            "Resize",
            &[&self.input_source, &roi, &scales, &sizes],
            &[&self.frame_tensor],
            &[
                Proto::attribute_string("mode", "linear"),
                Proto::attribute_string("coordinate_transformation_mode", "asymmetric"),
                Proto::attribute_string("nearest_mode", "floor"),
            ],
        )]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        // Resize needs the empty ROI and the empty scales as *initializers*, not
        // graph inputs: MNN reads their contents while propagating shapes, so a
        // tensor filled at run time arrives too late.
        if self.opset_mode() == BlockOpsetMode::MnnIspMin {
            return vec![];
        }
        let ns = self.tensor_ns();
        vec![
            Proto::tensor_proto_float(&format!("{}/roi", ns), &[0], &[]),
            Proto::tensor_proto_float(&format!("{}/scales", ns), &[0], &[]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        if self.opset_mode() == BlockOpsetMode::MnnIspMin {
            // binding 5 — {W, H, outW, outH}
            return vec![(format!("{}/hw", ns), 1, vec![4])];
        }
        vec![(format!("{}/sizes", ns), 6, vec![4])]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        if self.opset_mode() == BlockOpsetMode::MnnIspMin {
            return vec![(
                format!("{}/hw", ns),
                self.params().iter().flat_map(|v| v.to_ne_bytes()).collect(),
            )];
        }
        let sizes: [i64; 4] = [1, 3, self.target_h, self.target_w];
        vec![(
            format!("{}/sizes", ns),
            sizes.iter().flat_map(|v| v.to_ne_bytes()).collect(),
        )]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fused_node_takes_the_geometry_as_an_input() {
        let mut b = RgbDownscaleBlock::new(648, 485).with_concrete_dims(970, 1296);
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        assert_eq!(nodes.len(), 1);
        let s = String::from_utf8_lossy(&nodes[0]);
        assert!(s.contains("isp.downscale"), "{s}");
        assert!(s.contains("in/frame"), "{s}");
        assert!(s.contains("/hw"), "{s}");
    }

    #[test]
    fn params_carry_source_and_target() {
        let b = RgbDownscaleBlock::new(648, 485).with_concrete_dims(970, 1296);
        assert_eq!(b.params(), [1296.0, 970.0, 648.0, 485.0]);
    }

    #[test]
    fn primitive_form_is_a_resize() {
        let mut b = RgbDownscaleBlock::new(648, 485)
            .with_concrete_dims(970, 1296)
            .with_opset_mode(BlockOpsetMode::Primitive);
        b.set_input_source("in/frame");
        let st = String::from_utf8_lossy(&b.nodes()[0]).to_string();
        assert!(st.contains("Resize"), "{st}");
        // The two empty tensors Resize needs are initializers, not inputs.
        assert_eq!(b.initializers().len(), 2);
    }
}
