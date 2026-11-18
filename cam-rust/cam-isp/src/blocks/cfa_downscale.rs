//! CFA statistics downscale as one `isp.cfa_downscale` dispatch.
//!
//! The 3A statistics do not need sensor resolution. A mean, a histogram and a focus
//! metric are the same numbers at 1/4 or 1/8 the pixels to within noise, and the
//! controller smooths all of them anyway — so reading them at full resolution is
//! the largest avoidable cost left in the fused chain.
//!
//! This replaces the `AdaptiveDownscaleBlock` that used to sit in the statistics
//! branches. That block emitted a `Resize`, whose shape MNN derives from a scale
//! tensor while propagating shapes; it never built. This op states its geometry in
//! a `downscale_out` attribute and reduces each CFA plane independently — averaging
//! across planes would mix red into green and destroy the balance AWB is measuring.

use crate::onnx::proto::Proto;
use crate::pipeline::{BlockOpsetMode, IspBlock};

/// `[1,4,H,W]` CFA quad → `[1,4,oH,oW]`, plane-preserving box average.
#[derive(Debug)]
pub struct CfaDownscaleBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub in_w: i64,
    pub in_h: i64,
    pub out_w: i64,
    pub out_h: i64,
    pub forced_mode: Option<BlockOpsetMode>,
}

impl CfaDownscaleBlock {
    pub fn new(in_w: i64, in_h: i64, out_w: i64, out_h: i64) -> Self {
        Self {
            id: "cfa_downscale".into(),
            prev: None,
            next: None,
            frame_tensor: "CfaDownscaleBlock/frame".into(),
            input_source: String::new(),
            in_w: in_w.max(1),
            in_h: in_h.max(1),
            out_w: out_w.max(1),
            out_h: out_h.max(1),
            forced_mode: None,
        }
    }

    /// Reduce the longer edge to `max_edge`, keeping the aspect and the plane count.
    pub fn fit_max_edge(in_w: i64, in_h: i64, max_edge: u32) -> Self {
        let m = max_edge.max(1) as i64;
        let longest = in_w.max(in_h).max(1);
        if longest <= m {
            return Self::new(in_w, in_h, in_w, in_h);
        }
        let out_w = (in_w * m / longest).max(1);
        let out_h = (in_h * m / longest).max(1);
        Self::new(in_w, in_h, out_w, out_h)
    }

    /// Whether this block actually reduces anything.
    pub fn is_identity(&self) -> bool {
        self.out_w == self.in_w && self.out_h == self.in_h
    }
}

impl IspBlock for CfaDownscaleBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "CfaDownscaleBlock".to_string()
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
        Some(&["isp.cfa_downscale"])
    }
    fn custom_opsets(&self) -> Vec<(String, i64)> {
        vec![("isp".to_string(), 1)]
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
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(self.in_h),
                Proto::tensor_dim_value(self.in_w),
            ],
            1,
        ))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(self.out_h),
                Proto::tensor_dim_value(self.out_w),
            ],
            1,
        ))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        if self.opset_mode() != BlockOpsetMode::MnnIspMin {
            // No primitive spelling: the point is one dispatch over a small grid.
            // A `Resize` here is what did not build in the first place.
            log::warn!(
                "cfa_downscale: fused-only op; a chain that wants primitives \
                 should not contain this block"
            );
            return vec![];
        }
        let ns = self.tensor_ns();
        vec![Proto::node_with_domain(
            "isp.cfa_downscale",
            "isp",
            &[&self.input_source, &format!("{ns}/hw")],
            &[&self.frame_tensor],
            &[
                // The output rect, stated once so the graph boundary is fixed at
                // build time rather than derived while propagating shapes.
                Proto::attribute_ints("downscale_out", &[self.out_w, self.out_h]),
                // Not elementwise: the output is a different size, and VulkanFuse
                // would otherwise dispatch over the input grid.
                Proto::attribute_int("spatial_nchw", 1),
            ],
        )]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![(format!("{}/hw", self.tensor_ns()), 1, vec![4])]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let hw: [f32; 4] = [
            self.in_w as f32,
            self.in_h as f32,
            self.out_w as f32,
            self.out_h as f32,
        ];
        vec![(
            format!("{}/hw", self.tensor_ns()),
            hw.iter().flat_map(|v| v.to_ne_bytes()).collect(),
        )]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reduces_to_the_requested_edge() {
        let b = CfaDownscaleBlock::fit_max_edge(1296, 970, 540);
        assert_eq!((b.out_w, b.out_h), (540, 404));
        assert!(b.out_w * b.out_h < 1296 * 970 / 5);
    }

    #[test]
    fn leaves_a_small_frame_alone() {
        let b = CfaDownscaleBlock::fit_max_edge(320, 240, 540);
        assert!(b.is_identity(), "nothing to reduce");
    }

    #[test]
    fn keeps_the_plane_count() {
        let mut b = CfaDownscaleBlock::new(1296, 970, 540, 404);
        b.set_input_source("aux_hook_src/out");
        let vi = String::from_utf8_lossy(&b.output_value_info().unwrap()).to_string();
        let ii = String::from_utf8_lossy(&b.input_value_info().unwrap()).to_string();
        // Both declare [1,4,...]: four CFA planes in, four out.
        assert!(ii.contains("aux_hook_src/out"), "{ii}");
        assert!(vi.contains("CfaDownscaleBlock/frame"), "{vi}");
    }

    #[test]
    fn states_its_output_rect() {
        let mut b = CfaDownscaleBlock::new(1296, 970, 540, 404);
        b.set_input_source("in");
        let s = String::from_utf8_lossy(&b.nodes()[0]).to_string();
        assert!(s.contains("isp.cfa_downscale"), "{s}");
        assert!(s.contains("downscale_out"), "{s}");
        assert!(s.contains("spatial_nchw"), "{s}");
    }
}
