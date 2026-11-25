//! The whole ISP chain as one `isp.pipeline` dispatch.
//!
//! `isp.pipeline` covers crop → normalize → BLC → LSC → WB → bad-pixel →
//! demosaic → CCM → saturation → shadow lift → gamma → fit/letterbox → display
//! scale, in a single pass over the sensor buffer. It is not a faster version of
//! the per-stage chain, it is a different formulation: the dispatch covers the
//! *output* grid and area-averages each output pixel's footprint in the raw
//! plane, so the resample, the CFA unpack and the demosaic are the same step and
//! no intermediate frame ever exists.
//!
//! That is what makes it worth a profile of its own: seven dispatches become one,
//! and the six full-frame round trips between them disappear.
//!
//! What it does **not** do, and therefore what a profile using it gives up:
//! - no 3A readback (statistics need the per-stage graph's extra outputs),
//! - no edge-directed demosaic — the footprint average is a box filter,
//! - no bilateral / LDCI / FCS / unsharp-as-a-separate-pass,
//! - no EIS warp.
//!
//! Input is `[1,1,H,W]` float (the raw plane, in sensor counts — the shader
//! divides by the white level itself), output is `[1,3,outH,outW]` planar RGB in
//! the 0..1 domain softisp's planar consumers expect.

use crate::onnx::proto::Proto;
use crate::pipeline::{BlockOpsetMode, IspBlock};

/// Which `isp.pipeline` stages to run.
///
/// The shader takes the mask as one integer, so this is a bitfield rather than a
/// struct of bools — and it is the shader's bitfield, not a softisp invention:
/// the bits are the same ones `isp_pipeline.comp` defines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PipelineStages(pub i32);

impl PipelineStages {
    pub const DEMOSAIC: i32 = 1 << 0;
    pub const NORMALIZE: i32 = 1 << 1;
    pub const BLC: i32 = 1 << 2;
    pub const WB: i32 = 1 << 3;
    pub const CCM: i32 = 1 << 4;
    pub const LIFT: i32 = 1 << 5;
    pub const GAMMA: i32 = 1 << 6;
    pub const LETTERBOX: i32 = 1 << 7;
    /// Inverted: set means *skip* the resample and take a 1:1 centre crop.
    pub const NOSCALE: i32 = 1 << 8;
    pub const SATURATION: i32 = 1 << 9;
    pub const DPC: i32 = 1 << 10;
    pub const LSC: i32 = 1 << 11;
    /// Emit 0..1 instead of 0..255.
    pub const LEVELS: i32 = 1 << 12;

    /// Everything except the two bits that change geometry or scale.
    pub const ALL_STAGES: i32 = Self::DEMOSAIC
        | Self::NORMALIZE
        | Self::BLC
        | Self::WB
        | Self::CCM
        | Self::LIFT
        | Self::GAMMA
        | Self::LETTERBOX
        | Self::SATURATION
        | Self::DPC
        | Self::LSC;

    /// The set a camera preview wants: the full RAW chain, display-encoded, and
    /// rescaled to the target size.
    pub const PREVIEW: Self = Self(Self::ALL_STAGES);
}

impl Default for PipelineStages {
    fn default() -> Self {
        Self::PREVIEW
    }
}

/// The whole ISP as `isp.pipeline`.
#[derive(Debug)]
pub struct PipelineAllBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub stages: PipelineStages,
    /// Sensor geometry. The shader reads these from its params buffer to build
    /// the footprint map, so they are not optional.
    pub src_w: i64,
    pub src_h: i64,
    pub target_w: i64,
    pub target_h: i64,
    /// Raw-domain white level, and the number the 0..1 output is divided by.
    pub white_level: f32,
    pub bayer_pattern: i32,
    pub forced_mode: Option<BlockOpsetMode>,
}

impl PipelineAllBlock {
    pub fn new(src_w: i64, src_h: i64, target_w: i64, target_h: i64) -> Self {
        Self {
            id: "pipeline_all".into(),
            prev: None,
            next: None,
            frame_tensor: "PipelineAllBlock/frame".into(),
            input_source: String::new(),
            stages: PipelineStages::PREVIEW,
            src_w: src_w.max(2),
            src_h: src_h.max(2),
            target_w: target_w.max(1),
            target_h: target_h.max(1),
            white_level: 1023.0,
            bayer_pattern: 2,
            forced_mode: None,
        }
    }

    pub fn with_stages(mut self, stages: PipelineStages) -> Self {
        self.stages = stages;
        self
    }

    pub fn with_white_level(mut self, wl: f32) -> Self {
        self.white_level = wl.max(1.0);
        self
    }

    pub fn with_bayer_pattern(mut self, p: i32) -> Self {
        self.bayer_pattern = p & 3;
        self
    }

    pub fn with_opset_mode(mut self, mode: BlockOpsetMode) -> Self {
        self.forced_mode = Some(mode);
        self
    }

    fn ns(&self) -> &'static str {
        "isp_pipeline"
    }

    /// `{srcW, srcH, outW, outH}` — binding `hw` (5).
    fn hw(&self) -> [f32; 4] {
        [
            self.src_w as f32,
            self.src_h as f32,
            self.target_w as f32,
            self.target_h as f32,
        ]
    }
}

impl IspBlock for PipelineAllBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        self.ns().to_string()
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
        Some(&["isp.pipeline"])
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
        // The raw plane, in sensor counts: one channel, full resolution.
        Some(Proto::value_info(
            &self.input_source,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(self.src_h),
                Proto::tensor_dim_value(self.src_w),
            ],
            1, // FLOAT32 — the converter upcasts the declared INT16 input
        ))
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
        if self.opset_mode() != BlockOpsetMode::MnnIspMin {
            // There is no primitive spelling of this op: it *is* the fusion. A
            // chain that wants primitives should not contain this block, so
            // saying so beats emitting something that only looks equivalent.
            log::warn!(
                "pipeline_all: isp.pipeline has no primitive form; \
                 this block only emits the fused op"
            );
            return vec![];
        }
        let ns = self.ns();
        let mut inputs: Vec<String> = vec![self.input_source.clone()];
        for t in [
            "hw", "blc", "wb", "ccm", "gamma", "lift", "pattern", "white", "stages", "sat", "dpc",
            "lsc",
        ] {
            inputs.push(format!("{ns}/{t}"));
        }
        let refs: Vec<&str> = inputs.iter().map(String::as_str).collect();

        vec![Proto::node_with_domain(
            "isp.pipeline",
            "isp",
            &refs,
            &[&self.frame_tensor],
            &[
                // The output rect. Without it the shader falls back to half the
                // input, which is right for a 2x tail and wrong for any other
                // target.
                Proto::attribute_ints("downscale_out", &[self.target_w, self.target_h]),
                // The op's input is planar NCHW and its output has a different
                // W/H. `elementwise` would make VulkanFuse dispatch over the
                // INPUT grid, so it must stay off — `spatial` is what keeps the
                // derived W/H on dims[-1]/dims[-2] without asserting a 1:1 shape.
                Proto::attribute_int("spatial_nchw", 1),
            ],
        )]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        use crate::blocks::VignettingBlock;
        let ns = self.ns();
        let grid = VignettingBlock::gain_map_channels(13, 17, 0.5, 2.0, 4);
        vec![
            (format!("{ns}/hw"), 1, vec![4]),
            (format!("{ns}/blc"), 1, vec![4]),
            (format!("{ns}/wb"), 1, vec![4]),
            (format!("{ns}/ccm"), 1, vec![9]),
            (format!("{ns}/gamma"), 1, vec![1]),
            (format!("{ns}/lift"), 1, vec![1]),
            (format!("{ns}/pattern"), 1, vec![1]),
            (format!("{ns}/white"), 1, vec![1]),
            (format!("{ns}/stages"), 1, vec![1]),
            (format!("{ns}/sat"), 1, vec![1]),
            (format!("{ns}/dpc"), 1, vec![1]),
            (format!("{ns}/lsc"), 1, vec![grid.len() as i64]),
        ]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        use crate::blocks::VignettingBlock;
        let ns = self.ns();
        let f = |v: &[f32]| -> Vec<u8> { v.iter().flat_map(|x| x.to_ne_bytes()).collect() };
        // One grid, four phases: the radial falloff is the same at every Bayer
        // site, so the plane axis only saves recomputing it.
        let grid = VignettingBlock::gain_map_channels(13, 17, 0.5, 2.0, 4);
        vec![
            (format!("{ns}/hw"), f(&self.hw())),
            (format!("{ns}/blc"), f(&[0.0; 4])),
            (format!("{ns}/wb"), f(&[1.0; 4])),
            (
                format!("{ns}/ccm"),
                f(&[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
            ),
            (format!("{ns}/gamma"), f(&[1.0 / 2.2])),
            (format!("{ns}/lift"), f(&[0.0])),
            (format!("{ns}/pattern"), f(&[self.bayer_pattern as f32])),
            (format!("{ns}/white"), f(&[self.white_level])),
            (format!("{ns}/stages"), f(&[self.stages.0 as f32])),
            (format!("{ns}/sat"), f(&[1.0])),
            // A normalized threshold: 0.02 of the white level. Unfed it is 0, so
            // every site differs from its neighbourhood and every pixel would be
            // "corrected" — which averages the whole frame to mush.
            (format!("{ns}/dpc"), f(&[self.white_level * 0.02])),
            (format!("{ns}/lsc"), f(&grid)),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emits_one_fused_node() {
        let mut b = PipelineAllBlock::new(2592, 1940, 648, 485);
        b.set_input_source("raw_frame");
        let nodes = b.nodes();
        assert_eq!(nodes.len(), 1, "isp.pipeline is a single dispatch");
        let s = String::from_utf8_lossy(&nodes[0]);
        assert!(s.contains("isp.pipeline"), "{s}");
        assert!(s.contains("downscale_out"), "{s}");
        assert!(s.contains("spatial_nchw"), "{s}");
    }

    #[test]
    fn output_shape_is_the_target() {
        let b = PipelineAllBlock::new(2592, 1940, 648, 485);
        let vi = b.output_value_info().unwrap();
        // [1,3,485,648] — 3 planes at the requested size, not half the input.
        let s = String::from_utf8_lossy(&vi);
        assert!(s.contains("PipelineAllBlock/frame"), "{s}");
    }

    #[test]
    fn every_binding_has_a_default() {
        let b = PipelineAllBlock::new(2592, 1940, 648, 485);
        let decl: Vec<String> = b.extra_inputs().into_iter().map(|(n, _, _)| n).collect();
        let defaults: Vec<String> = b
            .extra_input_defaults()
            .into_iter()
            .map(|(n, _)| n)
            .collect();
        for name in &decl {
            assert!(
                defaults.contains(name),
                "{name} is declared but never given a value"
            );
        }
    }

    #[test]
    fn gamma_binding_is_the_reciprocal() {
        // isp.pipeline reads binding 9 as `TONE_CURVE/GAMMA_RECIP` and uses it
        // directly as the display exponent, so the value must be 1/γ, not γ.
        let b = PipelineAllBlock::new(2592, 1940, 648, 485);
        let d = b
            .extra_input_defaults()
            .into_iter()
            .find(|(n, _)| n.ends_with("/gamma"))
            .expect("gamma default");
        let recip = f32::from_ne_bytes(d.1.as_chunks::<4>().0[0]);
        assert!((recip - 1.0 / 2.2).abs() < 1e-5, "got {recip}");
        assert!(recip < 1.0, "display gamma reciprocal must be below one");
    }

    #[test]
    fn defaults_are_not_silently_zero() {
        let b = PipelineAllBlock::new(2592, 1940, 648, 485);
        let d = b
            .extra_input_defaults()
            .into_iter()
            .find(|(n, _)| n.ends_with("/stages"))
            .expect("stages default");
        let bits = f32::from_ne_bytes(d.1.as_chunks::<4>().0[0]);
        assert_ne!(bits, 0.0, "an all-zero mask is a bare area demosaic");
        let has = |bit: i32| (bits as i32) & bit != 0;
        assert!(has(PipelineStages::DEMOSAIC));
        assert!(has(PipelineStages::WB));
        assert!(has(PipelineStages::CCM));
        assert!(!has(PipelineStages::NOSCALE), "NOSCALE skips the resample");
    }

    #[test]
    fn dpc_threshold_scales_with_the_white_level() {
        // A normalized threshold must come out of the sensor's own scale: a
        // literal 0.02 against a 1023-count frame would mark every pixel bad.
        let a = PipelineAllBlock::new(2592, 1940, 648, 485).with_white_level(1023.0);
        let b = PipelineAllBlock::new(2592, 1940, 648, 485).with_white_level(4095.0);
        let get = |x: &PipelineAllBlock| -> f32 {
            let d = x
                .extra_input_defaults()
                .into_iter()
                .find(|(n, _)| n.ends_with("/dpc"))
                .unwrap();
            f32::from_ne_bytes(d.1.as_chunks::<4>().0[0])
        };
        assert!(get(&b) > get(&a), "a 12-bit sensor needs a wider threshold");
    }
}
