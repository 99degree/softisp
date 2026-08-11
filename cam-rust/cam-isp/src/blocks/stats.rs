//! ONNX-based statistics blocks for ISP controller feedback.
//!
//! These blocks compute AWB/AE statistics from tone-mapped RGB data
//! and produce tensors that the IspController reads for parameter
//! estimation (white balance gains, exposure, tone curve).
//!
//! Blocks:
//! - `ZoneStatsBlock` — per-zone RGB means `[1, 3, rows, cols]`
//! - `ChannelMeansBlock` — global channel means `[1, 3]`
//! - `ToneStatsBlock` — luma mean, min, max, clipped/shadows `[6]`
//! - `CoarseHistogramBlock` — 16-bin luminance histogram `[16]`

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

// ═══════════════════════════════════════════════════════════════════
// ZoneStatsBlock
// ═══════════════════════════════════════════════════════════════════

/// Zone statistics block — per-zone RGB averages via AveragePool.
///
/// Graph: AveragePool(kernel=H/rows, stride=H/rows) → `[1, 3, rows, cols]`
///
/// Kernel sizes are computed from concrete dims set via `with_concrete_dims()`.
/// The pipeline builder passes the target resolution at model-build time,
/// so kernels adapt to landscape vs portrait orientation automatically.
///
/// Used for multi-illuminant AWB.  48 zones (6×8) is typical for FHD.
pub struct ZoneStatsBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub zone_rows: i64,
    pub zone_cols: i64,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
}

impl ZoneStatsBlock {
    pub fn new(rows: i64, cols: i64) -> Self {
        Self {
            id: "zone_stats".into(),
            prev: None,
            next: None,
            frame_tensor: "ZoneStatsBlock/frame".into(),
            input_source: String::new(),
            zone_rows: rows,
            zone_cols: cols,
            concrete_h: None,
            concrete_w: None,
        }
    }
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }
    /// Kernel height = input_h / zone_rows  (e.g. 1080/6 = 180 @ FHD landscape)
    fn kernel_h(&self) -> i64 {
        self.concrete_h
            .map_or(1, |h| ((h + self.zone_rows - 1) / self.zone_rows).max(1))
    }
    /// Kernel width  = ceil(input_w / zone_cols).
    /// Uses ceil division so the full frame is covered even when resolution
    /// doesn't divide evenly (e.g. 1900/8 → 238, not 237).
    fn kernel_w(&self) -> i64 {
        self.concrete_w
            .map_or(1, |w| ((w + self.zone_cols - 1) / self.zone_cols).max(1))
    }
}

impl IspBlock for ZoneStatsBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "ZoneStatsBlock".into()
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
        let dims = vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(3),
            Proto::tensor_dim_param("H"),
            Proto::tensor_dim_param("W"),
        ];
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let dims = vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(3),
            Proto::tensor_dim_value(self.zone_rows),
            Proto::tensor_dim_value(self.zone_cols),
        ];
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        vec![Proto::node(
            "AveragePool",
            &[&self.input_source],
            &[&self.frame_tensor],
            &[
                Proto::attribute_ints("kernel_shape", &[self.kernel_h(), self.kernel_w()]),
                Proto::attribute_ints("strides", &[self.kernel_h(), self.kernel_w()]),
                Proto::attribute_int("ceil_mode", 1),
            ],
        )]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
}

// ═══════════════════════════════════════════════════════════════════
// ChannelMeansBlock
// ═══════════════════════════════════════════════════════════════════

/// Global channel means — ReduceMean over spatial dims.
/// Output: `[1, 3]` flat tensor of (mean_R, mean_G, mean_B).
pub struct ChannelMeansBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
}

impl Default for ChannelMeansBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl ChannelMeansBlock {
    pub fn new() -> Self {
        Self {
            id: "channel_means".into(),
            prev: None,
            next: None,
            frame_tensor: "ChannelMeansBlock/frame".into(),
            input_source: String::new(),
            concrete_h: None,
            concrete_w: None,
        }
    }
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }
}

impl IspBlock for ChannelMeansBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "ChannelMeansBlock".into()
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
        let dims = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ]
        };
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let dims = vec![Proto::tensor_dim_value(1), Proto::tensor_dim_value(3)];
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        vec![Proto::node(
            "ReduceMean",
            &[&self.input_source],
            &[&self.frame_tensor],
            &[
                Proto::attribute_ints("axes", &[2, 3]),
                Proto::attribute_int("keepdims", 0),
            ],
        )]
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
}

// ═══════════════════════════════════════════════════════════════════
// ToneStatsBlock
// ═══════════════════════════════════════════════════════════════════

/// Tone statistics block — luma mean, min, max, clipped, shadows, count.
///
/// Output: `[6]` flat tensor:
///   `[0]` mean luma
///   `[1]` min luma
///   `[2]` max luma
///   `[3]` clipped fraction (pixels > 0.95)
///   `[4]` shadow fraction (pixels < 0.05)
///   `[5]` total pixel count
///
/// Used by AE for exposure metering and tone mapping.
/// All statistics are derived from luminance: L = 0.299R + 0.587G + 0.114B
pub struct ToneStatsBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
}

impl Default for ToneStatsBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl ToneStatsBlock {
    pub fn new() -> Self {
        Self {
            id: "tone_stats".into(),
            prev: None,
            next: None,
            frame_tensor: "ToneStatsBlock/frame".into(),
            input_source: String::new(),
            concrete_h: None,
            concrete_w: None,
        }
    }
}

impl IspBlock for ToneStatsBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "ToneStatsBlock".into()
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
        let dims = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ]
        };
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let dims = vec![Proto::tensor_dim_value(6)];
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();

        // Luma weights as a Conv1×1: [1, 3, 1, 1]
        // L = 0.299R + 0.587G + 0.114B
        vec![
            // ── Compute luma via Conv1×1 ──
            // Weight shape [1, 3, 1, 1], single output channel
            Proto::node(
                "Conv",
                &[
                    &self.input_source,
                    &format!("{}/luma_w", ns),
                    &format!("{}/luma_b", ns),
                ],
                &[&format!("{}/luma", ns)],
                &[
                    Proto::attribute_ints("kernel_shape", &[1, 1]),
                    Proto::attribute_ints("strides", &[1, 1]),
                    Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                ],
            ),
            // ── Mean luma ──
            Proto::node(
                "ReduceMean",
                &[&format!("{}/luma", ns)],
                &[&format!("{}/mean", ns)],
                &[
                    Proto::attribute_ints("axes", &[2, 3]),
                    Proto::attribute_int("keepdims", 0),
                ],
            ),
            // ── Min luma ──
            Proto::node(
                "ReduceMin",
                &[&format!("{}/luma", ns)],
                &[&format!("{}/min", ns)],
                &[
                    Proto::attribute_ints("axes", &[2, 3]),
                    Proto::attribute_int("keepdims", 0),
                ],
            ),
            // ── Max luma ──
            Proto::node(
                "ReduceMax",
                &[&format!("{}/luma", ns)],
                &[&format!("{}/max", ns)],
                &[
                    Proto::attribute_ints("axes", &[2, 3]),
                    Proto::attribute_int("keepdims", 0),
                ],
            ),
            // ── Clipped mask (luma > 0.95) ──
            Proto::node(
                "Greater",
                &[&format!("{}/luma", ns), &format!("{}/clip_thresh", ns)],
                &[&format!("{}/clip_mask", ns)],
                &[],
            ),
            Proto::node(
                "ReduceSum",
                &[&format!("{}/clip_mask", ns)],
                &[&format!("{}/clip_cnt", ns)],
                &[
                    Proto::attribute_ints("axes", &[2, 3]),
                    Proto::attribute_int("keepdims", 0),
                ],
            ),
            // ── Shadow mask (luma < 0.05) ──
            Proto::node(
                "Less",
                &[&format!("{}/luma", ns), &format!("{}/shadow_thresh", ns)],
                &[&format!("{}/shadow_mask", ns)],
                &[],
            ),
            Proto::node(
                "ReduceSum",
                &[&format!("{}/shadow_mask", ns)],
                &[&format!("{}/shadow_cnt", ns)],
                &[
                    Proto::attribute_ints("axes", &[2, 3]),
                    Proto::attribute_int("keepdims", 0),
                ],
            ),
            // ── Total pixel count ──
            Proto::node(
                "Size",
                &[&format!("{}/luma", ns)],
                &[&format!("{}/total_px", ns)],
                &[],
            ),
            // ── Concat into [6] output ──
            Proto::node(
                "Concat",
                &[
                    &format!("{}/mean", ns),
                    &format!("{}/min", ns),
                    &format!("{}/max", ns),
                    &format!("{}/clip_cnt", ns),
                    &format!("{}/shadow_cnt", ns),
                    &format!("{}/total_px", ns),
                ],
                &[&self.frame_tensor],
                &[Proto::attribute_int("axis", 1)],
            ),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![
            // Luma weights [1, 4, 1, 1]: the stats blocks read the pre-demosaic
            // 4-quadrant Bayer tensor (aux_hook_src); luma ≈ mean of the quads.
            Proto::tensor_proto_float(
                &format!("{}/luma_w", ns),
                &[1, 4, 1, 1],
                &[0.25, 0.25, 0.25, 0.25],
            ),
            Proto::tensor_proto_float(&format!("{}/luma_b", ns), &[1], &[0.0]),
            Proto::tensor_proto_float_scalar(&format!("{}/clip_thresh", ns), 0.95),
            Proto::tensor_proto_float_scalar(&format!("{}/shadow_thresh", ns), 0.05),
        ]
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
}

// ═══════════════════════════════════════════════════════════════════
// CoarseHistogramBlock
// ═══════════════════════════════════════════════════════════════════

/// Coarse luminance histogram — 16 evenly-spaced bins.
///
/// Output: `[1, 16]` flat tensor of bin counts (normalised to fraction of pixels).
///
/// Uses `Less` + `GreaterOrEqual` + `ReduceSum` per bin — 16 × 3 ≈ 48 ops
/// over the full H×W tensor.  At FHD this is ~6ms on CPU, <1ms on GPU.
///
/// A full 256-bin histogram is impractical in ONNX (OneHot creates
/// 256×H×W intermediate = 2GB at FHD).  Use this coarse version for
/// AE clipping detection and use the Rust software path for full 256-bin
/// when higher resolution is needed.
pub struct CoarseHistogramBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub num_bins: i64,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
}

impl CoarseHistogramBlock {
    pub fn new(num_bins: i64) -> Self {
        Self {
            id: "histogram".into(),
            prev: None,
            next: None,
            frame_tensor: "CoarseHistogramBlock/frame".into(),
            input_source: String::new(),
            num_bins: num_bins.clamp(2, 64),
            concrete_h: None,
            concrete_w: None,
        }
    }
}

impl IspBlock for CoarseHistogramBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "CoarseHistogramBlock".into()
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
        let dims = vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(3),
            Proto::tensor_dim_param("H"),
            Proto::tensor_dim_param("W"),
        ];
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let dims = vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(self.num_bins),
        ];
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // ── Compute luma via Conv1×1 ──
        nodes.push(Proto::node(
            "Conv",
            &[
                &self.input_source,
                &format!("{}/luma_w", ns),
                &format!("{}/luma_b", ns),
            ],
            &[&format!("{}/luma", ns)],
            &[
                Proto::attribute_ints("kernel_shape", &[1, 1]),
                Proto::attribute_ints("strides", &[1, 1]),
                Proto::attribute_ints("pads", &[0, 0, 0, 0]),
            ],
        ));

        // ── For each bin: bin_value < luma <= bin_value_next ──
        // Bin edges: [0, 1/num_bins, 2/num_bins, ..., 1]
        let bin_w = 1.0 / self.num_bins as f32;
        for i in 0..self.num_bins {
            let _lo = i as f32 * bin_w;
            let _hi = (i + 1) as f32 * bin_w;
            let bin_name = format!("{}/b{}", ns, i);

            if i == 0 {
                // First bin: luma <= hi  →  Less(luma, hi)
                nodes.push(Proto::node(
                    "Less",
                    &[&format!("{}/luma", ns), &format!("{}/e{}", ns, i + 1)],
                    &[&bin_name],
                    &[],
                ));
            } else if i == self.num_bins - 1 {
                // Last bin: luma > lo  →  Greater(luma, lo)
                nodes.push(Proto::node(
                    "Greater",
                    &[&format!("{}/luma", ns), &format!("{}/e{}", ns, i)],
                    &[&bin_name],
                    &[],
                ));
            } else {
                // Inner bins: luma > lo AND luma <= hi
                let low = format!("{}/b{}_lo", ns, i);
                let high = format!("{}/b{}_hi", ns, i);
                nodes.push(Proto::node(
                    "Greater",
                    &[&format!("{}/luma", ns), &format!("{}/e{}", ns, i)],
                    &[&low],
                    &[],
                ));
                nodes.push(Proto::node(
                    "Less",
                    &[&format!("{}/luma", ns), &format!("{}/e{}", ns, i + 1)],
                    &[&high],
                    &[],
                ));
                nodes.push(Proto::node("Mul", &[&low, &high], &[&bin_name], &[]));
            }
        }

        // ── Reduce each bin mask to a scalar count ──
        let mut bin_counts: Vec<String> = Vec::with_capacity(self.num_bins as usize);
        for i in 0..self.num_bins {
            let cnt = format!("{}/c{}", ns, i);
            bin_counts.push(cnt.clone());
            nodes.push(Proto::node(
                "ReduceSum",
                &[&format!("{}/b{}", ns, i)],
                &[&cnt],
                &[
                    Proto::attribute_ints("axes", &[2, 3]),
                    Proto::attribute_int("keepdims", 0),
                ],
            ));
        }

        // ── Concat all bin counts → [1, num_bins] ──
        let axis_name = format!("{}/axis", ns);
        let mut inputs: Vec<&str> = bin_counts.iter().map(|s| s.as_str()).collect();
        inputs.push(&axis_name);
        nodes.push(Proto::node(
            "Concat",
            &inputs,
            &[&self.frame_tensor],
            &[Proto::attribute_int("axis", 1)],
        ));

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut inits = vec![
            Proto::tensor_proto_float(
                &format!("{}/luma_w", ns),
                &[1, 3, 1, 1],
                &[0.299, 0.587, 0.114],
            ),
            Proto::tensor_proto_float(&format!("{}/luma_b", ns), &[1], &[0.0]),
        ];

        // Bin edges: [0, 1/N, 2/N, ..., 1]
        for i in 0..=self.num_bins {
            let edge_val = i as f32 / self.num_bins as f32;
            inits.push(Proto::tensor_proto_float_scalar(
                &format!("{}/e{}", ns, i),
                edge_val,
            ));
        }

        // Concat axis
        inits.push(Proto::tensor_proto_int64(&format!("{}/axis", ns), &[1]));

        inits
    }
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
}

// ═══════════════════════════════════════════════════════════════════
// CalibrationBlock — quad-level Bayer stats for AF/learner
// ═══════════════════════════════════════════════════════════════════

/// Calibration statistics block — quad-level Bayer stats for AF + learner.
///
/// Input:  `[1, 4, H, W]`  Bayer quadrants (from aux_hook_src or its downscale)
/// Output: `[1, 24]`       calibration stats matching CalibrationStats layout:
///   `[0:4]`   quad_means     — mean of each quad channel
///   `[4:8]`   quad_vars      — variance per quad (used by AF focus metric)
///   `[8:12]`  quad_mins      — min per quad
///   `[12:16]` quad_maxs      — max per quad
///   `[16:20]` quad_ranges    — (max-min)/(max+ε) per quad
///   `[20]`    frame_lum      — mean of 4 quad means
///   `[21]`    frame_noise    — mean of 4 quad vars
///   `[22]`    frame_min      — min across all quads
///   `[23]`    frame_max      — max across all quads
///
/// Graph: var = E`[X²]` - E`[X]`² (linearity of variance)
pub struct CalibrationBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
}

impl Default for CalibrationBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl CalibrationBlock {
    pub fn new() -> Self {
        Self {
            id: "calibration".into(),
            prev: None,
            next: None,
            frame_tensor: "CalibrationBlock/frame".into(),
            input_source: String::new(),
            concrete_h: None,
            concrete_w: None,
        }
    }
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }
}

impl IspBlock for CalibrationBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "CalibrationBlock".into()
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
        let dims = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(4),
                Proto::tensor_dim_param("H"),
                Proto::tensor_dim_param("W"),
            ]
        };
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let dims = vec![Proto::tensor_dim_value(1), Proto::tensor_dim_value(24)];
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        use crate::onnx::proto::Proto as P;
        let x = &self.input_source;
        let ns = |name: &str| format!("CalibrationBlock/{}", name);

        let x2 = ns("x2"); // X²
        let e_x = ns("e_x"); // E[X]
        let e_x2 = ns("e_x2"); // E[X²]
        let e_x_2 = ns("e_x_2"); // E[X]²
        let var = ns("var"); // Var[X]
        let min = ns("min"); // per-quad min
        let max = ns("max"); // per-quad max
        let r_pre = ns("r_pre"); // max - min
        let r_max = ns("r_max"); // max + ε
        let range = ns("range"); // (max-min)/(max+ε)
        let lum = ns("lum"); // frame luminance
        let noise = ns("noise"); // frame noise
        let fmin = ns("fmin"); // frame min
        let fmax = ns("fmax"); // frame max
        let c_all = ns("c_all");
        let out = &self.frame_tensor;

        vec![
            // 1. X² = Mul(x, x)
            P::node("Mul", &[x, x], &[&x2], &[]),
            // 2. E[X] = ReduceMean(x, axes=[2,3])
            P::node(
                "ReduceMean",
                &[x],
                &[&e_x],
                &[
                    P::attribute_ints("axes", &[2, 3]),
                    P::attribute_int("keepdims", 1),
                ],
            ),
            // 3. E[X²] = ReduceMean(x², axes=[2,3])
            P::node(
                "ReduceMean",
                &[&x2],
                &[&e_x2],
                &[
                    P::attribute_ints("axes", &[2, 3]),
                    P::attribute_int("keepdims", 1),
                ],
            ),
            // 4. E[X]² = Mul(E[X], E[X])
            P::node("Mul", &[&e_x, &e_x], &[&e_x_2], &[]),
            // 5. Var = E[X²] - E[X]²
            P::node("Sub", &[&e_x2, &e_x_2], &[&var], &[]),
            // 6. Min = ReduceMin(x, axes=[2,3])
            P::node(
                "ReduceMin",
                &[x],
                &[&min],
                &[
                    P::attribute_ints("axes", &[2, 3]),
                    P::attribute_int("keepdims", 1),
                ],
            ),
            // 7. Max = ReduceMax(x, axes=[2,3])
            P::node(
                "ReduceMax",
                &[x],
                &[&max],
                &[
                    P::attribute_ints("axes", &[2, 3]),
                    P::attribute_int("keepdims", 1),
                ],
            ),
            // 8. Range_pre = Max - Min
            P::node("Sub", &[&max, &min], &[&r_pre], &[]),
            // 9. Max_safe = Max + ε (avoid div-by-zero)
            P::node("Add", &[&max, "CalibrationBlock/eps"], &[&r_max], &[]),
            // 10. Range = Range_pre / Max_safe
            P::node("Div", &[&r_pre, &r_max], &[&range], &[]),
            // 11-14. Scalar frame stats (ReduceMean/Min/Max on axis=1 of [1,4,1,1])
            P::node(
                "ReduceMean",
                &[&e_x],
                &[&lum],
                &[
                    P::attribute_ints("axes", &[1]),
                    P::attribute_int("keepdims", 1),
                ],
            ),
            P::node(
                "ReduceMean",
                &[&var],
                &[&noise],
                &[
                    P::attribute_ints("axes", &[1]),
                    P::attribute_int("keepdims", 1),
                ],
            ),
            P::node(
                "ReduceMin",
                &[&min],
                &[&fmin],
                &[
                    P::attribute_ints("axes", &[1]),
                    P::attribute_int("keepdims", 1),
                ],
            ),
            P::node(
                "ReduceMax",
                &[&max],
                &[&fmax],
                &[
                    P::attribute_ints("axes", &[1]),
                    P::attribute_int("keepdims", 1),
                ],
            ),
            // 15. Concat all 9 tensors along axis=1
            //   [1,4,1,1] ×5 + [1,1,1,1] ×4 → [1,24,1,1]
            P::node(
                "Concat",
                &[&e_x, &var, &min, &max, &range, &lum, &noise, &fmin, &fmax],
                &[&c_all],
                &[P::attribute_int("axis", 1)],
            ),
            // 16. Reshape [1,24,1,1] → [1,24]
            P::node("Reshape", &[&c_all, "CalibrationBlock/shape"], &[out], &[]),
        ]
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![
            Proto::tensor_proto_float_scalar("CalibrationBlock/eps", 1e-6f32),
            Proto::tensor_proto_int64("CalibrationBlock/shape", &[1, 24]),
        ]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blocks::DemosaicCcmBlock;
    use crate::pipeline::GraphComposer;

    // ── ZoneStatsBlock ──
    #[test]
    fn test_zone_kernels_exact() {
        // 1920×1080, 6×8 zones → divides evenly
        let b = ZoneStatsBlock::new(6, 8).with_concrete_dims(1080, 1920);
        assert_eq!(b.kernel_h(), 180, "1080/6 = 180");
        assert_eq!(b.kernel_w(), 240, "1920/8 = 240");
    }

    #[test]
    fn test_zone_kernels_remainder() {
        // 1900×1088, 6×8 zones → non-divisible, tests ceil
        let b = ZoneStatsBlock::new(6, 8).with_concrete_dims(1088, 1900);
        assert_eq!(b.kernel_h(), 182, "ceil(1088/6) = ceil(181.3) = 182");
        assert_eq!(b.kernel_w(), 238, "ceil(1900/8) = ceil(237.5) = 238");
    }

    #[test]
    fn test_zone_kernels_portrait() {
        // Portrait 1080×1920, 6×8 zones
        let b = ZoneStatsBlock::new(6, 8).with_concrete_dims(1920, 1080);
        assert_eq!(b.kernel_h(), 320, "ceil(1920/6) = 320");
        assert_eq!(b.kernel_w(), 135, "ceil(1080/8) = 135");
    }

    #[test]
    fn test_zone_kernels_odd() {
        // Unusual 1001×1001, 4×4 zones
        let b = ZoneStatsBlock::new(4, 4).with_concrete_dims(1001, 1001);
        assert_eq!(b.kernel_h(), 251, "ceil(1001/4) = 251");
        assert_eq!(b.kernel_w(), 251, "ceil(1001/4) = 251");
    }

    #[test]
    fn test_zone_stats_pipeline() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(
                crate::blocks::RawInputBlock::new()
                    .with_elem_type(1)
                    .with_concrete_dims(48, 64),
            ),
            Box::new(crate::blocks::NormalizeBlock::new()),
            Box::new(crate::blocks::CfaBlock::new().with_concrete_dims(48, 64)),
            Box::new(crate::blocks::BlcBlock::new()),
            Box::new(crate::blocks::BayerWbBlock::new()),
            Box::new(DemosaicCcmBlock::new(2).with_concrete_dims(24, 32)),
            Box::new(ZoneStatsBlock::new(2, 2).with_concrete_dims(24, 32)),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        assert!(GraphComposer::compose_from_vec(&refs, &[], 16).is_ok());
    }

    // ── ChannelMeansBlock ──
    #[test]
    fn test_channel_means_nodes() {
        assert_eq!(ChannelMeansBlock::new().nodes().len(), 1);
    }

    #[test]
    fn test_channel_means_pipeline() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(
                crate::blocks::RawInputBlock::new()
                    .with_elem_type(1)
                    .with_concrete_dims(48, 64),
            ),
            Box::new(crate::blocks::NormalizeBlock::new()),
            Box::new(crate::blocks::CfaBlock::new().with_concrete_dims(48, 64)),
            Box::new(crate::blocks::BlcBlock::new()),
            Box::new(crate::blocks::DemosaicCcmBlock::new(2).with_concrete_dims(24, 32)),
            Box::new(ChannelMeansBlock::new()),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        assert!(GraphComposer::compose_from_vec(&refs, &[], 16).is_ok());
    }

    // ── ToneStatsBlock ──
    #[test]
    fn test_tone_stats_nodes() {
        let nodes = ToneStatsBlock::new().nodes();
        assert_eq!(
            nodes.len(),
            10,
            "ToneStats: luma+mean+min+max+clip+shadow+px+concat=10"
        );
    }

    #[test]
    fn test_tone_stats_pipeline() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(
                crate::blocks::RawInputBlock::new()
                    .with_elem_type(1)
                    .with_concrete_dims(48, 64),
            ),
            Box::new(crate::blocks::NormalizeBlock::new()),
            Box::new(crate::blocks::DemosaicCcmBlock::new(2).with_concrete_dims(24, 32)),
            Box::new(ToneStatsBlock::new()),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        assert!(GraphComposer::compose_from_vec(&refs, &[], 16).is_ok());
    }

    // ── CoarseHistogramBlock ──
    #[test]
    fn test_histogram_nodes() {
        let b = CoarseHistogramBlock::new(8);
        // 1 (luma) + 2 (first+last bins) + 3*(n-2) (inner) + n (reduce) + 1 (concat) = 4n - 2
        assert_eq!(b.nodes().len(), (4 * b.num_bins - 2) as usize);
    }

    #[test]
    fn test_histogram_initializers() {
        let b = CoarseHistogramBlock::new(16);
        // luma_w + luma_b + (num_bins+1) edges + axis = 4 + 16
        assert_eq!(b.initializers().len(), 2 + (b.num_bins + 1) as usize + 1);
    }

    #[test]
    fn test_histogram_pipeline() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(
                crate::blocks::RawInputBlock::new()
                    .with_elem_type(1)
                    .with_concrete_dims(8, 8),
            ),
            Box::new(crate::blocks::NormalizeBlock::new()),
            Box::new(crate::blocks::DemosaicCcmBlock::new(2).with_concrete_dims(4, 4)),
            Box::new(CoarseHistogramBlock::new(4)),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        assert!(GraphComposer::compose_from_vec(&refs, &[], 16).is_ok());
    }
}
