//! Pipeline profiles — feature complexity presets for ISP block chain construction.
//!
//! Each profile defines which blocks are enabled, trading off image quality vs
//! processing cost. Ported from `com.camcore.isp.pipeline.PipelineProfile` (Java).

use crate::engine::OutputFormat;

/// Orientation transform constants (32-bit integer for fast comparison).
pub const ROTATE_NONE: i32 = 0; // No transform
pub const ROTATE_ROT90: i32 = 1; // 90° clockwise
pub const ROTATE_ROT180: i32 = 2; // 180°
pub const ROTATE_ROT270: i32 = 3; // 90° counter-clockwise
pub const ROTATE_HFLIP: i32 = 4; // Horizontal mirror
pub const ROTATE_VFLIP: i32 = 5; // Vertical mirror

/// Demosaic quality selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DemosaicQuality {
    /// Fast bilinear demosaic.
    #[default]
    Standard,
    /// Higher-quality gradient-based (Malvar 2004).
    HqLinear,
    /// Edge-aware demosaic with false color suppression.
    Edge,
}

/// Feature complexity level — gates controller features.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum PipelineLevel {
    Lite = 0,
    Medium = 1,
    Heavy = 2,
    Pro = 3,
}

/// Pipeline profile defining which ISP blocks to enable.
///
/// Each profile has a fixed set of feature flags and can build a block chain
/// via `build_chain()`.
#[derive(Debug, Clone, Copy)]
pub struct PipelineProfile {
    /// Human-readable label.
    pub label: &'static str,
    /// Feature level for controller gating.
    pub level: PipelineLevel,
    /// Input type: "raw" for Bayer RAW, "yuv" for YUV formats.
    pub input_type: &'static str,
    /// Enable packed INT32 input with UnpackBlock for true zero-copy.
    /// When true (default), RawInputBlock uses INT32 at half-width and
    /// UnpackBlock is injected after it. When false, uses legacy FLOAT path.
    pub use_unpack: bool,
    /// Force the pure 16-bit raw input convention: `[1,1,H,W]` INT16 via
    /// `RawInput16Block`, instead of the packed 2×int16-as-int32 tensor
    /// (`[1,2,H,W/2]`, `RawInputPackedBlock`). The MNN converter upcasts the
    /// INT16 input to FLOAT32; the engine converts the u16 buffer to f32
    /// (RawFloat mode). Takes precedence over `use_unpack`. Default: false.
    pub force_input16: bool,
    /// Enable false color suppression (FCS).
    pub use_fcs: bool,
    /// Enable local contrast enhancement (LDCI).
    pub use_ldci: bool,
    /// Enable edge enhancement / unsharp mask (EE).
    pub use_ee: bool,
    /// Enable defective pixel correction (hot pixel removal).
    pub use_bad_pixel: bool,
    /// Demosaic quality.
    pub demosaic_quality: DemosaicQuality,
    /// Enable local contrast enhancement.
    pub use_local_contrast: bool,
    /// Enable unsharp mask edge enhancement.
    pub use_unsharp: bool,
    /// Enable lens shading correction.
    pub use_lsc: bool,
    /// Lens shading strength (0.0 = off, 1.0 = full correction).
    ///
    /// The gain map is computed from this at frame time; it is not a
    /// per-sensor calibration table, so a device with a real LSC grid would
    /// override the map rather than this scalar.
    pub lsc_strength: f32,
    /// Enable image warping (EIS/deshake).
    pub use_warp: bool,
    /// Enable HDR merge.
    pub use_hdr: bool,
    /// Orientation transform: 0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip.
    pub rotate_mode: i32,
    /// Enable per-zone RGB means (AveragePool) for multi-illuminant AWB.
    pub use_zone_stats: bool,
    /// Enable global channel means (ReduceMean) for grey-world AWB.
    pub use_channel_means: bool,
    /// Enable tone stats (luma mean/min/max, clipped/shadows) for AE.
    pub use_tone_stats: bool,
    /// Enable coarse luminance histogram (16 bins) for AE metering.
    pub use_histogram: bool,
    /// Maximum pixel dimension for stats processing.
    /// If the input resolution exceeds this, the stats blocks are preceded
    /// by a ResizeBlock that downsamples to ≤ this dimension (preserving
    /// aspect ratio).  Set to 0 to always process at full resolution.
    ///
    /// Example: for 4K input (2160×3840), setting this to 1080 downsamples
    /// stats to 540×960 — 16× fewer pixels, 16× faster stats.
    pub stats_downscale_max: u32,
    /// Maximum pixel dimension for the main pipeline processing.
    /// If the sensor resolution exceeds this, a ResizeBlock is inserted
    /// after CFA/BLC to downsample before the expensive blocks
    /// (demosaic, CCM, tone, cosmetic, display).
    ///
    /// Set to 0 to always process at full resolution.
    /// Example: 4K sensor (3840) → set to 1920 for FHD-equivalent processing.
    pub pipeline_downscale_target: u32,
    /// Downscale to `(w, h)` immediately **after** `CcmBlock`, before tone and
    /// the cosmetic stages. `(0, 0)` disables it.
    ///
    /// This is the cheap place to shrink the frame. Everything before CCM runs
    /// on Bayer mosaics (¼ the elements of RGB at the same pixel count), so
    /// demosaic input stays full resolution and only the RGB tail — tone, EE,
    /// bilateral, sharpen, display — is processed at the smaller size. The
    /// display tensor is the largest allocation in the graph, so this is what
    /// decides whether the pipeline fits on the GPU.
    pub post_ccm_downscale: (u32, u32),
    /// EIS margin fraction (0.05 = 5%). Passed to AdaptiveDownscaleBlock for
    /// reserving edge pixels for EIS/deshake warp shifts.
    pub eis_margin: f64,
    /// Enable bilateral filter (edge-preserving noise reduction).
    pub use_bilateral: bool,
    /// Enable saturation control.
    pub use_saturation: bool,
    /// Enable vignetting correction.
    pub use_vignetting: bool,
    /// Enable color space conversion (HSV/LAB).
    pub use_colorspace: bool,
    /// Enable gamma correction.
    pub use_gamma: bool,
    /// Enable sharpening (unsharp mask).
    pub use_sharpen: bool,
    /// Enable wavelet denoising.
    pub use_wavelet_denoise: bool,
    /// Enable auto contrast adjustment.
    pub use_auto_contrast: bool,
    /// Enable normalization (zero-mean, unit-variance).
    pub use_normalize: bool,
    /// Fold the input normalisation into the *display* stage instead of
    /// dividing the Bayer frame up front.
    ///
    /// The ISP chain works on `[0,1]` floats: `BlcBlock`, `BayerWbBlock`,
    /// `CcmBlock`, `ToneBlock` and the display Conv all clamp or scale against
    /// `<ns>/one`, which is 1.0. Inserting `NormalizeBlock` (a Cast + Div by
    /// `sensor_max`) is what establishes that range.
    ///
    /// With this flag the divide is skipped at the input and every block before
    /// the display keeps working in sensor counts; the display Conv weights and
    /// the tone contrast are divided by the same factor so the final image is
    /// identical. That removes one full-frame pass over the Bayer data, but it
    /// assumes no `Clip` in the chain saturates below the sensor's white level.
    /// Real sensor data reaches ~0.97 full scale, which the 1.0 clips will cut,
    /// so this is **off by default**.
    pub normalize_at_display: bool,
    /// Enable tiled rendering for high-resolution output (e.g., 4K→4K).
    /// When true, the output is split into tiles processed independently.
    pub use_tiled_rendering: bool,
    /// Number of tiles horizontally (e.g., 2 for 4K→4K with 2×2 tiles).
    pub tile_count_x: u32,
    /// Number of tiles vertically (e.g., 2 for 4K→4K with 2×2 tiles).
    pub tile_count_y: u32,
    /// Overlap pixels between tiles for convolution boundary handling.
    /// Set to kernel_radius for convolution-based blocks (e.g., 2 for 5×5).
    pub tile_overlap: u32,
    /// Output pixel format.
    pub output_format: OutputFormat,
    /// Opset mode for ONNX emission: 0=Primitive, 1=Custom, 2=MnnIspMin.
    /// Default is 0 (Primitive) for maximum compatibility.
    pub opset_mode: i32,
    /// Which stages may emit a fused `isp.*` op, even when `opset_mode` says
    /// MnnIspMin.
    ///
    /// Fusion is all-or-nothing per block otherwise: `opset_mode = 2` switches
    /// every fusion-capable stage at once, so a single bad shader takes the
    /// whole preview with it. This lets a stage be turned on after it has been
    /// verified, one at a time, and keeps the rest on the primitive path.
    pub fuse_stages: FuseStages,
}

/// Per-stage permission for fused `isp.*` emission.
///
/// Every stage defaults to **off**. A stage is switched on only after its fused
/// form has been measured against the primitive one it replaces — see
/// `probe_fusion_stepwise`, which does that comparison one block at a time.
///
/// The gate is deliberately separate from `opset_mode`: that field says what
/// the backend *can* run, this one says what this profile has decided to trust.
/// Emit a `Transpose(perm=[0,1,2,3])` bridge between every pair of blocks.
///
/// A bridge is a semantic no-op and a full-frame GPU read + write. They exist so
/// every pipeline position has a named node (`id_{prev}_{next}`) in a graph dump;
/// on the FUSED chain they are 19 of 33 nodes. Off by default — turn it on when
/// reading a converted model, not when running one.
pub const BRIDGE_IDENTITIES: bool = false;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FuseStages {
    /// The RAW head as one `isp.unpack_blc_normal_cfa`.
    ///
    /// Replaces `RawInputPacked + Unpack + Normalize + Blc + CfaPacked` — five
    /// blocks that between them cast, divide, clip and slice the full-resolution
    /// frame — with a single dispatch that reads the packed sensor buffer and
    /// emits the quad directly.
    pub unpack_blc_normal_cfa: bool,
    /// The RAW head as one `isp.unpack16_blc_normal_cfa`.
    ///
    /// Replaces `RawInput16 + UnpackCfa + Normalize + Blc` — four blocks that
    /// convert INT16 to float, extract the CFA quad, divide by sensor_max, and
    /// subtract BLC — with a single dispatch that reads the INT16 sensor buffer
    /// directly.
    pub unpack16_blc_normal_cfa: bool,
    /// The RAW head as one `isp.unpack32_blc_normal_cfa`.
    ///
    /// Same as `unpack16_blc_normal_cfa` but reads FLOAT32 sensor samples
    /// (engine `RawFloat` mode) instead of INT16.
    pub unpack32_blc_normal_cfa: bool,
    /// `DemosaicBlock` + `CcmBlock` as one `isp.demosaic_ccm`.
    pub demosaic_ccm: bool,
    /// Fold `BayerWbBlock` into the `isp.demosaic_ccm` weights.
    ///
    /// The op already multiplies the CFA quad by a 3x4 matrix, and white balance
    /// is a per-plane scalar — so it is a scale on that matrix's columns, not
    /// another stage. Only meaningful together with `demosaic_ccm`; on its own it
    /// would drop the gains on the floor.
    pub wb_into_demosaic: bool,
    /// `CcmBlock` as `isp.ccm`.
    pub ccm: bool,
    /// `EeBlock` as `isp.ee`.
    pub ee: bool,
    /// `LdciBlock` as `isp.ldci`.
    pub ldci: bool,
    /// `DisplayBlock` as `isp.display`.
    pub display: bool,
    /// The post-CCM RGB tail downscale as `isp.downscale`.
    pub tail_downscale: bool,
    /// The WHOLE RAW chain as one `isp.pipeline` dispatch.
    ///
    /// Not a peer of the stages above: when this is on, everything from the raw
    /// sensor plane to the display tensor is one node, so the other bits describe
    /// *inside* it and are read as the op's stage mask rather than as separate
    /// dispatches. It is off for every profile that emits a per-stage chain.
    pub whole_pipeline: bool,
}

impl FuseStages {
    /// Every stage that has a verified fused form.
    ///
    /// Each entry was measured against its primitive form before being added
    /// here; see `probe_enable_fused_stages` and `probe_fused_head`.
    pub const ALL: Self = Self {
        unpack_blc_normal_cfa: true,
        unpack16_blc_normal_cfa: true,
        unpack32_blc_normal_cfa: true,
        demosaic_ccm: true,
        ccm: true,
        ee: true,
        ldci: true,
        display: true,
        // Verified against `AdaptiveDownscaleBlock` (Resize): mean|Δ| 0.00037,
        // peak|Δ| 0.00037 on a 1296x970 -> 648x485 tail.
        tail_downscale: true,
        whole_pipeline: false,
        // Verified pixel-identical against the separate block on a patterned
        // frame: mean|Δ| 0.000000, peak|Δ| 0.000000.
        wb_into_demosaic: true,
    };

    /// Nothing fused — every stage emits its primitive form.
    pub const NONE: Self = Self {
        unpack_blc_normal_cfa: false,
        unpack16_blc_normal_cfa: false,
        unpack32_blc_normal_cfa: false,
        demosaic_ccm: false,
        ccm: false,
        ee: false,
        ldci: false,
        display: false,
        tail_downscale: false,
        whole_pipeline: false,
        wb_into_demosaic: false,
    };

    /// Whether `stage` is allowed, given the profile-wide `opset_mode`.
    ///
    /// A stage only fuses when the profile asked for MnnIspMin *and* the stage
    /// is on the allowlist. Asking for MnnIspMin with an empty allowlist is a
    /// no-op rather than a surprise.
    pub fn allows(&self, opset_mode: i32, stage: bool) -> bool {
        opset_mode == 2 && stage
    }

    /// The shader paths this stage set's fused heads select, in the order
    /// `profile_builder` tries them.
    ///
    /// Both INT16 and INT32 fused heads are compiled in together
    /// (`FuseStages::ALL` sets both), and the enum alone does not decide which
    /// one a frame takes — `MnnEngine`'s `InputMode` does. A chain whose sensor
    /// mode is `RawFloat` reads `isp.unpack32_*` (host already converted u16→f32),
    /// and one that is `RawInt`/`SplitInt16` reads `isp.unpack16_*` (the shader
    /// does the conversion). `FuseStages` gates *whether* fusion applies; the
    /// raw buffer's element type decides which fused form.
    pub fn fused_head_shader_paths(&self) -> Vec<&'static str> {
        if self.unpack32_blc_normal_cfa {
            vec![
                "isp.unpack32_blc_normal_cfa",
                "isp.unpack16_blc_normal_cfa",
                "isp.unpack_blc_normal_cfa",
            ]
        } else if self.unpack16_blc_normal_cfa {
            vec!["isp.unpack16_blc_normal_cfa", "isp.unpack_blc_normal_cfa"]
        } else if self.unpack_blc_normal_cfa {
            vec!["isp.unpack_blc_normal_cfa"]
        } else {
            vec![]
        }
    }
}

impl PipelineProfile {
    /// Minimal pipeline: raw → norm → CFA → BLC → WB → demosaic → CCM → tone → display.
    pub const LITE: Self = Self {
        label: "LITE",
        level: PipelineLevel::Lite,
        input_type: "raw",
        use_unpack: true,
        force_input16: false,
        use_fcs: false,
        use_ldci: false,
        use_ee: false,
        use_bad_pixel: false,
        demosaic_quality: DemosaicQuality::HqLinear,
        use_local_contrast: false,
        use_unsharp: false,
        use_lsc: false,
        lsc_strength: 0.5,
        use_warp: false,
        use_hdr: false,
        rotate_mode: 0,
        use_zone_stats: true,    // zone RGB means for basic AWB
        use_channel_means: true, // channel means for grey-world AWB
        use_tone_stats: false,   // skip AE tone stats (LITE)
        use_histogram: false,    // skip histogram (LITE)
        stats_downscale_max: 0,  // full resolution
        pipeline_downscale_target: 0,
        post_ccm_downscale: (0, 0),
        eis_margin: 0.0,
        use_bilateral: false,
        use_saturation: false,
        use_vignetting: false,
        use_colorspace: false,
        use_gamma: false,
        use_sharpen: false,
        use_wavelet_denoise: false,
        use_auto_contrast: false,
        use_normalize: false,
        normalize_at_display: false,
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
        opset_mode: 0, // Primitive by default
        fuse_stages: FuseStages::NONE,
    };

    /// Medium: adds bad pixel correction + unsharp mask.
    pub const MED: Self = Self {
        label: "MED",
        level: PipelineLevel::Medium,
        input_type: "raw",
        use_unpack: true,
        force_input16: false,
        use_fcs: false,
        use_ldci: false,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Standard,
        use_local_contrast: false,
        use_unsharp: true,
        use_lsc: false,
        lsc_strength: 0.5,
        use_warp: false,
        use_hdr: false,
        rotate_mode: 0,
        use_zone_stats: true,    // zone stats for multi-illuminant AWB
        use_channel_means: true, // channel means for grey-world AWB
        use_tone_stats: true,    // tone stats for AE metering
        use_histogram: false,    // skip histogram (MED)
        stats_downscale_max: 0,  // full resolution
        pipeline_downscale_target: 0,
        post_ccm_downscale: (0, 0),
        eis_margin: 0.0,
        use_bilateral: false,
        use_saturation: true,
        use_vignetting: false,
        use_colorspace: false,
        use_gamma: false,
        use_sharpen: false,
        use_wavelet_denoise: false,
        use_auto_contrast: false,
        use_normalize: false,
        normalize_at_display: false,
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::Bgra,
        opset_mode: 0, // Primitive by default
        fuse_stages: FuseStages::NONE,
    };

    /// Fused: MED's stage set, with every fusion-capable stage emitting a
    /// single `isp.*` dispatch instead of its primitive block chain.
    ///
    /// This is the profile the app runs. The difference from [`Self::MED`] is
    /// only how the stages are expressed — the pixel maths is the same, and
    /// `probe_fused_head` / `probe_enable_fused_stages` check each substitution
    /// against its primitive form before it is allowed in here.
    ///
    /// Fusing changes the memory picture in two ways worth knowing:
    ///
    ///   * the head becomes one dispatch over the packed sensor buffer instead
    ///     of five blocks that cast, split, divide, clip and slice the
    ///     full-resolution frame;
    ///   * a chain containing an `isp.*` op cannot use fp16 tensor storage —
    ///     those shaders declare `float[]` buffers, and MNN allocates two bytes
    ///     per element when fp16 is on. `MnnEngine::chain_has_fused_op` detects
    ///     that and drops to fp32 with a warning.
    pub const FUSED: Self = Self {
        label: "FUSED",
        level: PipelineLevel::Medium,
        input_type: "raw",
        use_unpack: true,
        force_input16: false,
        use_fcs: false,
        use_ldci: false,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Standard,
        use_local_contrast: false,
        use_unsharp: true,
        use_lsc: false,
        lsc_strength: 0.5,
        use_warp: false,
        use_hdr: false,
        rotate_mode: 0,
        use_zone_stats: true,
        use_channel_means: true,
        use_tone_stats: true,
        // AE clipping needs the histogram: without it `histogram_valid` is
        // always false and `IspController::update_histogram()` never runs, so
        // the production FUSED chain had an open AE loop. It reads the same
        // pooled stats tensor as the other reductions.
        use_histogram: true,
        // The one downscale every statistics block shares.
        //
        // Measured at 3840x2160 (`probe_4k_stats_shared`): 438.6 ms with the
        // statistics reading the full frame against 270.7 ms pooled to 540 — about
        // 38% of the frame. The numbers it produces do not move: calibration quad
        // means [0.9963, 1.0000, 0.9987] at full resolution against [0.9962,
        // 1.0000, 0.9987] at 648 and [0.9961, 1.0000, 0.9987] at 324, and the
        // displayed frame is identical to four decimals at every level.
        //
        // 540 rather than a smaller edge: past it the pool's own dispatch cost
        // dominates the reduction it saves (324 measured *slower* than 648 in one
        // sweep), and a mean is exact under the box filter at any factor, so there
        // is nothing to gain from going further.
        stats_downscale_max: 540,
        pipeline_downscale_target: 0,
        post_ccm_downscale: (0, 0),
        eis_margin: 0.0,
        use_bilateral: false,
        use_saturation: true,
        use_vignetting: false,
        use_colorspace: false,
        use_gamma: false,
        use_sharpen: false,
        use_wavelet_denoise: false,
        use_auto_contrast: false,
        use_normalize: false,
        normalize_at_display: false,
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::Bgra,
        // MnnIspMin is what lets `fuse_stages` have any effect at all.
        opset_mode: 2,
        fuse_stages: FuseStages::ALL,
        // The tail downscale only engages when `post_ccm_downscale` is set;
        // FUSED leaves the geometry alone unless the caller asks for it.
    };

    /// The whole RAW→display chain as one `isp.pipeline` dispatch.
    ///
    /// Not a ninth stage in `FuseStages::ALL` — a different *shape* of pipeline.
    /// Where `FUSED` is seven dispatches with six full-frame round trips between
    /// them, this is one dispatch with none. The cost of that is real and is worth
    /// naming:
    ///
    /// * the demosaic is a box filter over the output pixel's footprint, not an
    ///   edge-directed reconstruction. It is exact for a 1:1 crop and progressive
    ///   as the reduction grows — the same thing `isp.downscale` does, applied to
    ///   the CFA instead of to RGB;
    /// * there is no statistics readback, so the 3A loop is open. Wavelet, LDCI,
    ///   FCS, bilateral, the EIS warp and the per-format display permutations are
    ///   all absent — the op writes planar RGB in 0..1 and nothing else.
    ///
    /// What it keeps is everything the head of the chain does, which is where the
    /// cost was: bad-pixel correction before the footprint sum, lens shading, white
    /// balance and black level on the raw site, CCM, shadow lift, gamma and the
    /// fit/letterbox resample.
    ///
    ///   MED    270.0 ms   15 blocks
    ///   FUSED  141.4 ms    9 blocks
    ///   WHOLE  114.2 ms    2 blocks   mean 0.6019 vs MED's 0.6243
    pub const WHOLE: Self = Self {
        label: "WHOLE",
        level: PipelineLevel::Medium,
        input_type: "raw",
        // No unpack: `isp.pipeline` reads one sample per pixel, so there is no
        // packed pair to split. The head is a plain full-resolution raw plane.
        use_unpack: false,
        force_input16: true,
        // Match FUSED's feature set so the single `isp.pipeline` dispatch has
        // every stage bit enabled via `PipelineStages::PREVIEW` (ALL_STAGES).
        use_fcs: false,
        use_ldci: false,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Standard,
        use_local_contrast: false,
        use_unsharp: true,
        use_lsc: false,
        lsc_strength: 0.5,
        use_warp: false,
        use_hdr: false,
        rotate_mode: 0,
        // No stats blocks: the fused op has no way to emit their inputs, and a
        // chain that pretends to close the 3A loop while measuring nothing is
        // worse than one that says it is open.
        use_zone_stats: false,
        use_channel_means: false,
        use_tone_stats: false,
        use_histogram: false,
        stats_downscale_max: 0,
        pipeline_downscale_target: 0,
        post_ccm_downscale: (0, 0),
        eis_margin: 0.0,
        use_bilateral: false,
        use_saturation: true,
        use_vignetting: false,
        use_colorspace: false,
        use_gamma: false,
        use_sharpen: false,
        use_wavelet_denoise: false,
        use_auto_contrast: false,
        use_normalize: false,
        normalize_at_display: false,
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        // The op emits planar RGB in 0..1; `FloatRgb` is the consumer that means
        // exactly that.
        output_format: OutputFormat::FloatRgb,
        opset_mode: 2,
        fuse_stages: FuseStages {
            whole_pipeline: true,
            ..FuseStages::NONE
        },
    };

    /// Heavy: bad pixel + edge demosaic + local contrast + unsharp + LSC.
    pub const HEAVY: Self = Self {
        label: "HEAVY",
        level: PipelineLevel::Heavy,
        input_type: "raw",
        use_unpack: true,
        force_input16: false,
        use_fcs: true,
        use_ldci: true,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Edge,
        use_local_contrast: true,
        use_unsharp: true,
        use_lsc: true,
        lsc_strength: 0.5,
        use_warp: false,
        use_hdr: false,
        rotate_mode: 0,
        use_zone_stats: true,
        use_channel_means: true,
        use_tone_stats: true,
        use_histogram: true,
        stats_downscale_max: 540, // stats read from ~540p (downscaled from aux_hook_src)
        pipeline_downscale_target: 0, // 0 = disabled; set to e.g. 1920 for perf measurement
        post_ccm_downscale: (0, 0),
        eis_margin: 0.0,
        use_bilateral: true,
        use_saturation: false,
        use_vignetting: true,
        use_colorspace: false,
        use_gamma: true,
        use_sharpen: true,
        use_wavelet_denoise: false,
        use_auto_contrast: false,
        use_normalize: false,
        normalize_at_display: false,
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
        opset_mode: 0, // Primitive by default
        fuse_stages: FuseStages::NONE,
    };

    /// Everything-on profile: all available blocks enabled.
    pub const PRO: Self = Self {
        label: "PRO",
        level: PipelineLevel::Pro,
        input_type: "raw",
        use_unpack: true,
        force_input16: false,
        use_fcs: true,
        use_ldci: true,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Edge,
        use_local_contrast: true,
        use_unsharp: true,
        use_lsc: true,
        lsc_strength: 0.5,
        use_warp: true,
        use_hdr: true,
        rotate_mode: 0,
        use_zone_stats: true,    // zone stats for multi-illuminant AWB
        use_channel_means: true, // channel means for grey-world AWB
        use_tone_stats: true,    // tone stats for AE metering
        use_histogram: true,     // 16-bin histogram (full AE stats)
        stats_downscale_max: 0,  // full resolution
        pipeline_downscale_target: 0,
        post_ccm_downscale: (0, 0),
        eis_margin: 0.0,
        use_bilateral: true,
        use_saturation: true,
        use_vignetting: true,
        use_colorspace: true,
        use_gamma: true,
        use_sharpen: true,
        use_wavelet_denoise: true,
        use_auto_contrast: true,
        use_normalize: false,
        normalize_at_display: false,
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
        opset_mode: 0, // Primitive by default
        fuse_stages: FuseStages::NONE,
    };

    /// Test profile: minimal blocks for fast unit testing.
    /// Uses identity/placeholder blocks that skip expensive computation.
    pub const TEST: Self = Self {
        label: "TEST",
        level: PipelineLevel::Lite,
        input_type: "raw",
        use_unpack: true,
        force_input16: false,
        use_bad_pixel: false,
        demosaic_quality: DemosaicQuality::Standard,
        use_fcs: false,
        use_ldci: false,
        use_ee: false,
        use_lsc: false,
        lsc_strength: 0.5,
        use_warp: false,
        use_hdr: false,
        use_local_contrast: false,
        use_unsharp: false,
        rotate_mode: 0,
        use_zone_stats: true,     // zone stats for AWB (single block test)
        use_channel_means: false, // skip channel means
        use_tone_stats: false,    // skip tone stats
        use_histogram: false,     // skip histogram
        stats_downscale_max: 0,   // full resolution
        pipeline_downscale_target: 0,
        post_ccm_downscale: (0, 0),
        eis_margin: 0.0,
        use_bilateral: false,
        use_saturation: false,
        use_vignetting: false,
        use_colorspace: false,
        use_gamma: false,
        use_sharpen: false,
        use_wavelet_denoise: false,
        use_auto_contrast: false,
        use_normalize: false,
        normalize_at_display: false,
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
        opset_mode: 0, // Primitive by default
        fuse_stages: FuseStages::NONE,
    };

    /// Unified profile — full ISP pipeline with all blocks enabled.
    /// Combines raw input, unpack, BLC, CCM, white balance, demosaic,
    /// tone, FCS, LDCI, EE, and post-processing blocks.
    /// Note: bilateral, vignetting, colorspace are CPU-only, not GPU-fused.
    pub const UNIFIED: Self = Self {
        label: "UNIFIED",
        level: PipelineLevel::Heavy,
        input_type: "raw",
        use_unpack: true,
        force_input16: false,
        use_fcs: false,
        use_ldci: false,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Edge,
        use_local_contrast: false,
        use_unsharp: false,
        use_lsc: false,
        lsc_strength: 0.5,
        use_warp: true,
        use_hdr: false,
        rotate_mode: 0,
        use_zone_stats: true,
        use_channel_means: true,
        use_tone_stats: true,
        use_histogram: false,
        stats_downscale_max: 540,
        pipeline_downscale_target: 0,
        post_ccm_downscale: (0, 0),
        eis_margin: 0.0,
        use_bilateral: false,
        use_saturation: true,
        use_vignetting: false,
        use_colorspace: false,
        use_gamma: false,
        use_sharpen: false,
        use_wavelet_denoise: false,
        use_auto_contrast: false,
        use_normalize: false,
        normalize_at_display: false,
        use_tiled_rendering: true, // Enable for 4K→4K
        tile_count_x: 2,
        tile_count_y: 2,
        tile_overlap: 2,
        output_format: OutputFormat::Bgra,
        opset_mode: 2, // MnnIspMin for Vulkan GPU backend
        fuse_stages: FuseStages::ALL,
    };

    /// HDR profile — multi-exposure burst with async HDR merge.
    /// Pipeline processes each exposure individually (MED-level quality),
    /// then queues ARGB frames for async HDR worker.
    ///
    /// Capture flow:
    ///   1. 3-frame burst: -2EV, 0EV, +2EV
    ///   2. Each frame processed by ISP: BayerWB → Demosaic+CCM → Tone→ ARGB
    ///   3. Frames queued to HdrCaptureQueue (MPSC channel)
    ///   4. Async worker: Align → Merge → Neural Enhance → Encode
    ///
    /// Memory: 3× ISP output buffers + 2× worker buffers (<200MB peak)
    /// Latency: ~100ms ISP + ~20ms HDR = ~120ms total
    pub const HDR: Self = Self {
        label: "HDR",
        level: PipelineLevel::Heavy,
        input_type: "raw",
        use_unpack: true,
        force_input16: false,
        use_fcs: true,
        use_ldci: false,
        use_ee: false,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::HqLinear,
        use_local_contrast: false,
        use_unsharp: false,
        use_lsc: true,
        lsc_strength: 0.5,
        use_warp: true, // EIS alignment between exposures
        use_hdr: true,
        rotate_mode: 0,
        use_zone_stats: true,
        use_channel_means: true,
        use_tone_stats: true,
        use_histogram: false,
        stats_downscale_max: 0,
        pipeline_downscale_target: 0,
        post_ccm_downscale: (0, 0),
        eis_margin: 0.05, // 5% margin for EIS alignment warp
        use_bilateral: false,
        use_saturation: false,
        use_vignetting: true,
        use_colorspace: false,
        use_gamma: false,
        use_sharpen: false,
        use_wavelet_denoise: false,
        use_auto_contrast: false,
        use_normalize: false,
        normalize_at_display: false,
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
        opset_mode: 0, // Primitive by default
        fuse_stages: FuseStages::NONE,
    };

    /// YUV-source profile — for cameras that only deliver `YUV_420_888`.
    ///
    /// The capture layer (`cam-camera-app::camera::copy_pixels`) expands the
    /// 8-bit luma plane into a `GG` mosaic and scales it to full `u16` range,
    /// so by the time a frame reaches the engine it is Bayer-shaped `u16` — the
    /// same tensor the RAW profiles consume. The head must therefore be the
    /// packed RAW head, not `Yuv420InputBlock`: that block declares a
    /// `[1,3,H,W]` FLOAT tensor (three planes), which cannot be fed from a
    /// single-channel `u16` buffer and makes MNN's resize-on-demand refuse the
    /// session outright.
    pub const YUV: Self = Self {
        label: "YUV",
        level: PipelineLevel::Medium,
        input_type: "raw",
        use_unpack: true,
        force_input16: false,
        use_fcs: false,
        use_ldci: false,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Standard,
        use_local_contrast: false,
        use_unsharp: true,
        use_lsc: false,
        lsc_strength: 0.5,
        use_warp: false,
        use_hdr: false,
        rotate_mode: 0,
        use_zone_stats: true,    // zone stats for multi-illuminant AWB
        use_channel_means: true, // channel means for grey-world AWB
        use_tone_stats: true,    // tone stats for AE metering
        use_histogram: false,    // skip histogram
        stats_downscale_max: 0,  // full resolution
        pipeline_downscale_target: 0,
        post_ccm_downscale: (0, 0),
        eis_margin: 0.0,
        use_bilateral: false,
        use_saturation: true,
        use_vignetting: false,
        use_colorspace: false,
        use_gamma: false,
        use_sharpen: false,
        use_wavelet_denoise: false,
        use_auto_contrast: false,
        use_normalize: false,
        normalize_at_display: false,
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::Bgra,
        opset_mode: 0, // Primitive by default
        fuse_stages: FuseStages::NONE,
    };

    /// All built-in profiles.
    pub const ALL: [Self; 7] = [
        Self::LITE,
        Self::MED,
        Self::HEAVY,
        Self::PRO,
        Self::TEST,
        Self::UNIFIED,
        Self::HDR,
    ];

    /// Create a custom profile with override flags.
    pub const fn custom(
        label: &'static str,
        level: PipelineLevel,
        input_type: &'static str,
        use_unpack: bool,
        use_fcs: bool,
        use_ldci: bool,
        use_ee: bool,
        use_bad_pixel: bool,
        demosaic_quality: DemosaicQuality,
        use_local_contrast: bool,
        use_unsharp: bool,
        use_lsc: bool,
        use_warp: bool,
        use_hdr: bool,
        rotate_mode: i32, // 0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip
        use_zone_stats: bool, // per-zone RGB means (AveragePool) for multi-illuminant AWB
        use_channel_means: bool, // global channel means (ReduceMean) for grey-world AWB
        use_tone_stats: bool, // luma mean/min/max + clipped/shadows for AE metering
        use_histogram: bool, // 16-bin luminance histogram for AE and clipping detection
        stats_downscale_max: u32, // max pixel dimension for stats (0 = full res)
        pipeline_downscale_target: u32, // max pixel dimension for main pipeline (0 = full res)
        post_ccm_downscale: (u32, u32), // (w, h) downscale right after CCM, (0,0) = off
        eis_margin: f64,  // EIS margin fraction (0.05 = 5%), default 0.0
        use_bilateral: bool, // bilateral filter (edge-preserving denoise)
        use_saturation: bool, // saturation control
        use_vignetting: bool, // vignetting correction
        use_colorspace: bool, // color space conversion
        use_gamma: bool,  // gamma correction
        use_sharpen: bool, // sharpening
        use_wavelet_denoise: bool, // wavelet denoising
        use_auto_contrast: bool, // auto contrast
        use_normalize: bool, // normalization
        normalize_at_display: bool, // fold the normalise divide into the display stage
        use_tiled_rendering: bool, // enable tiled rendering for high-res output
        tile_count_x: u32, // horizontal tile count (e.g., 2 for 4K→4K)
        tile_count_y: u32, // vertical tile count (e.g., 2 for 4K→4K)
        tile_overlap: u32, // overlap pixels between tiles for convolution
        opset_mode: i32,  // 0=Primitive, 1=Custom, 2=MnnIspMin
        fuse_stages: FuseStages, // per-stage allowlist for fused isp.* emission
    ) -> Self {
        Self {
            label,
            level,
            input_type,
            use_unpack,
            force_input16: false,
            use_fcs,
            use_ldci,
            use_ee,
            use_bad_pixel,
            demosaic_quality,
            use_local_contrast,
            use_unsharp,
            use_lsc,
            fuse_stages,
            lsc_strength: 0.5,
            use_warp,
            use_hdr,
            rotate_mode,
            use_zone_stats,            // per-zone RGB means (AWB multi-illuminant)
            use_channel_means,         // global channel means (grey-world AWB)
            use_tone_stats,            // luma stats for AE metering
            use_histogram,             // 16-bin luminance histogram
            stats_downscale_max,       // adaptive stats downscale
            pipeline_downscale_target, // main pipeline downscale target
            post_ccm_downscale,        // post-CCM downscale (w, h)
            eis_margin,                // EIS margin fraction
            use_bilateral,             // bilateral filter
            use_saturation,            // saturation control
            use_vignetting,            // vignetting correction
            use_colorspace,            // color space conversion
            use_gamma,                 // gamma correction
            use_sharpen,               // sharpening
            use_wavelet_denoise,       // wavelet denoising
            use_auto_contrast,         // auto contrast
            use_normalize,             // normalization
            normalize_at_display,      // defer the divide to display
            use_tiled_rendering,       // enable tiled rendering
            tile_count_x,              // horizontal tile count
            tile_count_y,              // vertical tile count
            tile_overlap,              // overlap pixels
            output_format: OutputFormat::PackedRgb,
            opset_mode,
        }
    }

    /// Force the pure 16-bit raw input convention (`[1,1,H,W]` INT16) instead
    /// of the packed 2×int16-as-int32 tensor. The MNN converter upcasts the
    /// INT16 input to FLOAT32; the engine adapts to either convention.
    pub fn with_force_input16(mut self) -> Self {
        self.force_input16 = true;
        self
    }

    /// Override the packed-split input convention (`[1,2,H,W/2]` INT32).
    /// `false` selects the legacy raw `[1,1,H,W]` INT32 head (unless
    /// `force_input16` is set, which takes precedence).
    pub fn with_use_unpack(mut self, use_unpack: bool) -> Self {
        self.use_unpack = use_unpack;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fused_heads_are_compiled_together() {
        let f = FuseStages::ALL;
        assert!(f.unpack32_blc_normal_cfa);
        assert!(f.unpack16_blc_normal_cfa);
        assert!(f.unpack_blc_normal_cfa);
        let n = FuseStages::NONE;
        assert!(!n.unpack32_blc_normal_cfa);
        assert!(!n.unpack16_blc_normal_cfa);
        assert!(!n.unpack_blc_normal_cfa);
    }

    #[test]
    fn test_fused_head_shader_paths_all() {
        let paths = FuseStages::ALL.fused_head_shader_paths();
        assert_eq!(
            paths,
            vec![
                "isp.unpack32_blc_normal_cfa",
                "isp.unpack16_blc_normal_cfa",
                "isp.unpack_blc_normal_cfa",
            ]
        );
    }

    #[test]
    fn test_fused_head_shader_paths_none_is_empty() {
        assert!(FuseStages::NONE.fused_head_shader_paths().is_empty());
    }

    #[test]
    fn test_fused_heads_require_mnn_isp_min() {
        // Both fused heads only ever emit under opset 2 (MnnIspMin).
        let f = FuseStages::ALL;
        assert!(!f.allows(0, f.unpack32_blc_normal_cfa)); // Primitive
        assert!(!f.allows(1, f.unpack16_blc_normal_cfa)); // Custom
        assert!(f.allows(2, f.unpack32_blc_normal_cfa)); // MnnIspMin
        assert!(f.allows(2, f.unpack16_blc_normal_cfa));
    }

    #[test]
    fn test_lite_profile() {
        let p = PipelineProfile::LITE;
        assert_eq!(p.label, "LITE");
        assert_eq!(p.level, PipelineLevel::Lite);
        assert!(!p.use_fcs);
        assert!(!p.use_ldci);
        assert!(!p.use_ee);
        assert!(!p.use_bad_pixel);
    }

    #[test]
    fn test_heavy_profile() {
        let p = PipelineProfile::HEAVY;
        assert!(p.use_fcs);
        assert!(p.use_ldci);
        assert!(p.use_ee);
        assert!(p.use_bad_pixel);
        assert!(p.use_lsc);
        assert_eq!(p.demosaic_quality, DemosaicQuality::Edge);
    }

    /// Chains keep their *stage* count; the identity bridges that used to pad
    /// it are an emission detail and are off by default (see
    /// `BRIDGE_IDENTITIES`), so the expected length follows that switch.
    #[test]
    fn test_build_blocks_lite() {
        let blocks = PipelineProfile::LITE.build_blocks(128, 72, 0);
        // The count is not the contract — the *shape* is. Pass-through blocks
        // are elided (see `GraphComposer::elide_noops`), so asserting a length
        // would pin an emission detail instead of the pipeline.
        assert!(!blocks.is_empty(), "LITE built no blocks");
        assert!(
            blocks[0].id().starts_with("raw_input"),
            "LITE must start at the sensor, got '{}'",
            blocks[0].id()
        );
        assert_eq!(
            blocks.last().unwrap().id(),
            "display",
            "LITE must end at the display stage"
        );
        assert!(
            !blocks.iter().any(|b| b.is_noop()),
            "no pass-through block should survive into the graph"
        );
    }

    #[test]
    fn test_build_blocks_heavy() {
        let blocks = PipelineProfile::HEAVY.build_blocks(128, 72, 0);
        assert!(!blocks.is_empty(), "HEAVY built no blocks");
        assert!(
            blocks[0].id().starts_with("raw_input"),
            "HEAVY must start at the sensor, got '{}'",
            blocks[0].id()
        );
        assert_eq!(blocks.last().unwrap().id(), "display");
        assert!(
            !blocks.iter().any(|b| b.is_noop()),
            "no pass-through block should survive into the graph"
        );
        // HEAVY's distinguishing feature is the bad-pixel stage.
        assert!(
            blocks.iter().any(|b| b.id().starts_with("blc_dpc")),
            "HEAVY should keep its bad-pixel correction"
        );
    }

    #[test]
    fn test_yuv_profile_uses_packed_raw_head() {
        // The app hands the engine a Bayer-shaped `u16` buffer even on the YUV
        // fallback, so the YUV profile must compose the same packed RAW head
        // the RAW profiles use. A `Yuv420InputBlock` head would declare a
        // `[1,3,H,W]` float tensor that no `u16` frame can fill.
        let p = PipelineProfile::YUV;
        assert_eq!(p.input_type, "raw");
        assert!(p.use_unpack);
        assert!(!p.force_input16);

        let blocks = p.build_blocks(128, 72, 0);
        assert!(!blocks.is_empty());
        assert_eq!(
            blocks[0].id(),
            "raw_input_packed",
            "YUV profile head should be the packed RAW head, got '{}'",
            blocks[0].id()
        );
    }

    #[test]
    fn test_custom_profile() {
        let p = PipelineProfile::custom(
            "CUSTOM",
            PipelineLevel::Pro,
            "raw",
            true,
            true,
            true,
            true,
            true,
            DemosaicQuality::Edge,
            true,
            true,
            true,
            true,
            false,
            0,      // rotate_mode: none
            true,   // use_zone_stats
            false,  // use_channel_means
            true,   // use_tone_stats
            false,  // use_histogram
            0,      // stats_downscale_max
            0,      // pipeline_downscale_target
            (0, 0), // post_ccm_downscale
            0.0,    // eis_margin
            true,   // use_bilateral
            true,   // use_saturation
            true,   // use_vignetting
            false,  // use_colorspace
            true,   // use_gamma
            true,   // use_sharpen
            false,  // use_wavelet_denoise
            false,  // use_auto_contrast
            false,  // use_normalize
            false,  // normalize_at_display
            false,  // use_tiled_rendering
            1,      // tile_count_x
            1,      // tile_count_y
            0,      // tile_overlap
            0,      // opset_mode: Primitive
            FuseStages::NONE,
        );
        assert_eq!(p.label, "CUSTOM");
        assert!(p.use_warp);
        assert!(!p.use_hdr);
        assert!(p.use_zone_stats);
        assert!(!p.use_channel_means);
        assert!(p.use_tone_stats);
        assert!(!p.use_histogram);
    }

    #[test]
    fn test_legacy_float_path() {
        // Old FLOAT path: use_unpack=false, no UnpackBlock in pipeline
        let p = PipelineProfile::custom(
            "LEGACY",
            PipelineLevel::Lite,
            "raw",
            false,
            false,
            false,
            false,
            false,
            DemosaicQuality::Standard,
            false,
            false,
            false,
            false,
            false,
            0,      // rotate_mode: none
            false,  // use_zone_stats
            false,  // use_channel_means
            false,  // use_tone_stats
            false,  // use_histogram
            0,      // stats_downscale_max
            0,      // pipeline_downscale_target
            (0, 0), // post_ccm_downscale
            0.0,    // eis_margin
            false,  // use_bilateral
            false,  // use_saturation
            false,  // use_vignetting
            false,  // use_colorspace
            false,  // use_gamma
            false,  // use_sharpen
            false,  // use_wavelet_denoise
            false,  // use_auto_contrast
            false,  // use_normalize
            false,  // normalize_at_display
            false,  // use_tiled_rendering
            1,      // tile_count_x
            1,      // tile_count_y
            0,      // tile_overlap
            0,      // opset_mode: Primitive
            FuseStages::NONE,
        );
        let blocks = p.build_blocks(128, 72, 0);
        assert!(!blocks.is_empty(), "legacy profile built no blocks");
        assert!(
            blocks[0].id().starts_with("raw_input"),
            "legacy must start at the sensor, got '{}'",
            blocks[0].id()
        );

        // The point of this profile is the *absence* of `unpack`: the old float
        // path feeds a `[1,1,H,W]` plane straight into normalize.
        let ids: Vec<&str> = blocks.iter().map(|b| b.id()).collect();
        assert!(
            !ids.iter().any(|id| id.starts_with("unpack")),
            "legacy float path should have no Unpack block, got {ids:?}"
        );
        let normalize_at = ids
            .iter()
            .position(|id| *id == "normalize")
            .unwrap_or_else(|| panic!("legacy chain has no normalize: {ids:?}"));
        assert_eq!(
            normalize_at, 1,
            "normalize should follow raw_input directly, got {ids:?}"
        );
    }

    #[test]
    fn test_block_count() {
        // `block_count()` feeds memory estimates, so it has to be the count the
        // graph really contains. Pass-through blocks are elided and bridges are
        // off by default, so a literal number would go stale every time either
        // of those changes — assert the relationship instead.
        let lite = PipelineProfile::LITE.block_count();
        let lite_stages = PipelineProfile::LITE.build_blocks(128, 72, 0).len();
        let lite_aux = PipelineProfile::LITE.build_aux_blocks(72, 128).len();
        assert_eq!(
            lite,
            lite_stages + lite_aux,
            "LITE count must match its parts"
        );

        let heavy = PipelineProfile::HEAVY.block_count();
        let heavy_stages = PipelineProfile::HEAVY.build_blocks(128, 72, 0).len();
        // block_count() queries the aux blocks at the same tiny geometry it uses
        // for the main chain, and build_aux_blocks can add a stats downscale
        // depending on the size it is given. Mirror the call exactly.
        let heavy_aux = PipelineProfile::HEAVY.build_aux_blocks(72, 128).len();
        assert_eq!(
            heavy,
            heavy_stages + heavy_aux,
            "HEAVY count must match its parts"
        );

        // HEAVY has strictly more work in it than LITE (bad pixel, LSC, edge
        // demosaic), which is the property the number is there to express.
        assert!(heavy > lite, "HEAVY should carry more stages than LITE");
    }

    #[test]
    fn test_build_aux_blocks_lite() {
        let aux = PipelineProfile::LITE.build_aux_blocks(1080, 1920);
        // LITE: zone_stats + channel_means + calibration
        assert_eq!(aux.len(), 3, "LITE stats: zone + channel + calibration");
        assert_eq!(aux[0].id(), "zone_stats");
        assert_eq!(aux[1].id(), "channel_means");
    }

    #[test]
    fn test_build_aux_blocks_pro() {
        let aux = PipelineProfile::PRO.build_aux_blocks(1080, 1920);
        // PRO: zone_stats + channel_means + tone_stats + histogram + calibration
        assert_eq!(aux.len(), 5, "PRO stats: all 4 + calibration");
    }

    #[test]
    fn test_build_aux_blocks_test() {
        let aux = PipelineProfile::TEST.build_aux_blocks(1080, 1920);
        // TEST: zone_stats + calibration
        assert_eq!(aux.len(), 2, "TEST stats: zone_stats + calibration");
    }
}
