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
    /// Enable packed INT32 input with UnpackBlock for true zero-copy.
    /// When true (default), RawInputBlock uses INT32 at half-width and
    /// UnpackBlock is injected after it. When false, uses legacy FLOAT path.
    pub use_unpack: bool,
    /// Declare the packed even/odd input as native INT16 (`[1,2,H,W/2]` INT16)
    /// instead of packed INT32. The MNN converter upcasts INT16-declared
    /// inputs to FLOAT32; the engine adapts to either convention (splitting
    /// the u16 pairs into i32 lanes for INT32 models, f32 lanes for FLOAT32
    /// models). Default: false (packed INT32).
    pub input_native16: bool,
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
}

impl PipelineProfile {
    /// Minimal pipeline: raw → norm → CFA → BLC → WB → demosaic → CCM → tone → display.
    pub const LITE: Self = Self {
        label: "LITE",
        level: PipelineLevel::Lite,
        use_unpack: true,
        input_native16: false,
        use_fcs: false,
        use_ldci: false,
        use_ee: false,
        use_bad_pixel: false,
        demosaic_quality: DemosaicQuality::HqLinear,
        use_local_contrast: false,
        use_unsharp: false,
        use_lsc: false,
        use_warp: false,
        use_hdr: false,
        rotate_mode: 0,
        use_zone_stats: true,    // zone RGB means for basic AWB
        use_channel_means: true, // channel means for grey-world AWB
        use_tone_stats: false,   // skip AE tone stats (LITE)
        use_histogram: false,    // skip histogram (LITE)
        stats_downscale_max: 0,  // full resolution
        pipeline_downscale_target: 0,
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
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
    };

    /// Medium: adds bad pixel correction + unsharp mask.
    pub const MED: Self = Self {
        label: "MED",
        level: PipelineLevel::Medium,
        use_unpack: true,
        input_native16: false,
        use_fcs: false,
        use_ldci: false,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Standard,
        use_local_contrast: false,
        use_unsharp: true,
        use_lsc: false,
        use_warp: false,
        use_hdr: false,
        rotate_mode: 0,
        use_zone_stats: true,    // zone stats for multi-illuminant AWB
        use_channel_means: true, // channel means for grey-world AWB
        use_tone_stats: true,    // tone stats for AE metering
        use_histogram: false,    // skip histogram (MED)
        stats_downscale_max: 0,  // full resolution
        pipeline_downscale_target: 0,
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
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
    };

    /// Heavy: bad pixel + edge demosaic + local contrast + unsharp + LSC.
    pub const HEAVY: Self = Self {
        label: "HEAVY",
        level: PipelineLevel::Heavy,
        use_unpack: true,
        input_native16: false,
        use_fcs: true,
        use_ldci: true,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Edge,
        use_local_contrast: true,
        use_unsharp: true,
        use_lsc: true,
        use_warp: false,
        use_hdr: false,
        rotate_mode: 0,
        use_zone_stats: true,
        use_channel_means: true,
        use_tone_stats: true,
        use_histogram: true,
        stats_downscale_max: 540, // stats read from ~540p (downscaled from aux_hook_src)
        pipeline_downscale_target: 0, // 0 = disabled; set to e.g. 1920 for perf measurement
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
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
    };

    /// Everything-on profile: all available blocks enabled.
    pub const PRO: Self = Self {
        label: "PRO",
        level: PipelineLevel::Pro,
        use_unpack: true,
        input_native16: false,
        use_fcs: true,
        use_ldci: true,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Edge,
        use_local_contrast: true,
        use_unsharp: true,
        use_lsc: true,
        use_warp: true,
        use_hdr: true,
        rotate_mode: 0,
        use_zone_stats: true,    // zone stats for multi-illuminant AWB
        use_channel_means: true, // channel means for grey-world AWB
        use_tone_stats: true,    // tone stats for AE metering
        use_histogram: true,     // 16-bin histogram (full AE stats)
        stats_downscale_max: 0,  // full resolution
        pipeline_downscale_target: 0,
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
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
    };

    /// Test profile: minimal blocks for fast unit testing.
    /// Uses identity/placeholder blocks that skip expensive computation.
    pub const TEST: Self = Self {
        label: "TEST",
        level: PipelineLevel::Lite,
        use_unpack: true,
        input_native16: false,
        use_bad_pixel: false,
        demosaic_quality: DemosaicQuality::Standard,
        use_fcs: false,
        use_ldci: false,
        use_ee: false,
        use_lsc: false,
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
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
    };

    /// Unified profile — full ISP pipeline with all blocks enabled.
    /// Combines raw input, unpack, BLC, CCM, white balance, demosaic,
    /// tone, FCS, LDCI, EE, and post-processing blocks.
    /// Note: bilateral, vignetting, colorspace are CPU-only, not GPU-fused.
    pub const UNIFIED: Self = Self {
        label: "UNIFIED",
        level: PipelineLevel::Heavy,
        use_unpack: false,
        input_native16: false,
        use_fcs: true,
        use_ldci: true,
        use_ee: true,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::Standard,
        use_local_contrast: false,
        use_unsharp: false,
        use_lsc: true,
        use_warp: false,
        use_hdr: false,
        rotate_mode: 0,
        use_zone_stats: true,
        use_channel_means: true,
        use_tone_stats: true,
        use_histogram: true,
        stats_downscale_max: 0,
        pipeline_downscale_target: 0,
        eis_margin: 0.0,
        use_bilateral: true, // Simplified: AveragePool blur
        use_saturation: true,
        use_vignetting: true, // Simplified: Mul with gain_map
        use_colorspace: true, // Simplified: Identity passthrough
        use_gamma: true,
        use_sharpen: true,
        use_wavelet_denoise: true, // Simplified: AveragePool denoise
        use_auto_contrast: true,   // GPU-accelerated via ISP op
        use_normalize: true,
        use_tiled_rendering: true, // Enable for 4K→4K
        tile_count_x: 2,
        tile_count_y: 2,
        tile_overlap: 2,
        output_format: OutputFormat::PackedRgb,
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
        use_unpack: true,
        input_native16: false,
        use_fcs: true,
        use_ldci: false,
        use_ee: false,
        use_bad_pixel: true,
        demosaic_quality: DemosaicQuality::HqLinear,
        use_local_contrast: false,
        use_unsharp: false,
        use_lsc: true,
        use_warp: true, // EIS alignment between exposures
        use_hdr: true,
        rotate_mode: 0,
        use_zone_stats: true,
        use_channel_means: true,
        use_tone_stats: true,
        use_histogram: false,
        stats_downscale_max: 0,
        pipeline_downscale_target: 0,
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
        use_tiled_rendering: false,
        tile_count_x: 1,
        tile_count_y: 1,
        tile_overlap: 0,
        output_format: OutputFormat::PackedRgb,
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
        use_tiled_rendering: bool, // enable tiled rendering for high-res output
        tile_count_x: u32, // horizontal tile count (e.g., 2 for 4K→4K)
        tile_count_y: u32, // vertical tile count (e.g., 2 for 4K→4K)
        tile_overlap: u32, // overlap pixels between tiles for convolution
    ) -> Self {
        Self {
            label,
            level,
            use_unpack,
            input_native16: false,
            use_fcs,
            use_ldci,
            use_ee,
            use_bad_pixel,
            demosaic_quality,
            use_local_contrast,
            use_unsharp,
            use_lsc,
            use_warp,
            use_hdr,
            rotate_mode,
            use_zone_stats,            // per-zone RGB means (AWB multi-illuminant)
            use_channel_means,         // global channel means (grey-world AWB)
            use_tone_stats,            // luma stats for AE metering
            use_histogram,             // 16-bin luminance histogram
            stats_downscale_max,       // adaptive stats downscale
            pipeline_downscale_target, // main pipeline downscale target
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
            use_tiled_rendering,       // enable tiled rendering
            tile_count_x,              // horizontal tile count
            tile_count_y,              // vertical tile count
            tile_overlap,              // overlap pixels
            output_format: OutputFormat::PackedRgb,
        }
    }

    /// Declare the packed even/odd input as native INT16 (`[1,2,H,W/2]` INT16)
    /// instead of packed INT32. The MNN converter upcasts INT16-declared
    /// inputs to FLOAT32; the engine adapts to either convention.
    pub fn with_input_native16(mut self) -> Self {
        self.input_native16 = true;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn test_build_blocks_lite() {
        let blocks = PipelineProfile::LITE.build_blocks(128, 0);
        // LITE: 24 main blocks + 23 identity bridges = 47
        assert_eq!(
            blocks.len(),
            47,
            "LITE (primitive) should have 47 blocks (24 + 23 identities), got {}",
            blocks.len()
        );
    }

    #[test]
    fn test_build_blocks_heavy() {
        let blocks = PipelineProfile::HEAVY.build_blocks(128, 0);
        // HEAVY: 24 main blocks + 23 identity bridges = 47
        assert_eq!(
            blocks.len(),
            47,
            "HEAVY (primitive) should have 47 blocks (24 + 23 identities), got {}",
            blocks.len()
        );
    }

    #[test]
    fn test_custom_profile() {
        let p = PipelineProfile::custom(
            "CUSTOM",
            PipelineLevel::Pro,
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
            0,     // rotate_mode: none
            true,  // use_zone_stats
            false, // use_channel_means
            true,  // use_tone_stats
            false, // use_histogram
            0,     // stats_downscale_max
            0,     // pipeline_downscale_target
            0.0,   // eis_margin
            true,  // use_bilateral
            true,  // use_saturation
            true,  // use_vignetting
            false, // use_colorspace
            true,  // use_gamma
            true,  // use_sharpen
            false, // use_wavelet_denoise
            false, // use_auto_contrast
            false, // use_normalize
            false, // use_tiled_rendering
            1,     // tile_count_x
            1,     // tile_count_y
            0,     // tile_overlap
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
            0,     // rotate_mode: none
            false, // use_zone_stats
            false, // use_channel_means
            false, // use_tone_stats
            false, // use_histogram
            0,     // stats_downscale_max
            0,     // pipeline_downscale_target
            0.0,   // eis_margin
            false, // use_bilateral
            false, // use_saturation
            false, // use_vignetting
            false, // use_colorspace
            false, // use_gamma
            false, // use_sharpen
            false, // use_wavelet_denoise
            false, // use_auto_contrast
            false, // use_normalize
            false, // use_tiled_rendering
            1,     // tile_count_x
            1,     // tile_count_y
            0,     // tile_overlap
        );
        let blocks = p.build_blocks(128, 0);
        assert_eq!(
            blocks.len(),
            45,
            "Legacy should have 45 blocks (23 + 22 identities), got {}",
            blocks.len()
        );
        assert_eq!(blocks[0].id(), "raw_input");
        // With identity bridges, the second block is the identity bridge
        assert!(
            blocks[1].id().starts_with("id_raw_input_"),
            "Second block should be identity bridge, got '{}'",
            blocks[1].id()
        );
        assert_eq!(
            blocks[2].id(),
            "normalize",
            "Third block should be Normalize (no Unpack)"
        );
    }

    #[test]
    fn test_block_count() {
        // block_count() calls build_blocks() which now includes identity bridges
        // LITE: 24 main + 23 identities + 3 stats = 50
        assert_eq!(
            PipelineProfile::LITE.block_count(),
            50,
            "LITE: 24 main + 23 identities + 3 stats = 50"
        );
        // HEAVY: 24 main + 23 identities + 5 stats = 52
        assert_eq!(
            PipelineProfile::HEAVY.block_count(),
            52,
            "HEAVY: 24 main + 23 identities + 5 stats = 52"
        );
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
