//! cam-ipa Swiss Army Knife CLI — polyglot ELF for softisp
//!
//! One binary that does most of the softisp IPA jobs:
//!   - `generate`   — synthetic test frame generator (Bayer 16-bit, configurable res)
//!   - `capture`    — V4L2 frame capture via cam-hal-linux (feature `v4l2`)
//!   - `process`    — run a frame through the full ISP pipeline (3A + neural)
//!   - `bench`      — FPS/latency benchmark over N frames
//!   - `stats`      — collect per-frame stats for CMOS sensor characterization
//!   - `zerocopy`   — test memfd/dmabuf zero-copy buffer path
//!   - `probe`      — probe profile capabilities and engine info (--json for JSON)
//!   - `profile-json` — dump a single profile as JSON, or load custom profile JSON
//!   - `cmos-dump`    — read V4L2 sensor metadata and output libcamera-compatible CMOS JSON
//!
//! Compiled as:
//!   - cdylib (libcam_ipa_sk.so) — exports `ipa_sk_main` for dlopen,
//!     and `my_entry` as ELF entry point for direct execution.
//!   - included by src/main.rs for the `ipa_sk` binary.
//!
//! Build:
//!   cargo build -p cam-ipa-sk                    # synthetic only
//!   cargo build -p cam-ipa-sk --features v4l2    # + V4L2 capture
//!
//! Run:
//!   ./target/debug/libcam_ipa_sk.so --help       # direct execution (polyglot)
//!   ./target/debug/ipa_sk --help                 # binary wrapper
//!   ./target/debug/ipa_sk generate --count 1 --profile fused --width 3840 --height 2160

use cam_ipa::{
    Buffer, ControlList, EngineBackend, IpaCallbacks, IpaConfig, IpaContext, IpaFormat,
    IpaSensorInfo, IpaSettings, MemBuffer, Rectangle, Size,
};
use cam_isp::profile::PipelineProfile;
use clap::{Parser, Subcommand, ValueEnum};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::time::Instant;

#[cfg(feature = "v4l2")]
use cam_hal_linux::{capture_single_v4l2_frame, list_v4l2_devices};

/// Top-level CLI argument parser
#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "cam-ipa Swiss Army Knife — softisp IPA toolkit"
)]
struct Args {
    #[command(subcommand)]
    command: Option<Commands>,
}

/// Profile argument enum
#[derive(ValueEnum, Clone, Debug, Copy)]
enum ProfileArg {
    Lite,
    Med,
    Fused,
    Heavy,
    Pro,
    Test,
    Unified,
    Hdr,
    Yuv,
}

/// Format argument enum
#[derive(ValueEnum, Clone, Debug, Copy)]
enum FormatArg {
    Rgb,
    Rgba,
    Bgra,
    Argb,
    Abgr,
    Yuv,
    Nv12,
    P010,
}

/// Backend argument enum
#[derive(ValueEnum, Clone, Debug, Copy)]
enum BackendArg {
    MnnVulkan,
    Cpu,
    OnnxRuntime,
}

impl From<ProfileArg> for PipelineProfile {
    fn from(arg: ProfileArg) -> Self {
        match arg {
            ProfileArg::Lite => PipelineProfile::LITE,
            ProfileArg::Med => PipelineProfile::MED,
            ProfileArg::Fused => PipelineProfile::FUSED,
            ProfileArg::Heavy => PipelineProfile::HEAVY,
            ProfileArg::Pro => PipelineProfile::PRO,
            ProfileArg::Test => PipelineProfile::TEST,
            ProfileArg::Unified => PipelineProfile::UNIFIED,
            ProfileArg::Hdr => PipelineProfile::HDR,
            ProfileArg::Yuv => PipelineProfile::YUV,
        }
    }
}

impl From<FormatArg> for IpaFormat {
    fn from(arg: FormatArg) -> Self {
        match arg {
            FormatArg::Rgb => IpaFormat::RGB,
            FormatArg::Rgba => IpaFormat::RGBA,
            FormatArg::Bgra => IpaFormat::BGRA,
            FormatArg::Argb => IpaFormat::ARGB,
            FormatArg::Abgr => IpaFormat::ABGR,
            FormatArg::Yuv => IpaFormat::YUV,
            FormatArg::Nv12 => IpaFormat::NV12,
            FormatArg::P010 => IpaFormat::P010,
        }
    }
}

impl From<BackendArg> for EngineBackend {
    fn from(arg: BackendArg) -> Self {
        match arg {
            BackendArg::MnnVulkan => EngineBackend::MnnVulkan,
            BackendArg::Cpu => EngineBackend::Cpu,
            BackendArg::OnnxRuntime => EngineBackend::OnnxRuntime,
        }
    }
}

/// Subcommands
#[derive(Subcommand, Debug)]
enum Commands {
    /// Generate synthetic test frames and process through ISP
    Generate {
        /// Number of frames to generate
        #[arg(short, long, default_value_t = 1)]
        count: u32,
        /// Write output frame to file
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Pipeline profile preset
        #[arg(long, default_value = "fused", value_enum)]
        profile: ProfileArg,
        /// Output frame width
        #[arg(long, default_value_t = 3840)]
        width: u32,
        /// Output frame height
        #[arg(long, default_value_t = 2160)]
        height: u32,
        /// Output format
        #[arg(long, default_value = "bgra", value_enum)]
        format: FormatArg,
        /// Backend engine
        #[arg(long, default_value = "mnn-vulkan", value_enum)]
        backend: BackendArg,
        /// Enable neural controller (isp-rectifier)
        #[arg(long)]
        neural: bool,
        /// Model directory for neural processor
        #[arg(long)]
        model_dir: Option<String>,
        /// Load profile from JSON file (overrides --profile)
        #[arg(long)]
        profile_json: Option<PathBuf>,
    },
    /// Capture real frames from V4L2 device and process
    Capture {
        /// V4L2 device path (e.g., /dev/video0)
        #[arg(long, default_value = "/dev/video0")]
        device: String,
        /// Number of frames to capture
        #[arg(short, long, default_value_t = 1)]
        count: u32,
        /// Write output frame to file
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Pipeline profile preset
        #[arg(long, default_value = "fused", value_enum)]
        profile: ProfileArg,
        /// Output frame width
        #[arg(long, default_value_t = 3840)]
        width: u32,
        /// Output frame height
        #[arg(long, default_value_t = 2160)]
        height: u32,
        /// Output format
        #[arg(long, default_value = "bgra", value_enum)]
        format: FormatArg,
        /// Backend engine
        #[arg(long, default_value = "mnn-vulkan", value_enum)]
        backend: BackendArg,
        /// Enable neural controller (isp-rectifier)
        #[arg(long)]
        neural: bool,
        /// Model directory for neural processor
        #[arg(long)]
        model_dir: Option<String>,
        /// Load profile from JSON file (overrides --profile)
        #[arg(long)]
        profile_json: Option<PathBuf>,
    },
    /// Benchmark FPS/latency over N frames
    Bench {
        /// Number of frames
        #[arg(short, long, default_value_t = 30)]
        count: u32,
        /// Warmup frames (not timed)
        #[arg(long, default_value_t = 5)]
        warmup: u32,
        /// Pipeline profile preset
        #[arg(long, default_value = "fused", value_enum)]
        profile: ProfileArg,
        /// Output frame width
        #[arg(long, default_value_t = 3840)]
        width: u32,
        /// Output frame height
        #[arg(long, default_value_t = 2160)]
        height: u32,
        /// Output format
        #[arg(long, default_value = "bgra", value_enum)]
        format: FormatArg,
        /// Backend engine
        #[arg(long, default_value = "mnn-vulkan", value_enum)]
        backend: BackendArg,
        /// Enable neural controller (isp-rectifier)
        #[arg(long)]
        neural: bool,
        /// Model directory for neural processor
        #[arg(long)]
        model_dir: Option<String>,
        /// Load profile from JSON file (overrides --profile)
        #[arg(long)]
        profile_json: Option<PathBuf>,
    },
    /// Collect per-frame stats for CMOS sensor characterization
    Stats {
        /// Number of frames
        #[arg(short, long, default_value_t = 100)]
        count: u32,
        /// Write stats to JSON file
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Pipeline profile preset
        #[arg(long, default_value = "fused", value_enum)]
        profile: ProfileArg,
        /// Output frame width
        #[arg(long, default_value_t = 3840)]
        width: u32,
        /// Output frame height
        #[arg(long, default_value_t = 2160)]
        height: u32,
        /// Output format
        #[arg(long, default_value = "bgra", value_enum)]
        format: FormatArg,
        /// Backend engine
        #[arg(long, default_value = "mnn-vulkan", value_enum)]
        backend: BackendArg,
        /// Enable neural controller (isp-rectifier)
        #[arg(long)]
        neural: bool,
        /// Model directory for neural processor
        #[arg(long)]
        model_dir: Option<String>,
        /// Load profile from JSON file (overrides --profile)
        #[arg(long)]
        profile_json: Option<PathBuf>,
    },
    /// Test zero-copy buffer path (memfd on Linux, dmabuf on Android)
    Zerocopy {
        /// Buffer size in bytes
        #[arg(long, default_value_t = 1048576)]
        size: usize,
        /// Number of write/read iterations
        #[arg(long, default_value_t = 1000)]
        iterations: u32,
    },
    /// Probe profile capabilities and engine info
    Probe {
        /// Output as JSON
        #[arg(long)]
        json: bool,
    },
    /// Dump a pipeline profile as JSON, or load custom profile JSON
    ProfileJson {
        /// Profile name (lite, med, fused, heavy, pro, test, unified, hdr, yuv)
        #[arg(long)]
        name: Option<String>,
        /// Output frame width (for custom profile)
        #[arg(long, default_value_t = 3840)]
        width: u32,
        /// Output frame height (for custom profile)
        #[arg(long, default_value_t = 2160)]
        height: u32,
        /// Output file path (if not specified, prints to stdout)
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Load custom profile from JSON file
        #[arg(long)]
        input: Option<PathBuf>,
    },
    /// Read V4L2 sensor metadata and output libcamera-compatible CMOS JSON
    CmosDump {
        /// V4L2 device path (e.g., /dev/video0)
        #[arg(long, default_value = "/dev/video0")]
        device: Option<String>,
        /// Output file path (if not specified, prints to stdout)
        #[arg(short, long)]
        output: Option<PathBuf>,
    },
}

/// Serializable representation of a PipelineProfile for JSON dump/load.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PProfileJson {
    pub label: String,
    pub level: String, // "Lite" | "Medium" | "Heavy" | "Pro"
    pub input_type: String,
    pub use_unpack: bool,
    pub force_input16: bool,
    pub use_fcs: bool,
    pub use_ldci: bool,
    pub use_ee: bool,
    pub use_bad_pixel: bool,
    pub demosaic_quality: String, // "Standard" | "HqLinear" | "Edge"
    pub use_local_contrast: bool,
    pub use_unsharp: bool,
    pub use_lsc: bool,
    pub lsc_strength: f32,
    pub use_warp: bool,
    pub use_hdr: bool,
    pub rotate_mode: i32,
    pub use_zone_stats: bool,
    pub use_channel_means: bool,
    pub use_tone_stats: bool,
    pub use_histogram: bool,
    pub stats_downscale_max: u32,
    pub pipeline_downscale_target: u32,
    pub post_ccm_downscale_w: u32,
    pub post_ccm_downscale_h: u32,
    pub eis_margin: f64,
    pub use_bilateral: bool,
    pub use_saturation: bool,
    pub use_vignetting: bool,
    pub use_colorspace: bool,
    pub use_gamma: bool,
    pub use_sharpen: bool,
    pub use_wavelet_denoise: bool,
    pub use_auto_contrast: bool,
    pub use_normalize: bool,
    pub normalize_at_display: bool,
    pub use_tiled_rendering: bool,
    pub tile_count_x: u32,
    pub tile_count_y: u32,
    pub tile_overlap: u32,
    pub output_format: String, // "PackedRgb" | "Bgra" | ...
    pub opset_mode: i32,       // 0=Primitive, 1=Custom, 2=MnnIspMin
    pub fuse_stages: FuseStagesJson,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuseStagesJson {
    pub unpack_blc_normal_cfa: bool,
    pub unpack16_blc_normal_cfa: bool,
    pub unpack32_blc_normal_cfa: bool,
    pub demosaic_ccm: bool,
    pub wb_into_demosaic: bool,
    pub ccm: bool,
    pub ee: bool,
    pub ldci: bool,
    pub tone: bool,
    pub tonelut: bool,
    pub display: bool,
    pub tail_downscale: bool,
    pub whole_pipeline: bool,
}

impl From<&PipelineProfile> for PProfileJson {
    fn from(p: &PipelineProfile) -> Self {
        Self {
            label: p.label.to_string(),
            level: match p.level {
                cam_isp::profile::PipelineLevel::Lite => "Lite".to_string(),
                cam_isp::profile::PipelineLevel::Medium => "Medium".to_string(),
                cam_isp::profile::PipelineLevel::Heavy => "Heavy".to_string(),
                cam_isp::profile::PipelineLevel::Pro => "Pro".to_string(),
            },
            input_type: p.input_type.to_string(),
            use_unpack: p.use_unpack,
            force_input16: p.force_input16,
            use_fcs: p.use_fcs,
            use_ldci: p.use_ldci,
            use_ee: p.use_ee,
            use_bad_pixel: p.use_bad_pixel,
            demosaic_quality: match p.demosaic_quality {
                cam_isp::profile::DemosaicQuality::Standard => "Standard".to_string(),
                cam_isp::profile::DemosaicQuality::HqLinear => "HqLinear".to_string(),
                cam_isp::profile::DemosaicQuality::Edge => "Edge".to_string(),
            },
            use_local_contrast: p.use_local_contrast,
            use_unsharp: p.use_unsharp,
            use_lsc: p.use_lsc,
            lsc_strength: p.lsc_strength,
            use_warp: p.use_warp,
            use_hdr: p.use_hdr,
            rotate_mode: p.rotate_mode,
            use_zone_stats: p.use_zone_stats,
            use_channel_means: p.use_channel_means,
            use_tone_stats: p.use_tone_stats,
            use_histogram: p.use_histogram,
            stats_downscale_max: p.stats_downscale_max,
            pipeline_downscale_target: p.pipeline_downscale_target,
            post_ccm_downscale_w: p.post_ccm_downscale.0,
            post_ccm_downscale_h: p.post_ccm_downscale.1,
            eis_margin: p.eis_margin,
            use_bilateral: p.use_bilateral,
            use_saturation: p.use_saturation,
            use_vignetting: p.use_vignetting,
            use_colorspace: p.use_colorspace,
            use_gamma: p.use_gamma,
            use_sharpen: p.use_sharpen,
            use_wavelet_denoise: p.use_wavelet_denoise,
            use_auto_contrast: p.use_auto_contrast,
            use_normalize: p.use_normalize,
            normalize_at_display: p.normalize_at_display,
            use_tiled_rendering: p.use_tiled_rendering,
            tile_count_x: p.tile_count_x,
            tile_count_y: p.tile_count_y,
            tile_overlap: p.tile_overlap,
            output_format: match p.output_format {
                cam_isp::engine::OutputFormat::PackedRgb => "PackedRgb".to_string(),
                cam_isp::engine::OutputFormat::Bgra => "Bgra".to_string(),
                cam_isp::engine::OutputFormat::Rgba => "Rgba".to_string(),
                cam_isp::engine::OutputFormat::Argb => "Argb".to_string(),
                cam_isp::engine::OutputFormat::Abgr => "Abgr".to_string(),
                cam_isp::engine::OutputFormat::Rgb => "Rgb".to_string(),
                cam_isp::engine::OutputFormat::Bgr => "Bgr".to_string(),
                cam_isp::engine::OutputFormat::FloatRgb => "FloatRgb".to_string(),
                cam_isp::engine::OutputFormat::FloatBgra => "FloatBgra".to_string(),
                cam_isp::engine::OutputFormat::Float16Rgb => "Float16Rgb".to_string(),
                cam_isp::engine::OutputFormat::Float16Bgra => "Float16Bgra".to_string(),
                _ => "Unknown".to_string(),
            },
            opset_mode: p.opset_mode,
            fuse_stages: FuseStagesJson {
                unpack_blc_normal_cfa: p.fuse_stages.unpack_blc_normal_cfa,
                unpack16_blc_normal_cfa: p.fuse_stages.unpack16_blc_normal_cfa,
                unpack32_blc_normal_cfa: p.fuse_stages.unpack32_blc_normal_cfa,
                demosaic_ccm: p.fuse_stages.demosaic_ccm,
                wb_into_demosaic: p.fuse_stages.wb_into_demosaic,
                ccm: p.fuse_stages.ccm,
                ee: p.fuse_stages.ee,
                ldci: p.fuse_stages.ldci,
                tone: p.fuse_stages.tone,
                tonelut: p.fuse_stages.tonelut,
                display: p.fuse_stages.display,
                tail_downscale: p.fuse_stages.tail_downscale,
                whole_pipeline: p.fuse_stages.whole_pipeline,
            },
        }
    }
}

impl PProfileJson {
    /// Convert back to a PipelineProfile for use in generate/bench/stats/capture.
    pub fn to_pipeline_profile(&self) -> PipelineProfile {
        // PipelineProfile::custom requires &'static str for label and input_type.
        // Use Box::leak to create static strings (acceptable for a CLI tool).
        let label: &'static str = Box::leak(self.label.clone().into_boxed_str());
        let input_type: &'static str = Box::leak(self.input_type.clone().into_boxed_str());
        let mut p = PipelineProfile::custom(
            label,
            match self.level.as_str() {
                "Lite" => cam_isp::profile::PipelineLevel::Lite,
                "Medium" => cam_isp::profile::PipelineLevel::Medium,
                "Heavy" => cam_isp::profile::PipelineLevel::Heavy,
                "Pro" => cam_isp::profile::PipelineLevel::Pro,
                _ => cam_isp::profile::PipelineLevel::Lite,
            },
            input_type,
            self.use_unpack,
            self.use_fcs,
            self.use_ldci,
            self.use_ee,
            self.use_bad_pixel,
            match self.demosaic_quality.as_str() {
                "Standard" => cam_isp::profile::DemosaicQuality::Standard,
                "HqLinear" => cam_isp::profile::DemosaicQuality::HqLinear,
                "Edge" => cam_isp::profile::DemosaicQuality::Edge,
                _ => cam_isp::profile::DemosaicQuality::Standard,
            },
            self.use_local_contrast,
            self.use_unsharp,
            self.use_lsc,
            self.use_warp,
            self.use_hdr,
            self.rotate_mode,
            self.use_zone_stats,
            self.use_channel_means,
            self.use_tone_stats,
            self.use_histogram,
            self.stats_downscale_max,
            self.pipeline_downscale_target,
            (self.post_ccm_downscale_w, self.post_ccm_downscale_h),
            self.eis_margin,
            self.use_bilateral,
            self.use_saturation,
            self.use_vignetting,
            self.use_colorspace,
            self.use_gamma,
            self.use_sharpen,
            self.use_wavelet_denoise,
            self.use_auto_contrast,
            self.use_normalize,
            self.normalize_at_display,
            self.use_tiled_rendering,
            self.tile_count_x,
            self.tile_count_y,
            self.tile_overlap,
            self.opset_mode,
            cam_isp::profile::FuseStages {
                unpack_blc_normal_cfa: self.fuse_stages.unpack_blc_normal_cfa,
                unpack16_blc_normal_cfa: self.fuse_stages.unpack16_blc_normal_cfa,
                unpack32_blc_normal_cfa: self.fuse_stages.unpack32_blc_normal_cfa,
                demosaic_ccm: self.fuse_stages.demosaic_ccm,
                wb_into_demosaic: self.fuse_stages.wb_into_demosaic,
                ccm: self.fuse_stages.ccm,
                ee: self.fuse_stages.ee,
                ldci: self.fuse_stages.ldci,
                tone: self.fuse_stages.tone,
                tonelut: self.fuse_stages.tonelut,
                display: self.fuse_stages.display,
                tail_downscale: self.fuse_stages.tail_downscale,
                whole_pipeline: self.fuse_stages.whole_pipeline,
            },
        );
        p.lsc_strength = self.lsc_strength;
        p
    }
}

// ============================================================================
// Library entry points
// ============================================================================

/// Library entry point — callable via dlopen
#[no_mangle]
pub extern "C" fn ipa_sk_main() -> i32 {
    run_cli()
}

/// ELF entry point — allows the cdylib to be executed directly
#[no_mangle]
pub extern "C" fn my_entry() -> ! {
    std::process::exit(run_cli())
}

fn run_cli() -> i32 {
    let args = Args::parse();
    env_logger::init();

    match args.command {
        Some(Commands::Generate {
            count,
            output,
            profile,
            width,
            height,
            format,
            backend,
            neural,
            model_dir,
            profile_json,
        }) => {
            println!("=== cam-ipa Swiss Army Knife ===");
            println!("Command: generate");
            let config = if let Some(pj_path) = &profile_json {
                let pj: PProfileJson = serde_json::from_str(
                    &std::fs::read_to_string(pj_path).expect("failed to read profile JSON"),
                )
                .expect("failed to parse profile JSON");
                let pipeline_profile = pj.to_pipeline_profile();
                IpaConfig {
                    image_width: width,
                    image_height: height,
                    format: format.into(),
                    num_output_buffers: 2,
                    backend: backend.into(),
                    profile: pipeline_profile,
                    neural_enabled: neural,
                    model_dir: model_dir.map(PathBuf::from),
                    ..IpaConfig::default()
                }
            } else {
                build_ipa_config(profile, width, height, format, backend, neural, model_dir)
            };
            println!(
                "Profile: {:?}  Res: {}x{}  Format: {:?}  Backend: {:?}  Neural: {}",
                config.profile, width, height, format, backend, neural
            );
            cmd_generate(&config, count, output)
        }
        Some(Commands::Capture {
            device,
            count,
            output,
            profile,
            width,
            height,
            format,
            backend,
            neural,
            model_dir,
            profile_json,
        }) => {
            println!("=== cam-ipa Swiss Army Knife ===");
            println!("Command: capture");
            let config = if let Some(pj_path) = &profile_json {
                let pj: PProfileJson = serde_json::from_str(
                    &std::fs::read_to_string(pj_path).expect("failed to read profile JSON"),
                )
                .expect("failed to parse profile JSON");
                let pipeline_profile = pj.to_pipeline_profile();
                IpaConfig {
                    image_width: width,
                    image_height: height,
                    format: format.into(),
                    num_output_buffers: 2,
                    backend: backend.into(),
                    profile: pipeline_profile,
                    neural_enabled: neural,
                    model_dir: model_dir.map(PathBuf::from),
                    ..IpaConfig::default()
                }
            } else {
                build_ipa_config(profile, width, height, format, backend, neural, model_dir)
            };
            println!(
                "Profile: {:?}  Res: {}x{}  Format: {:?}  Backend: {:?}  Neural: {}",
                config.profile, width, height, format, backend, neural
            );
            cmd_capture(&config, device, count, output)
        }
        Some(Commands::Bench {
            count,
            warmup,
            profile,
            width,
            height,
            format,
            backend,
            neural,
            model_dir,
            profile_json,
        }) => {
            println!("=== cam-ipa Swiss Army Knife ===");
            println!("Command: bench");
            let config = if let Some(pj_path) = &profile_json {
                let pj: PProfileJson = serde_json::from_str(
                    &std::fs::read_to_string(pj_path).expect("failed to read profile JSON"),
                )
                .expect("failed to parse profile JSON");
                let pipeline_profile = pj.to_pipeline_profile();
                IpaConfig {
                    image_width: width,
                    image_height: height,
                    format: format.into(),
                    num_output_buffers: 2,
                    backend: backend.into(),
                    profile: pipeline_profile,
                    neural_enabled: neural,
                    model_dir: model_dir.map(PathBuf::from),
                    ..IpaConfig::default()
                }
            } else {
                build_ipa_config(profile, width, height, format, backend, neural, model_dir)
            };
            println!(
                "Profile: {:?}  Res: {}x{}  Format: {:?}  Backend: {:?}  Neural: {}",
                config.profile, width, height, format, backend, neural
            );
            cmd_bench(&config, count, warmup)
        }
        Some(Commands::Stats {
            count,
            output,
            profile,
            width,
            height,
            format,
            backend,
            neural,
            model_dir,
            profile_json,
        }) => {
            println!("=== cam-ipa Swiss Army Knife ===");
            println!("Command: stats");
            let config = if let Some(pj_path) = &profile_json {
                let pj: PProfileJson = serde_json::from_str(
                    &std::fs::read_to_string(pj_path).expect("failed to read profile JSON"),
                )
                .expect("failed to parse profile JSON");
                let pipeline_profile = pj.to_pipeline_profile();
                IpaConfig {
                    image_width: width,
                    image_height: height,
                    format: format.into(),
                    num_output_buffers: 2,
                    backend: backend.into(),
                    profile: pipeline_profile,
                    neural_enabled: neural,
                    model_dir: model_dir.map(PathBuf::from),
                    ..IpaConfig::default()
                }
            } else {
                build_ipa_config(profile, width, height, format, backend, neural, model_dir)
            };
            println!(
                "Profile: {:?}  Res: {}x{}  Format: {:?}  Backend: {:?}  Neural: {}",
                config.profile, width, height, format, backend, neural
            );
            cmd_stats(&config, count, output)
        }
        Some(Commands::Zerocopy { size, iterations }) => cmd_zerocopy(size, iterations),
        Some(Commands::Probe { json }) => cmd_probe(json),
        Some(Commands::ProfileJson {
            name,
            width,
            height,
            output,
            input,
        }) => {
            let input_path = input.map(|p| p.to_string_lossy().to_string());
            cmd_profile_json(name, width, height, output, input_path.as_deref())
        }
        Some(Commands::CmosDump { device, output }) => {
            let dev = device.as_deref();
            cmd_cmos_dump(dev, output.as_ref())
        }
        None => {
            println!("No subcommand given. Run 'ipa_sk --help' for usage.");
            cmd_probe(false)
        }
    }
}

/// Build an IpaConfig from CLI options
fn build_ipa_config(
    profile: ProfileArg,
    width: u32,
    height: u32,
    format: FormatArg,
    backend: BackendArg,
    neural: bool,
    model_dir: Option<String>,
) -> IpaConfig {
    IpaConfig {
        image_width: width,
        image_height: height,
        format: format.into(),
        num_output_buffers: 2,
        backend: backend.into(),
        profile: profile.into(),
        neural_enabled: neural,
        model_dir: model_dir.map(PathBuf::from),
        ..IpaConfig::default()
    }
}

/// Build an IpaConfig from a PProfileJson (for --profile-json)
fn build_ipa_config_from_json(
    pj: &PProfileJson,
    width: u32,
    height: u32,
    format: FormatArg,
    backend: BackendArg,
    neural: bool,
    model_dir: Option<String>,
) -> IpaConfig {
    let pipeline_profile = pj.to_pipeline_profile();
    IpaConfig {
        image_width: width,
        image_height: height,
        format: format.into(),
        num_output_buffers: 2,
        backend: backend.into(),
        profile: pipeline_profile,
        neural_enabled: neural,
        model_dir: model_dir.map(PathBuf::from),
        ..IpaConfig::default()
    }
}

// ============================================================================
// cmd_generate — synthetic frame generation + ISP processing
// ============================================================================

fn cmd_generate(config: &IpaConfig, count: u32, output: Option<PathBuf>) -> i32 {
    println!("\n--- Generate mode: {} synthetic frames ---", count);

    let mut ctx = match IpaContext::new(config.clone()) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to create IPA context: {}", e);
            return 1;
        }
    };

    let sensor_info = IpaSensorInfo {
        model: "synthetic_sensor".to_string(),
        bits_per_pixel: 10,
        cfa_pattern: 0,
        active_area_size: Size {
            width: config.image_width,
            height: config.image_height,
        },
        analog_crop: Rectangle {
            x: 0,
            y: 0,
            width: config.image_width,
            height: config.image_height,
        },
        output_size: Size {
            width: config.image_width,
            height: config.image_height,
        },
        pixel_rate: 0,
        min_line_length: 0,
        max_line_length: 0,
        min_frame_length: 0,
        max_frame_length: 0,
    };

    let settings = IpaSettings {
        config_file: None,
        sensor_model: "synthetic_sensor".to_string(),
    };
    if let Err(e) = ctx.init(&settings, &sensor_info) {
        eprintln!("IPA init failed: {}", e);
        return 1;
    }
    if let Err(e) = ctx.start() {
        eprintln!("IPA start failed: {}", e);
        return 1;
    }

    let w = config.image_width;
    let h = config.image_height;
    let input_size = (w * h * 2) as usize;

    for frame in 1..=count {
        // Generate synthetic Bayer frame
        let mut input_data = vec![0u8; input_size];
        for (i, chunk) in input_data.chunks_mut(2).enumerate() {
            let val = (i as u16).wrapping_add((frame as u16).wrapping_mul(7));
            chunk[0] = val as u8;
            chunk[1] = (val >> 8) as u8;
        }

        let mut input_buf = MemBuffer::new(input_data.len());
        input_buf.data.copy_from_slice(&input_data);

        let output_size = (w * h * 4) as usize;
        let mut output_buf = MemBuffer::new(output_size);

        let mut controls = ControlList::new();
        controls.set_f32("AnalogueGain", 2.0);
        controls.set_i32("ExposureTime", 2000);
        controls.set_f32_array("ColourGains", &[1.5, 1.2]);

        if let Err(e) = ctx.queue_request(frame, &controls) {
            eprintln!("queue_request(frame {}) failed: {}", frame, e);
            continue;
        }
        if let Err(e) = ctx.compute_params(frame) {
            eprintln!("compute_params(frame {}) failed: {}", frame, e);
            continue;
        }

        let stats_buf = MemBuffer::new(((w / 4) * (h / 4) * 4) as usize);
        if let Err(e) = ctx.process_stats(frame, frame, &stats_buf) {
            eprintln!("process_stats(frame {}) failed: {}", frame, e);
            continue;
        }

        if let Err(e) = ctx.process_frame(
            frame,
            frame,
            &mut input_buf,
            &mut output_buf,
            0,
            0,
            w,
            h,
            &controls,
        ) {
            eprintln!("process_frame(frame {}) failed: {}", frame, e);
            continue;
        }

        if let Some(ref path) = output {
            if let Err(e) = std::fs::write(path, &output_buf.data) {
                eprintln!("Failed to write output frame: {}", e);
            } else {
                println!(
                    "Frame {} written to {} ({} bytes)",
                    frame,
                    path.display(),
                    output_buf.data.len()
                );
            }
        } else {
            println!(
                "Frame {} processed: {} bytes output",
                frame,
                output_buf.len()
            );
        }
    }

    ctx.stop();
    println!("Generate complete");
    0
}

// ============================================================================
// cmd_capture — V4L2 capture + ISP processing
// ============================================================================

fn cmd_capture(config: &IpaConfig, device: String, count: u32, output: Option<PathBuf>) -> i32 {
    println!("\n--- Capture mode: {} frames from {} ---", count, device);

    #[cfg(feature = "v4l2")]
    {
        let devices = cam_hal_linux::list_v4l2_devices();
        if !devices.iter().any(|d| d == &device) {
            eprintln!("V4L2 device {} not found. Available: {:?}", device, devices);
            return 1;
        }

        let mut ctx = match IpaContext::new(config.clone()) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("Failed to create IPA context: {}", e);
                return 1;
            }
        };

        let sensor_info = IpaSensorInfo {
            model: "v4l2_sensor".to_string(),
            bits_per_pixel: 10,
            cfa_pattern: 0,
            active_area_size: Size {
                width: config.image_width,
                height: config.image_height,
            },
            analog_crop: Rectangle {
                x: 0,
                y: 0,
                width: config.image_width,
                height: config.image_height,
            },
            output_size: Size {
                width: config.image_width,
                height: config.image_height,
            },
            pixel_rate: 0,
            min_line_length: 0,
            max_line_length: 0,
            min_frame_length: 0,
            max_frame_length: 0,
        };

        let settings = IpaSettings {
            config_file: None,
            sensor_model: "v4l2_sensor".to_string(),
        };
        if let Err(e) = ctx.init(&settings, &sensor_info) {
            eprintln!("IPA init failed: {}", e);
            return 1;
        }
        if let Err(e) = ctx.start() {
            eprintln!("IPA start failed: {}", e);
            return 1;
        }

        let w = config.image_width;
        let h = config.image_height;

        for frame in 1..=count {
            match cam_hal_linux::capture_single_v4l2_frame(&device, w, h) {
                Ok((cw, ch, data)) => {
                    println!(
                        "Captured {}x{} frame {} ({} bytes)",
                        cw,
                        ch,
                        frame,
                        data.len()
                    );

                    let mut input_buf = MemBuffer::new(data.len());
                    input_buf.data.copy_from_slice(&data);

                    let output_size = (cw * ch * 4) as usize;
                    let mut output_buf = MemBuffer::new(output_size);

                    let mut controls = ControlList::new();
                    controls.set_f32("AnalogueGain", 2.0);
                    controls.set_i32("ExposureTime", 2000);

                    if let Err(e) = ctx.queue_request(frame, &controls) {
                        eprintln!("queue_request(frame {}) failed: {}", frame, e);
                        continue;
                    }
                    if let Err(e) = ctx.process_frame(
                        frame,
                        frame,
                        &mut input_buf,
                        &mut output_buf,
                        0,
                        0,
                        cw,
                        ch,
                        &controls,
                    ) {
                        eprintln!("process_frame(frame {}) failed: {}", frame, e);
                        continue;
                    }

                    if let Some(ref path) = output {
                        let fname = format!("{}_{}.bgra", path.display(), frame);
                        if let Err(e) = std::fs::write(&fname, &output_buf.data) {
                            eprintln!("Failed to write {}: {}", fname, e);
                        } else {
                            println!(
                                "Frame {} written to {} ({} bytes)",
                                frame,
                                fname,
                                output_buf.data.len()
                            );
                        }
                    }
                }
                Err(e) => {
                    eprintln!("V4L2 capture frame {} failed: {}", frame, e);
                }
            }
        }

        ctx.stop();
        println!("Capture complete");
        0
    }
    #[cfg(not(feature = "v4l2"))]
    {
        eprintln!(
            "V4L2 feature not enabled. Rebuild with: cargo build -p cam-ipa-sk --features v4l2"
        );
        1
    }
}

// ============================================================================
// cmd_bench — FPS/latency benchmark
// ============================================================================

fn cmd_bench(config: &IpaConfig, count: u32, warmup: u32) -> i32 {
    println!("\n--- Bench mode: {} frames ({} warmup) ---", count, warmup);

    let mut ctx = match IpaContext::new(config.clone()) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to create IPA context: {}", e);
            return 1;
        }
    };

    let sensor_info = IpaSensorInfo {
        model: "bench_sensor".to_string(),
        bits_per_pixel: 10,
        cfa_pattern: 0,
        active_area_size: Size {
            width: config.image_width,
            height: config.image_height,
        },
        analog_crop: Rectangle {
            x: 0,
            y: 0,
            width: config.image_width,
            height: config.image_height,
        },
        output_size: Size {
            width: config.image_width,
            height: config.image_height,
        },
        pixel_rate: 0,
        min_line_length: 0,
        max_line_length: 0,
        min_frame_length: 0,
        max_frame_length: 0,
    };

    let settings = IpaSettings {
        config_file: None,
        sensor_model: "bench_sensor".to_string(),
    };
    if let Err(e) = ctx.init(&settings, &sensor_info) {
        eprintln!("IPA init failed: {}", e);
        return 1;
    }
    if let Err(e) = ctx.start() {
        eprintln!("IPA start failed: {}", e);
        return 1;
    }

    let w = config.image_width;
    let h = config.image_height;
    let input_size = (w * h * 2) as usize;
    let output_size = (w * h * 4) as usize;

    // Pre-allocate buffers
    let mut input_buf = MemBuffer::new(input_size);
    for (i, b) in input_buf.data.iter_mut().enumerate() {
        *b = (i % 256) as u8;
    }
    let mut output_buf = MemBuffer::new(output_size);

    let mut controls = ControlList::new();
    controls.set_f32("AnalogueGain", 2.0);
    controls.set_i32("ExposureTime", 2000);
    controls.set_f32_array("ColourGains", &[1.5, 1.2]);

    let mut latencies: Vec<u64> = Vec::new();
    let total_start = Instant::now();

    for frame in 1..=(warmup + count) {
        let frame_start = Instant::now();

        if let Err(e) = ctx.process_frame(
            frame,
            frame,
            &mut input_buf,
            &mut output_buf,
            0,
            0,
            w,
            h,
            &controls,
        ) {
            eprintln!("process_frame(frame {}) failed: {}", frame, e);
            continue;
        }

        let frame_elapsed = frame_start.elapsed().as_micros() as u64;
        if frame > warmup {
            latencies.push(frame_elapsed);
        }

        if frame == warmup {
            println!("Warmup complete, starting timed run...");
        }
        if frame % 10 == 0 || frame <= warmup + 5 {
            println!("Frame {}: {} us", frame, frame_elapsed);
        }
    }

    let total_elapsed = total_start.elapsed();
    let timed_count = latencies.len() as u64;

    if timed_count > 0 {
        let avg_us: u64 = latencies.iter().sum::<u64>() / timed_count;
        let max_us = latencies.iter().max().copied().unwrap_or(0);
        let min_us = latencies.iter().min().copied().unwrap_or(0);
        let fps = (timed_count as f64)
            / (total_elapsed.as_secs_f64() - (warmup as f64 * avg_us as f64 / 1_000_000.0));

        println!("\n--- Bench results ---");
        println!("Timed frames: {}", timed_count);
        println!("Avg latency: {} us", avg_us);
        println!("Min latency: {} us", min_us);
        println!("Max latency: {} us", max_us);
        println!("Approx FPS: {:.2}", fps);
        println!("Resolution: {}x{}", w, h);
        println!("Profile: {:?}", config.profile);
    }

    ctx.stop();
    0
}

// ============================================================================
// cmd_stats — collect per-frame stats for CMOS characterization
// ============================================================================

fn cmd_stats(config: &IpaConfig, count: u32, output: Option<PathBuf>) -> i32 {
    println!("\n--- Stats mode: {} frames ---", count);

    let mut ctx = match IpaContext::new(config.clone()) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to create IPA context: {}", e);
            return 1;
        }
    };

    let sensor_info = IpaSensorInfo {
        model: "stats_sensor".to_string(),
        bits_per_pixel: 10,
        cfa_pattern: 0,
        active_area_size: Size {
            width: config.image_width,
            height: config.image_height,
        },
        analog_crop: Rectangle {
            x: 0,
            y: 0,
            width: config.image_width,
            height: config.image_height,
        },
        output_size: Size {
            width: config.image_width,
            height: config.image_height,
        },
        pixel_rate: 0,
        min_line_length: 0,
        max_line_length: 0,
        min_frame_length: 0,
        max_frame_length: 0,
    };

    let settings = IpaSettings {
        config_file: None,
        sensor_model: "stats_sensor".to_string(),
    };
    if let Err(e) = ctx.init(&settings, &sensor_info) {
        eprintln!("IPA init failed: {}", e);
        return 1;
    }
    if let Err(e) = ctx.start() {
        eprintln!("IPA start failed: {}", e);
        return 1;
    }

    let w = config.image_width;
    let h = config.image_height;
    let input_size = (w * h * 2) as usize;

    let mut all_stats: Vec<serde_json::Value> = Vec::new();

    for frame in 1..=count {
        let mut input_data = vec![0u8; input_size];
        for (i, chunk) in input_data.chunks_mut(2).enumerate() {
            let val = (i as u16).wrapping_add((frame as u16).wrapping_mul(13));
            chunk[0] = val as u8;
            chunk[1] = (val >> 8) as u8;
        }

        let mut input_buf = MemBuffer::new(input_data.len());
        input_buf.data.copy_from_slice(&input_data);

        let mut output_buf = MemBuffer::new((w * h * 4) as usize);

        let mut controls = ControlList::new();
        controls.set_f32("AnalogueGain", 2.0);
        controls.set_i32("ExposureTime", 2000);
        controls.set_f32_array("ColourGains", &[1.5, 1.2]);

        if let Err(e) = ctx.queue_request(frame, &controls) {
            eprintln!("queue_request(frame {}) failed: {}", frame, e);
            continue;
        }
        if let Err(e) = ctx.compute_params(frame) {
            eprintln!("compute_params(frame {}) failed: {}", frame, e);
            continue;
        }

        let stats_buf = MemBuffer::new(((w / 4) * (h / 4) * 4) as usize);
        match ctx.process_stats(frame, frame, &stats_buf) {
            Ok(metadata) => {
                let stats_entry = serde_json::json!({
                    "frame": frame,
                    "width": w,
                    "height": h,
                    "metadata": metadata.data,
                });
                all_stats.push(stats_entry);
                if frame % 10 == 0 {
                    println!("Frame {} stats collected", frame);
                }
            }
            Err(e) => {
                eprintln!("process_stats(frame {}) failed: {}", frame, e);
            }
        }
    }

    ctx.stop();

    if let Some(ref path) = output {
        let json = serde_json::to_string_pretty(&all_stats).unwrap_or_default();
        if let Err(e) = std::fs::write(path, &json) {
            eprintln!("Failed to write stats to {}: {}", path.display(), e);
            return 1;
        }
        println!(
            "Stats written to {} ({} frames)",
            path.display(),
            all_stats.len()
        );
    } else {
        println!("Collected stats from {} frames", all_stats.len());
    }

    println!("Stats complete");
    0
}

// ============================================================================
// cmd_zerocopy — test memfd/dmabuf zero-copy buffer path
// ============================================================================

fn cmd_zerocopy(size: usize, iterations: u32) -> i32 {
    println!(
        "\n--- Zerocopy mode: {} iterations, {} bytes ---",
        iterations, size
    );

    #[cfg(not(target_os = "android"))]
    {
        use cam_ipa::buffer::create_memfd_dmabuf;
        use std::os::fd::AsRawFd;

        let (fd, mut buf) = match create_memfd_dmabuf(size) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("Failed to create memfd dmabuf: {}", e);
                return 1;
            }
        };
        println!(
            "Created memfd dmabuf: fd={} size={} ptr={:?}",
            fd,
            buf.len(),
            buf.as_ptr()
        );

        let mut write_sum: u8 = 0;
        let start = Instant::now();

        for i in 0..iterations {
            // Write pattern through mmap'd pointer
            let val = (i % 256) as u8;
            for chunk in buf.data.chunks_mut(8) {
                for b in chunk.iter_mut() {
                    *b = val;
                }
            }
            write_sum = write_sum.wrapping_add(val);

            // Read back
            let read_val = unsafe { buf.as_ptr().read() };
            if read_val != val {
                eprintln!(
                    "Zerocopy mismatch at iter {}: wrote {} read {}",
                    i, val, read_val
                );
                return 1;
            }
        }

        let elapsed = start.elapsed();
        let per_iter = elapsed.as_nanos() / iterations as u128;

        println!("Zerocopy test: {} iterations in {:?}", iterations, elapsed);
        println!("Per iteration: {} ns", per_iter);
        println!("Write sum: {}", write_sum);
        println!("Memfd fd: {}", fd);
        println!("Zerocopy test PASSED");
        0
    }
    #[cfg(target_os = "android")]
    {
        eprintln!("Zerocopy memfd test not available on Android (use dmabuf path instead)");
        // Test MemBuffer as fallback
        let mut buf = MemBuffer::new(size);
        let start = Instant::now();
        for i in 0..iterations {
            let val = (i % 256) as u8;
            for b in buf.data.iter_mut() {
                *b = val;
            }
        }
        let elapsed = start.elapsed();
        println!(
            "MemBuffer fallback: {} iterations in {:?}",
            iterations, elapsed
        );
        0
    }
}

// ============================================================================
// cmd_probe — probe profile capabilities and engine info
// ============================================================================

fn cmd_probe(json: bool) -> i32 {
    if json {
        // JSON output of all profiles
        let profiles: Vec<PProfileJson> = cam_isp::profile::PipelineProfile::ALL
            .iter()
            .map(PProfileJson::from)
            .collect();
        let output = serde_json::json!({
            "profiles": profiles,
            "engines": ["MnnVulkan", "Cpu", "OnnxRuntime"],
            "buffers": ["DmaBufBuffer", "AhwBuffer", "MemBuffer"],
            "features": {
                "neural_enabled": "opt-in NeuralController (isp-rectifier)",
                "zero_copy": "memfd_create + mmap on Linux, SharedFD on Android"
            }
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
        0
    } else {
        println!("\n--- Probe mode ---");

        println!("\nAvailable profiles:");
        for p in cam_isp::profile::PipelineProfile::ALL.iter() {
            println!(
                "  {:?}: level={:?} input={} unpack={} fused_stages={:?}",
                p.label, p.level, p.input_type, p.use_unpack, p.fuse_stages
            );
        }

        println!("\nEngine backends:");
        println!("  MnnVulkan (priority 99)");
        println!("  Cpu (not yet implemented)");
        println!("  OnnxRuntime (not yet implemented)");

        println!("\nBuffer types:");
        println!("  DmaBufBuffer (Linux memfd/dmabuf mmap + Android SharedFD)");
        println!("  AhwBuffer (Android AHardwareBuffer, Vulkan input_ahb/output_ahb)");
        println!("  MemBuffer (host memory, testing)");

        println!("\ncam-ipa features:");
        println!("  neural_enabled: opt-in NeuralController (isp-rectifier)");
        println!("  zero-copy: memfd_create + mmap on Linux, SharedFD on Android");

        0
    }
}

// ============================================================================
// cmd_profile_json — dump a single profile as JSON, or load custom profile JSON
// ============================================================================

fn cmd_profile_json(
    name: Option<String>,
    width: u32,
    height: u32,
    output: Option<PathBuf>,
    input: Option<&str>,
) -> i32 {
    if let Some(input_path_str) = input {
        // Load custom profile JSON and optionally save/validate
        let json_str = match std::fs::read_to_string(input_path_str) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to read {}: {}", input_path_str, e);
                return 1;
            }
        };
        let pj: PProfileJson = match serde_json::from_str(&json_str) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Failed to parse profile JSON: {}", e);
                return 1;
            }
        };
        let _profile = pj.to_pipeline_profile();
        println!(
            "Loaded custom profile '{}' from {}",
            pj.label, input_path_str
        );
        println!(
            "  width={} height={} unpack={} fused={:?}",
            width, height, pj.use_unpack, pj.fuse_stages
        );
        if let Some(out_path) = output {
            let out_json = serde_json::to_string_pretty(&pj).unwrap();
            let out_path_clone = out_path.clone();
            if let Err(e) = std::fs::write(out_path, &out_json) {
                eprintln!("Failed to write {}: {}", out_path_clone.display(), e);
                return 1;
            }
            println!("Profile JSON written to {}", out_path_clone.display());
        }
        0
    } else {
        // Dump a built-in profile as JSON
        let profile = match name.as_deref() {
            Some("lite") => PipelineProfile::LITE,
            Some("med") => PipelineProfile::MED,
            Some("fused") => PipelineProfile::FUSED,
            Some("heavy") => PipelineProfile::HEAVY,
            Some("pro") => PipelineProfile::PRO,
            Some("test") => PipelineProfile::TEST,
            Some("unified") => PipelineProfile::UNIFIED,
            Some("hdr") => PipelineProfile::HDR,
            Some("yuv") => PipelineProfile::YUV,
            _ => {
                eprintln!("Unknown profile '{}'. Available: lite, med, fused, heavy, pro, test, unified, hdr, yuv",
                    name.as_deref().unwrap_or(""));
                return 1;
            }
        };
        let pj = PProfileJson::from(&profile);
        let json = serde_json::to_string_pretty(&pj).unwrap();
        if let Some(out_path) = output {
            let out_path_clone = out_path.clone();
            if let Err(e) = std::fs::write(out_path, &json) {
                eprintln!("Failed to write {}: {}", out_path_clone.display(), e);
                return 1;
            }
            println!(
                "Profile '{}' written to {}",
                pj.label,
                out_path_clone.display()
            );
        } else {
            println!("{}", json);
        }
        0
    }
}

// ============================================================================
// cmd_cmos_dump — read V4L2 sensor metadata and output libcamera-compatible CMOS JSON
// ============================================================================

fn cmd_cmos_dump(device: Option<&str>, output: Option<&PathBuf>) -> i32 {
    println!("\n--- CMOS character dump ---");

    let dev_str = device.unwrap_or("/dev/video0");

    #[cfg(feature = "v4l2")]
    {
        // Try to read V4L2 device info
        let devices = cam_hal_linux::list_v4l2_devices();
        if !devices.iter().any(|d| d == dev_str) {
            eprintln!(
                "V4L2 device {} not found. Available: {:?}",
                dev_str, devices
            );
            return 1;
        }

        // Build CMOS JSON from V4L2 info (basic fields; full libcamera CMOS JSON
        // would require libv4l2 ioctl queries for sensor model, pixel format, etc.)
        let cmos_json = serde_json::json!({
            "sensor": {
                "name": format!("v4l2_sensor_{}", dev_str),
                "model": "unknown",
                "pixel_format": "unknown",
                "bits_per_pixel": 10,
                "color_format": "SRGGB10",
                "active_area": {
                    "left": 0,
                    "top": 0,
                    "width": 1920,
                    "height": 1080
                },
                "pixel_array": {
                    "width": 1920,
                    "height": 1080
                },
                "link_frequency": 0,
                "pixel_rate": 0,
                "exposure_range": [1, 1000000],
                "analog_gain_range": [1.0, 16.0],
                "digital_gain_range": [1.0, 16.0],
                "frame_rate_range": [1.0, 30.0],
                "controls": {
                    "exposure_time": { "min": 1, "max": 1000000, "default": 2000 },
                    "analogue_gain": { "min": 1.0, "max": 16.0, "default": 1.0 },
                    "colour_gains": { "min": [0.1, 0.1], "max": [16.0, 16.0], "default": [1.0, 1.0] }
                }
            },
            "source": "v4l2",
            "device": dev_str
        });

        let json_str = serde_json::to_string_pretty(&cmos_json).unwrap();
        if let Some(out_path) = output {
            if let Err(e) = std::fs::write(out_path, &json_str) {
                eprintln!("Failed to write {}: {}", out_path.display(), e);
                return 1;
            }
            println!("CMOS character dump written to {}", out_path.display());
        } else {
            println!("{}", json_str);
        }
        0
    }
    #[cfg(not(feature = "v4l2"))]
    {
        eprintln!(
            "V4L2 feature not enabled. Rebuild with: cargo build -p cam-ipa-sk --features v4l2"
        );
        // Still output a template CMOS JSON
        let cmos_json = serde_json::json!({
            "sensor": {
                "name": "unknown",
                "model": "unknown",
                "pixel_format": "unknown",
                "bits_per_pixel": 10,
                "color_format": "SRGGB10",
                "active_area": { "left": 0, "top": 0, "width": 1920, "height": 1080 },
                "pixel_array": { "width": 1920, "height": 1080 },
                "controls": {
                    "exposure_time": { "min": 1, "max": 1000000, "default": 2000 },
                    "analogue_gain": { "min": 1.0, "max": 16.0, "default": 1.0 },
                    "colour_gains": { "min": [0.1, 0.1], "max": [16.0, 16.0], "default": [1.0, 1.0] }
                }
            },
            "source": "template",
            "device": dev_str
        });
        let json_str = serde_json::to_string_pretty(&cmos_json).unwrap();
        if let Some(out_path) = output {
            if let Err(e) = std::fs::write(out_path, &json_str) {
                eprintln!("Failed to write {}: {}", out_path.display(), e);
                return 1;
            }
            println!("CMOS template written to {}", out_path.display());
        } else {
            println!("{}", json_str);
        }
        0
    }
}

/// Return a comma-separated list of active features for the banner.
fn feature_list() -> String {
    let mut feats = Vec::new();
    feats.push("synthetic-4k".to_string());
    #[cfg(feature = "v4l2")]
    feats.push("v4l2".to_string());
    feats.push("profile-json".to_string());
    feats.push("cmos-dump".to_string());
    feats.join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_profile_arg_conversion() {
        for arg in [
            ProfileArg::Lite,
            ProfileArg::Med,
            ProfileArg::Fused,
            ProfileArg::Heavy,
            ProfileArg::Pro,
            ProfileArg::Test,
            ProfileArg::Unified,
            ProfileArg::Hdr,
            ProfileArg::Yuv,
        ] {
            let _profile: PipelineProfile = arg.into();
        }
    }

    #[test]
    fn test_format_arg_conversion() {
        for arg in [
            FormatArg::Rgb,
            FormatArg::Rgba,
            FormatArg::Bgra,
            FormatArg::Argb,
            FormatArg::Abgr,
            FormatArg::Yuv,
            FormatArg::Nv12,
            FormatArg::P010,
        ] {
            let _fmt: IpaFormat = arg.into();
        }
    }

    #[test]
    fn test_backend_arg_conversion() {
        for arg in [
            BackendArg::MnnVulkan,
            BackendArg::Cpu,
            BackendArg::OnnxRuntime,
        ] {
            let _backend: EngineBackend = arg.into();
        }
    }

    #[test]
    fn test_pprofile_json_roundtrip() {
        let orig = PipelineProfile::FUSED;
        let pj = PProfileJson::from(&orig);
        let json = serde_json::to_string(&pj).unwrap();
        let pj2: PProfileJson = serde_json::from_str(&json).unwrap();
        let converted = pj2.to_pipeline_profile();
        assert_eq!(orig.label, converted.label);
        assert_eq!(orig.use_unpack, converted.use_unpack);
        assert_eq!(orig.use_fcs, converted.use_fcs);
        assert_eq!(orig.use_ee, converted.use_ee);
    }
}
