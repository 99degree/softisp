//! What do the remaining primitive stages in FUSED actually cost?
//!
//! The chain is now
//!     isp.unpack_blc_normal_cfa → isp.demosaic_ccm → tone → isp.ee
//!       → saturation → isp.display
//! and the open question is whether tone + saturation are worth folding into one
//! dispatch. Between them they are `Mul, Mul, Add, Clip` and `Mul` — five
//! elementwise ops over a 3-plane frame. Two dispatches of an elementwise kernel
//! may cost less than one larger kernel, or more, and the answer decides whether
//! the extension is worth its risk.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_tail_stage_cost --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::{FuseStages, PipelineProfile};

const W: u32 = 1296;
const H: u32 = 970;

fn render(mut p: PipelineProfile) -> Option<f64> {
    p.output_format = OutputFormat::FloatRgb;
    let mut blocks = p.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(H as i64, W as i64);
    let n = blocks.len() + 1;
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.build(head, blocks, None, 21).ok()?;

    let raw = cam_isp::synth_bayer::bayer_color_diffuse(W, H);
    let params = ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf: &raw,
        sensor_max: 1023.0,
        target_width: W,
        target_height: H,
        ccm_matrix: None,
        tone_params: cam_isp::engine::default_tone_params(),
        bayer_gains: None,
        awb_gains: None,
        bayer_pattern: 2,
        analog_gain: 1.0,
        scene_change: 0.0,
        lsc_gains: None,
        blc_values: None,
        warp_grid: None,
        warp_shading: None,
        output_format: OutputFormat::FloatRgb,
        timestamp_ns: 0,
        isp_params: None,
        input_buffer: None,
        output_buffer: None,
        input_ahb: None,
        output_ahb: None,
    };
    for _ in 0..6 {
        engine.process(&params).ok()?;
    }
    let mut t = Vec::new();
    for _ in 0..9 {
        t.push(engine.process(&params).ok()?.inference_duration_ns as f64 / 1e6);
    }
    t.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    eprintln!("  {} blocks: {:.1} ms", n, t[t.len() / 2]);
    Some(t[t.len() / 2])
}

#[test]
#[ignore]
fn tail_stage_cost() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let mut base = PipelineProfile::FUSED;
    base.fuse_stages = FuseStages::ALL;

    eprintln!("\nfull FUSED:");
    let full = render(base).expect("baseline failed");

    let cases: [(&str, fn(&mut PipelineProfile)); 4] = [
        ("no tone", |p| {
            p.use_saturation = true;
        }),
        ("no saturation", |p| p.use_saturation = false),
        ("no ee", |p| p.use_ee = false),
        ("no stats", |p| {
            p.use_zone_stats = false;
            p.use_channel_means = false;
            p.use_tone_stats = false;
        }),
    ];

    eprintln!("\nwithout each stage:");
    for (label, f) in cases {
        let mut p = base;
        f(&mut p);
        eprintln!("  {label}:");
        match render(p) {
            Some(t) => eprintln!("    saves {:.1} ms", full - t),
            None => eprintln!("    failed"),
        }
    }
}
