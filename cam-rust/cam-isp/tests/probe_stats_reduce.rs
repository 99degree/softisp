//! Does the statistics reduction get cheaper, and are its answers the same?
//!
//! Channel means and tone stats now box-average their input before reducing. A mean
//! is exact under that filter, and min/max/masks of a smooth frame barely move — so
//! the question is whether the numbers survive and what the traversal saves.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_stats_reduce --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

const W: u32 = 1296;
const H: u32 = 970;

struct Out {
    ms: f64,
    mean: f64,
    measured: bool,
    calib: Option<[f32; 24]>,
}

fn run(stats_max: u32) -> Option<Out> {
    let mut p = PipelineProfile::FUSED;
    p.output_format = OutputFormat::FloatRgb;
    p.opset_mode = 2;
    p.stats_downscale_max = stats_max;

    let mut blocks = p.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(H as i64, W as i64);
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

    for _ in 0..8 {
        engine.process(&params).ok()?;
    }
    let mut t = Vec::new();
    let mut last = None;
    for _ in 0..11 {
        let f = engine.process(&params).ok()?;
        t.push(f.inference_duration_ns as f64 / 1e6);
        last = Some(f);
    }
    let f = last?;
    t.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let vals: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    let mean = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len().max(1) as f64;
    let aux = f.aux.as_ref();
    Some(Out {
        ms: t[t.len() / 2],
        mean,
        measured: aux.map(|a| a.stats_measured).unwrap_or(false),
        calib: aux.and_then(|a| a.calibration_stats),
    })
}

#[test]
#[ignore]
fn stats_reduce_cost() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    eprintln!(
        "\n{:<14} {:>9} {:>8}  {:>9}  {}",
        "stats", "ms", "frame", "measured", "calibration[0..3]"
    );
    for stats_max in [0u32, 648, 324] {
        let tag = if stats_max == 0 {
            "full".to_string()
        } else {
            format!("{stats_max} edge")
        };
        match run(stats_max) {
            Some(o) => eprintln!(
                "{tag:<14} {:>9.1} {:>8.4}  {:>9}  {}",
                o.ms,
                o.mean,
                o.measured,
                match o.calib {
                    Some(c) => format!("[{:.4},{:.4},{:.4}]", c[0], c[1], c[2]),
                    None => "none".to_string(),
                }
            ),
            None => eprintln!("{tag:<14} FAILED"),
        }
    }
    eprintln!(
        "\n`frame` is the displayed picture and must not move: only the statistics\n\
         source changes. The means are what the controller consumes."
    );
}
