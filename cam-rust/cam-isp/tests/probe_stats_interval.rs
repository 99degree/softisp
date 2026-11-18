//! What does skipping the statistics readback on most frames buy?
//!
//! The readback is the expensive half of the statistics: the reduction runs on the
//! GPU either way, but `copyToHostTensor` forces a wait for the queue. Running it
//! every frame for numbers the controller smooths is wasted time.
//!
//! This measures the chain at several intervals and checks the frames in between
//! report `stats_measured == false` and carry nothing — the flag is what a consumer
//! uses to tell "nothing new" from "the readback broke".
//!
//! Run:
//!   cargo test -p cam-isp --test probe_stats_interval --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

const W: u32 = 1296;
const H: u32 = 970;

fn render(interval: u32, stats_max: u32) -> Option<(f64, usize, usize, f64)> {
    let mut p = PipelineProfile::FUSED;
    p.output_format = OutputFormat::FloatRgb;
    p.opset_mode = 2;
    p.stats_downscale_max = stats_max;

    let mut blocks = p.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(H as i64, W as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.set_stats_interval(interval);
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
    };

    // Warm up, then sample. `measured` and `skipped` count what the frames said.
    for _ in 0..8 {
        engine.process(&params).ok()?;
    }
    let mut t = Vec::new();
    let mut measured = 0;
    let mut skipped = 0;
    let mut last_mean = 0.0;
    for _ in 0..16 {
        let f = engine.process(&params).ok()?;
        t.push(f.inference_duration_ns as f64 / 1e6);
        match f.aux.as_ref() {
            Some(a) if a.stats_measured => measured += 1,
            Some(_) => skipped += 1,
            None => {}
        }
        let vals: Vec<f32> = f
            .data
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b))
            .collect();
        last_mean = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len().max(1) as f64;
    }
    t.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    Some((t[t.len() / 2], measured, skipped, last_mean))
}

#[test]
#[ignore]
fn stats_interval_cost() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    eprintln!(
        "\n{:<28} {:>9} {:>9} {:>8} {:>9}",
        "configuration", "ms", "measured", "skipped", "mean"
    );
    for (stats_max, label) in [(0u32, "stats full"), (540, "stats 540")] {
        for interval in [1u32, 2, 4, 8] {
            let tag = format!("{label}, every {interval}");
            match render(interval, stats_max) {
                Some((ms, m, s, mean)) => {
                    eprintln!("{tag:<28} {ms:>9.1} {m:>9} {s:>8} {mean:>9.4}")
                }
                None => eprintln!("{tag:<28} failed"),
            }
        }
    }
    eprintln!(
        "\nA skipped frame must carry no statistics (`measured` counts the ones that\n\
         did) and the picture must not move between rows."
    );
}
