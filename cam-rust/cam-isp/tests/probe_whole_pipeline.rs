//! Does the whole-pipeline op render, and does it match the per-stage chain?
//!
//! `isp.pipeline` is a different formulation, not an optimisation: it dispatches
//! over the *output* grid and area-averages each output pixel's footprint in the
//! raw plane, so resample + CFA + demosaic collapse into one step. Its box filter
//! will not agree with an edge-directed demosaic to the last bit — the question
//! this answers is whether it agrees closely enough to be a preview, and what it
//! costs.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_whole_pipeline --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

const W: u32 = 2592;
const H: u32 = 1940;

struct Stats {
    ms: f64,
    mean: f64,
    peak: f64,
    values: Vec<f32>,
}

fn run(profile: PipelineProfile) -> Option<Stats> {
    let mut profile = profile;
    profile.output_format = OutputFormat::FloatRgb;

    let mut blocks = profile.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let n = blocks.len() + 1;

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.build(head, blocks, None, 21).ok()?;

    let sample: u16 = 200;
    let mut buf = vec![0u8; (W as usize) * (H as usize) * 2];
    for c in buf.chunks_exact_mut(2) {
        c[0] = (sample & 0xFF) as u8;
        c[1] = (sample >> 8) as u8;
    }
    let params = ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf: &buf,
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

    for _ in 0..5 {
        engine.process(&params).ok()?;
    }
    let mut times = Vec::new();
    let mut values = Vec::new();
    for _ in 0..9 {
        let f = engine.process(&params).ok()?;
        times.push(f.inference_duration_ns as f64 / 1e6);
        values = f
            .data
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b))
            .collect();
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mean = values.iter().map(|v| *v as f64).sum::<f64>() / values.len().max(1) as f64;
    let peak = values.iter().cloned().fold(f32::MIN, f32::max) as f64;
    eprintln!("  ({n} blocks)");
    Some(Stats {
        ms: times[times.len() / 2],
        mean,
        peak,
        values,
    })
}

#[test]
#[ignore]
fn whole_pipeline_vs_per_stage() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let mut med = PipelineProfile::MED;
    med.output_format = OutputFormat::FloatRgb;
    let mut fused = PipelineProfile::FUSED;
    fused.output_format = OutputFormat::FloatRgb;
    // The real profile, not a MED with one flag flipped: that is what ships.
    let whole = PipelineProfile::WHOLE;

    eprintln!("\n{:<14} {:>9} {:>8} {:>7}", "chain", "ms", "mean", "peak");
    let a = run(med);
    let b = run(fused);
    let c = run(whole);
    for (label, r) in [("MED", &a), ("FUSED", &b), ("WHOLE", &c)] {
        match r {
            Some(s) => eprintln!(
                "{label:<14} {:>9.1} {:>8.4} {:>7.3}  {}",
                s.ms,
                s.mean,
                s.peak,
                if s.mean > 0.05 && s.peak > 0.2 {
                    "picture"
                } else {
                    "BLANK"
                }
            ),
            None => eprintln!("{label:<14} FAILED"),
        }
    }

    let (Some(med_s), Some(whole_s)) = (a, c) else {
        panic!("a chain failed to render");
    };
    assert!(
        whole_s.mean > 0.05 && whole_s.peak > 0.2,
        "isp.pipeline produced a blank frame (mean {:.4} peak {:.3})",
        whole_s.mean,
        whole_s.peak
    );

    // The two formulations cannot agree exactly: one demosaics with an edge
    // filter at full resolution, the other area-averages the footprint. Compare
    // the only thing that has to hold — the overall level.
    if med_s.values.len() == whole_s.values.len() {
        let mut sum = 0.0f64;
        for (x, y) in med_s.values.iter().zip(whole_s.values.iter()) {
            sum += (*x as f64 - *y as f64).abs();
        }
        let dm = sum / med_s.values.len() as f64;
        eprintln!("\nper-pixel mean|Δ| vs MED: {dm:.4}");
        eprintln!(
            "relative: {:.1}% of the frame's mean level",
            100.0 * dm / med_s.mean.max(1e-6)
        );
    } else {
        eprintln!(
            "\nvalue counts differ (MED {} vs WHOLE {}) — the whole-pipeline chain \
             emits its own geometry",
            med_s.values.len(),
            whole_s.values.len()
        );
    }
}
