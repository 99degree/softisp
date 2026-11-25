//! What would a dual-precision (fp16/fp32) shader set actually be worth?
//!
//! The fused `isp.*` shaders declare `float` buffers, so `MnnEngine` forces fp32
//! tensor storage whenever a chain contains one (`chain_has_fused_op`). That is a
//! correctness requirement today, not a tuning choice — and it means every
//! frame-sized tensor in the chain is 4 bytes per lane.
//!
//! Before writing a second fp16 variant of every shader, this measures the
//! ceiling: the same chain and op count at fp16 against fp32. Whatever fp16
//! storage saves is the most a dual-precision shader set could ever recover,
//! because the arithmetic would stay fp32 either way.
//!
//! It also measures the thing that decides the design: the per-dispatch cost at
//! a tiny frame. If a dispatch costs milliseconds regardless of pixel count, the
//! win is in *merging stages*, not in halving the bytes between them.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_fp16_storage --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

struct Stats {
    ms: f64,
    mean: f64,
}

fn run(profile: PipelineProfile, fp16: bool, w: u32, h: u32) -> Option<Stats> {
    let mut profile = profile;
    profile.output_format = OutputFormat::FloatRgb;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let n = blocks.len() + 1;

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    // Set before `build`: the session's tensor precision is chosen once, at
    // creation, and a chain containing a fused op overrides the request.
    engine.set_fp16_storage(fp16);
    engine.build(head, blocks, None, 21).ok()?;

    let sample: u16 = 200; // mid-dark: at 600/1023 everything clips to 1.0
    let mut buf = vec![0u8; (w as usize) * (h as usize) * 2];
    for c in buf.chunks_exact_mut(2) {
        c[0] = (sample & 0xFF) as u8;
        c[1] = (sample >> 8) as u8;
    }
    let params = ProcessParams {
        width: w,
        height: h,
        stride_width: w,
        buf: &buf,
        sensor_max: 1023.0,
        target_width: w,
        target_height: h,
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
    let mut times: Vec<f64> = Vec::new();
    let mut mean = 0.0;
    for _ in 0..9 {
        let f = engine.process(&params).ok()?;
        times.push(f.inference_duration_ns as f64 / 1e6);
        if mean == 0.0 {
            let vals: Vec<f32> = f
                .data
                .as_chunks::<4>()
                .0
                .iter()
                .map(|b| f32::from_le_bytes(*b))
                .collect();
            if vals.is_empty() {
                return None;
            }
            mean = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len() as f64;
        }
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let _ = n;
    Some(Stats {
        ms: times[times.len() / 2],
        mean,
    })
}

#[test]
#[ignore]
fn fp16_storage_ceiling() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    eprintln!(
        "\n{:<24} {:>9} {:>8}  {}",
        "chain / storage", "ms", "mean", "state"
    );
    for (label, profile) in [
        ("MED", PipelineProfile::MED),
        ("FUSED", PipelineProfile::FUSED),
    ] {
        for fp16 in [false, true] {
            let tag = format!("{label} {}", if fp16 { "fp16" } else { "fp32" });
            match run(profile, fp16, 1296, 970) {
                Some(s) => eprintln!(
                    "{tag:<24} {:>9.1} {:>8.4}  {}",
                    s.ms,
                    s.mean,
                    if s.mean > 0.05 { "picture" } else { "BLANK" }
                ),
                None => eprintln!("{tag:<24}        --        --  FAILED"),
            }
        }
    }

    // Per-dispatch cost: the same profile at 1/16 the pixels.
    eprintln!("\nper-dispatch cost (same chain, 1/16 the pixels):");
    for (label, profile) in [
        ("MED", PipelineProfile::MED),
        ("FUSED", PipelineProfile::FUSED),
    ] {
        match (
            run(profile, false, 1296, 970),
            run(profile, false, 324, 242),
        ) {
            (Some(big), Some(small)) => {
                let saved = big.ms - small.ms;
                eprintln!(
                    "  {label:<8} {:.1} ms -> {:.1} ms   {:.1} ms of that is per-pixel work",
                    big.ms, small.ms, saved
                );
            }
            _ => eprintln!("  {label:<8} failed"),
        }
    }
    eprintln!(
        "\nfp16 only helps the part that scales with pixels. A fused chain forces\n\
         fp32 before the session exists, so the FUSED/fp16 row is the fp32 number\n\
         again — that is what a dual-precision shader set would have to change."
    );
}
