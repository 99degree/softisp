//! Where does 100 ms of per-frame time go that is not the GPU?
//!
//! `probe_4k_drift` reported a 74% spread between two runs of the *same* 4K
//! configuration. Part of that is throttling, but the spread is large enough that
//! the measurement itself is suspect — so this times the halves the engine reports
//! separately: `prep` (CPU staging, shape setup) and `infer` (the MNN call).
//!
//! If prep is a large fraction, the 4K numbers are mostly the host staging the
//! sensor buffer, and no amount of fence-removal in the GPU path helps.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_4k_prep --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

fn env_usize(key: &str, default: usize) -> usize {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

#[test]
#[ignore]
fn prep_vs_infer_4k() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 20);

    let mut p = PipelineProfile::FUSED;
    p.output_format = OutputFormat::FloatRgb;
    let mut blocks = p.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.build(head, blocks, None, 21).expect("build");

    let raw: Vec<u8> = {
        let mut v = vec![0u8; (w as usize) * (h as usize) * 2];
        for (i, row) in v.chunks_exact_mut(w as usize * 2).enumerate() {
            for (x, px) in row.chunks_exact_mut(2).enumerate() {
                let s: u16 = ((x * 1023) / (w as usize).max(1)) as u16;
                let s = s.saturating_add((i % 256) as u16 * 2).min(1023);
                px[0] = (s & 0xFF) as u8;
                px[1] = (s >> 8) as u8;
            }
        }
        v
    };
    let params = ProcessParams {
        bayer_pattern: 2,
        output_format: OutputFormat::FloatRgb,
        ..ProcessParams::new(w, h, &raw)
    };
    for _ in 0..6 {
        engine.process(&params).expect("warmup");
    }

    let mut prep = Vec::new();
    let mut infer = Vec::new();
    let mut total = Vec::new();
    for _ in 0..frames {
        let f = engine.process(&params).expect("frame");
        prep.push(f.prep_duration_ns as f64 / 1e6);
        infer.push(f.inference_duration_ns as f64 / 1e6);
        total.push(f.total_duration_ns as f64 / 1e6);
    }
    let med = |v: &mut Vec<f64>| {
        v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        v[v.len() / 2]
    };

    let (p50_prep, p50_infer, p50_total) = (med(&mut prep), med(&mut infer), med(&mut total));
    eprintln!("\n== {w}x{h}, {frames} frames ==");
    eprintln!(
        "  prep  (CPU staging)  {p50_prep:>8.2} ms  ({:.0}%)",
        100.0 * p50_prep / p50_total
    );
    eprintln!(
        "  infer (MNN call)     {p50_infer:>8.2} ms  ({:.0}%)",
        100.0 * p50_infer / p50_total
    );
    eprintln!("  total                {p50_total:>8.2} ms");
    eprintln!(
        "\nprep is the even/odd split of a {:.0} MB sensor buffer plus the tensor\n\
         shape setup; if it dominates, the frame cost is host work.",
        (w as f64) * (h as f64) * 2.0 / 1e6
    );
}
