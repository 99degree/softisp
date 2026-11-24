//! Dual-GPU engine probe: two `isp.*` Vulkan engine instances alternate frames
//! (engine A frame N, engine B frame N+1) to overlap GPU inference with host-side
//! frame prep. Both engines run the same fused `isp.*` chain (fp32 tensor storage)
//! and share one `IspController` so AWB/AE/CCM state stays consistent.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_4k_dual --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::dual_engine::DualGpuEngine;
use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

fn env_usize(key: &str, default: usize) -> usize {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

struct Timing {
    label: String,
    blocks: usize,
    p50: f64,
    p95: f64,
    max: f64,
    fps: f64,
    mean: f64,
}

fn run_dual(
    label: &str,
    w: u32,
    h: u32,
    frames: usize,
    make: impl Fn(
        u32,
        u32,
    ) -> (
        Box<dyn cam_isp::pipeline::IspBlock>,
        Vec<Box<dyn cam_isp::pipeline::IspBlock>>,
    ),
) -> Option<Timing> {
    let (head_a, blocks_a) = make(w, h);
    let n_blocks = blocks_a.len() + 1;

    let mut engine_a = MnnEngine::with_pool_size(MnnBackend::Vulkan, 1);
    engine_a.set_stats_enabled(true); // engine A reads stats every frame
    if let Err(e) = engine_a.build(head_a, blocks_a, None, 21) {
        eprintln!("{label:<28} engine A build failed: {e}");
        return None;
    }

    let mut engine_b = MnnEngine::with_pool_size(MnnBackend::Vulkan, 1);
    engine_b.set_stats_enabled(false); // engine B skips stats entirely
    let (head_b, blocks_b) = make(w, h);
    if let Err(e) = engine_b.build(head_b, blocks_b, None, 21) {
        eprintln!("{label:<28} engine B build failed: {e}");
        return None;
    }

    let mut dual = DualGpuEngine::new(Box::new(engine_a), Box::new(engine_b));

    // 4K raw: an 8-bit-ish ramp so the pipeline has real gradients to work on and
    // the sensor-max divide lands well inside [0,1].
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
        width: w,
        height: h,
        stride_width: w,
        buf: &raw,
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
        input_buffer: None,
        output_buffer: None,
        input_ahb: None,
        output_ahb: None,
        output_format: OutputFormat::FloatRgb,
        timestamp_ns: 0,
        isp_params: None,
    };

    // Warm up: the first frames include session allocation and shape propagation.
    for _ in 0..8 {
        match dual.next().process(&params) {
            Ok(_) => {}
            Err(e) => {
                eprintln!("{label:<28} warmup failed: {e}");
                return None;
            }
        }
    }

    let mut t: Vec<f64> = Vec::with_capacity(frames);
    let mut sum = 0.0f64;
    let mut picture = false;
    for _ in 0..frames {
        match dual.next().process(&params) {
            Ok(f) => {
                let ms = f.inference_duration_ns as f64 / 1e6;
                t.push(ms);
                sum += ms;
                if !picture {
                    let vals: Vec<f32> = f
                        .data
                        .as_chunks::<4>()
                        .0
                        .iter()
                        .map(|b| f32::from_le_bytes(*b))
                        .collect();
                    let m = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len().max(1) as f64;
                    picture = m > 0.02;
                }
            }
            Err(e) => {
                eprintln!("{label:<28} process failed: {e}");
                return None;
            }
        }
    }
    t.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let p50 = t[t.len() / 2];
    let p95 = t[(t.len() * 95) / 100];
    let max = *t.last().unwrap();
    let mean = sum / t.len() as f64;
    if !picture {
        eprintln!("{label:<28} produced a blank frame");
        return None;
    }
    Some(Timing {
        label: label.to_string(),
        blocks: n_blocks,
        p50,
        p95,
        max,
        fps: 1000.0 / p50,
        mean,
    })
}

#[test]
#[ignore]
fn perf_4k_dual() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 40);

    let fused = || PipelineProfile::FUSED;

    let build = |p: PipelineProfile| {
        move |w: u32, h: u32| {
            let mut p = p;
            p.output_format = OutputFormat::FloatRgb;
            let mut blocks = p.build_blocks(w, h, 2);
            let head = blocks.remove(0);
            let aux = p.build_aux_blocks(h as i64, w as i64);
            blocks.extend(aux);
            (head, blocks)
        }
    };

    eprintln!("\n== {w}x{h}, {frames} frames after warmup (DUAL GPU engine) ==");
    eprintln!(
        "{:<30} {:>7} {:>9} {:>9} {:>9} {:>8}",
        "profile", "blocks", "p50 ms", "p95 ms", "max ms", "FPS"
    );

    let mut rows: Vec<Timing> = Vec::new();
    for (label, p) in [("DUAL fused", fused())] {
        if let Some(t) = run_dual(label, w, h, frames, build(p)) {
            eprintln!(
                "{:<30} {:>7} {:>9.2} {:>9.2} {:>9.2} {:>8.1}",
                t.label, t.blocks, t.p50, t.p95, t.max, t.fps
            );
            rows.push(t);
        }
    }

    if let Some(fused_r) = rows.first() {
        eprintln!(
            "\nDUAL engine: engine A stats ON + engine B stats OFF, mean {:.1} ms",
            fused_r.mean
        );
    }
}
