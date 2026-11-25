//! Does the fused (`isp.*`) chain render correctly at fp32, and at what pool size?
//!
//! The per-stage sweep showed fusion and fp16 storage are mutually exclusive:
//! `VulkanBackend::getTensorSize` allocates two bytes per float element when
//! `mUseFP16`, while every `isp.*` shader declares `float[]` buffers. A fused
//! CCM reads halves as floats and the stage outputs zeros.
//!
//! `probe_fp32_budget` showed the primitive chain fits at 2592x1940 in fp32 with
//! any pool size, so memory is not what stops fp32 — the earlier OOM came from
//! two engines alive at once (one per opset mode in a single process).
//!
//! This isolates the fused chain: one engine, fp32, walking pool sizes down from
//! 4, and reports whether the frame survives.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_fused_fp32 --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

struct Stats {
    mean: f64,
    peak: f64,
    distinct: usize,
}

fn stats(data: &[u8]) -> Stats {
    let vals: Vec<f32> = data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    if vals.is_empty() {
        return Stats {
            mean: 0.0,
            peak: 0.0,
            distinct: 0,
        };
    }
    let mean = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len() as f64;
    let peak = vals.iter().cloned().fold(f32::MIN, f32::max) as f64;
    let plane0 = &vals[..vals.len() / 3];
    let step = (plane0.len() / 4096).max(1);
    let mut seen = std::collections::HashSet::new();
    for v in plane0.iter().step_by(step) {
        seen.insert(v.to_bits());
    }
    Stats {
        mean,
        peak,
        distinct: seen.len(),
    }
}

fn run(pool: usize, opset_mode: i32, fp16: bool, w: u32, h: u32) -> Result<Stats, String> {
    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;
    profile.opset_mode = opset_mode;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, pool);
    engine.set_fp16_storage(fp16);
    engine
        .build(head, blocks, None, 21)
        .map_err(|e| format!("build: {e}"))?;

    // Mid-dark input on purpose. At 600/1023 the pipeline clips everything to
    // 1.0 and every configuration reports the same saturated mean, which hides
    // exactly the differences this probe is for.
    let sample: u16 = 200;
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
    let f = engine
        .process(&params)
        .map_err(|e| format!("process: {e}"))?;
    Ok(stats(&f.data))
}

#[test]
#[ignore]
fn fused_chain_needs_fp32() {
    cam_isp::init();

    // 1296x970 first: half the sensor in each axis, i.e. the RGB geometry the
    // chain actually produces. If fusion works here but not at full size, the
    // difference is memory, not the shaders.
    for (w, h) in [(1296u32, 970u32), (2592, 1940)] {
        eprintln!("\n== {w}x{h} ==");
        for (opset, label) in [(0, "primitive"), (2, "fused")] {
            for fp16 in [true, false] {
                let prec = if fp16 { "fp16" } else { "fp32" };
                match run(2, opset, fp16, w, h) {
                    Ok(s) => eprintln!(
                        "  {label:<10} {prec}  pool=2  mean {:.4} peak {:.3} distinct {}",
                        s.mean, s.peak, s.distinct
                    ),
                    Err(e) => eprintln!("  {label:<10} {prec}  pool=2  FAILED: {e}"),
                }
            }
        }
    }
}
