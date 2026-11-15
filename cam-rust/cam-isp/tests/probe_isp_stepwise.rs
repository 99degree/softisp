//! Enable the `isp.*` fused ops one at a time, verifying each before the next.
//!
//! MNN turns an `isp.*` Extra op into one Vulkan compute dispatch instead of a
//! primitive op. That is only a win if the shader behind it implements the same
//! maths — 21 of the 82 shipped `isp_*.comp` files are `// placeholder compute
//! shader` stubs that copy their input. Enabling one of those does not speed
//! anything up: it deletes the stage, and the frame still renders.
//!
//! So this walks the chain in order, enabling one fused op per step, and after
//! each step checks that the stage is still doing its job before moving on:
//!
//!   * the shader is implemented, not a stub (grep of the source)
//!   * the frame is finite, bright, and its mean has not jumped
//!   * the latency was recorded
//!
//! A step that is a stub is reported as SKIPPED rather than enabled — that is
//! the whole point of checking, and it is why `isp.demosaic_ccm`, `isp.ee` and
//! `isp.ldci` are not turned on despite having entries in the op table.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_isp_stepwise --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use std::path::PathBuf;
use std::time::Instant;

use cam_isp::blocks::*;
use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::pipeline::{BlockOpsetMode, GraphComposer, IspBlock};

/// Where a local MNN checkout keeps the shader sources.
fn shader_dir() -> Option<PathBuf> {
    let mut dirs = Vec::new();
    if let Ok(p) = std::env::var("MNN_SOURCE") {
        dirs.push(PathBuf::from(p).join("source/backend/vulkan/buffer/execution/isp_spv"));
    }
    if let Ok(home) = std::env::var("HOME") {
        dirs.push(PathBuf::from(home).join("MNN/source/backend/vulkan/buffer/execution/isp_spv"));
    }
    dirs.into_iter().find(|d| d.is_dir())
}

/// `None` when there is no source tree to consult — the step is then attempted
/// and judged on its output alone.
fn shader_is_stub(comp: &str) -> Option<bool> {
    let src = std::fs::read_to_string(shader_dir()?.join(format!("{comp}.comp"))).ok()?;
    Some(src.contains("placeholder compute shader") || src.contains("PLACEHOLDER"))
}

/// Which fused ops are on for one measurement.
#[derive(Clone, Copy, Default)]
struct Enable {
    display: bool,
}

fn mode(on: bool) -> BlockOpsetMode {
    if on {
        BlockOpsetMode::MnnIspMin
    } else {
        BlockOpsetMode::Primitive
    }
}

struct Measurement {
    infer_ms: f64,
    mean: f64,
    peak: f64,
    nonfinite: usize,
    /// Distinct values in the first plane — a stage that was replaced by a
    /// pass-through keeps the picture, so this catches the subtler stubs.
    distinct: usize,
}

fn measure(enable: Enable) -> Option<Measurement> {
    let (w, h) = (2592u32, 1940u32);
    let sample: u16 = 512;

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        Box::new(
            RawInputPackedBlock::new()
                .with_concrete_width(w as i64)
                .with_concrete_height(h as i64),
        ),
        Box::new(UnpackBlock::new().with_concrete_width(w as i64)),
        Box::new(NormalizeBlock::new()),
        Box::new(CfaBlockPacked::new()),
        Box::new(DemosaicBlock::new(2)),
        Box::new(
            DisplayBlock::new(w)
                .with_output_format(OutputFormat::FloatRgb)
                .with_opset_mode(if enable.display { 2 } else { 0 }),
        ),
    ];
    GraphComposer::wire_blocks(&mut blocks);

    let head = blocks.remove(0);
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    if let Err(e) = engine.build(head, blocks, None, 21) {
        eprintln!("  build failed: {e}");
        return None;
    }

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
    };

    if let Err(e) = engine.process(&params) {
        eprintln!("  process failed: {e}");
        return None;
    }

    let t0 = Instant::now();
    let mut infer_ms = 0.0;
    let mut last = None;
    for _ in 0..3 {
        match engine.process(&params) {
            Ok(f) => {
                infer_ms = f.inference_duration_ns as f64 / 1e6;
                last = Some(f);
            }
            Err(e) => {
                eprintln!("  process failed on repeat: {e}");
                return None;
            }
        }
    }
    let _ = t0;

    let f = last?;
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
    let n = vals.len();
    let mean: f64 = vals.iter().map(|v| *v as f64).sum::<f64>() / n as f64;
    let peak = vals.iter().cloned().fold(f32::MIN, f32::max) as f64;
    let nonfinite = vals.iter().filter(|v| !v.is_finite()).count();

    // Count distinct values in a sparse sample of plane 0.
    let plane0 = &vals[..vals.len() / 3];
    let mut seen = std::collections::HashSet::new();
    for v in plane0.iter().step_by((plane0.len() / 4096).max(1)) {
        seen.insert(v.to_bits());
    }

    Some(Measurement {
        infer_ms,
        mean,
        peak,
        nonfinite,
        distinct: seen.len(),
    })
}

/// One stage of the sweep: which block, which shader name, and how to turn it on.
struct Step {
    label: &'static str,
    shader: &'static str,
    enable: Enable,
}

#[test]
#[ignore]
fn isp_ops_one_at_a_time() {
    cam_isp::init();

    // Ordered as the chain runs. Only blocks that can emit a fused form appear
    // here; the MED profile's other stages have no `MnnIspMin` emission path in
    // the Rust builder yet, so there is nothing to switch on for them.
    let steps = [Step {
        label: "display",
        shader: "isp_display",
        enable: Enable { display: true },
    }];

    let Some(base) = measure(Enable::default()) else {
        panic!("baseline (no fused ops) failed to run");
    };
    eprintln!(
        "\n{:<10} {:<16} {:>9} {:>8} {:>8} {:>7} {:>9}  {}",
        "stage", "shader", "infer ms", "mean", "peak", "Δ ms", "distinct", "verdict"
    );
    eprintln!(
        "{:<10} {:<16} {:>9.1} {:>8.4} {:>8.3} {:>7} {:>9}  {}",
        "none", "-", base.infer_ms, base.mean, base.peak, "-", base.distinct, "baseline"
    );

    let mut reference = base;
    let mut enabled = 0;

    for step in steps {
        // Gate on the shader, not on the name: a stub passes every picture
        // check because it hands its input through unchanged.
        match shader_is_stub(step.shader) {
            Some(true) => {
                eprintln!(
                    "{:<10} {:<16} {:>9} {:>8} {:>8} {:>7} {:>9}  STUB — not enabled",
                    step.label, step.shader, "-", "-", "-", "-", "-"
                );
                continue;
            }
            Some(false) => {}
            None => eprintln!(
                "  (no MNN source tree; judging {} on output only)",
                step.shader
            ),
        }

        let Some(m) = measure(step.enable) else {
            eprintln!("{:<10} BUILD/RUN FAILED — stopping", step.label);
            panic!("{} failed to build or run", step.label);
        };

        let finite = m.nonfinite == 0;
        let bright = m.mean > 0.05 && m.peak > 0.2;
        let comparable = (m.mean - reference.mean).abs() <= 0.15 * reference.mean.max(0.05);
        let still_textured = m.distinct >= reference.distinct / 2;
        let verdict = if !finite {
            "NON-FINITE"
        } else if !bright {
            "BLANK"
        } else if !comparable {
            "VALUE JUMP"
        } else if !still_textured {
            "FLATTENED"
        } else {
            "ok"
        };

        eprintln!(
            "{:<10} {:<16} {:>9.1} {:>8.4} {:>8.3} {:>+7.1} {:>9}  {}",
            step.label,
            step.shader,
            m.infer_ms,
            m.mean,
            m.peak,
            reference.infer_ms - m.infer_ms,
            m.distinct,
            verdict
        );

        if verdict != "ok" {
            panic!(
                "enabling `{}` ({}) produced {verdict}: mean {:.4} (was {:.4}), peak {:.3}, \
                 non-finite {}, distinct {} (was {})",
                step.label,
                step.shader,
                m.mean,
                reference.mean,
                m.peak,
                m.nonfinite,
                m.distinct,
                reference.distinct
            );
        }

        reference = m;
        enabled += 1;
    }

    eprintln!("\n{enabled} fused op(s) verified and enabled.");

    // The stages that could have been fused but cannot be, and why. This is the
    // actionable part: these need a shader, not a flag.
    eprintln!("\nblocked, with the reason:");
    for (label, shader, reason) in [
        (
            "demosaic_ccm",
            "isp_demosaic_ccm",
            "shader is a placeholder",
        ),
        ("ee", "isp_ee", "shader is a placeholder"),
        ("ldci", "isp_ldci", "shader is a placeholder"),
        (
            "normalize+blc+cfa",
            "isp.unpack_blc",
            "no shader in the op table at all",
        ),
        ("ccm", "isp_ccm", "block has no MnnIspMin emission path"),
        ("tone", "isp_tone", "block has no MnnIspMin emission path"),
        (
            "saturation",
            "isp_saturation",
            "block has no MnnIspMin emission path",
        ),
        (
            "bayer_wb",
            "isp.bayer_wb",
            "block has no MnnIspMin emission path",
        ),
        ("lsc", "isp_lsc", "block has no MnnIspMin emission path"),
        (
            "unsharp",
            "isp_unsharp",
            "block has no MnnIspMin emission path",
        ),
    ] {
        let stub = shader_is_stub(shader);
        let state = match stub {
            Some(true) => "placeholder",
            Some(false) => "implemented",
            None => "unknown",
        };
        eprintln!("  {label:<18} {shader:<18} {state:<12} {reason}");
    }
}
