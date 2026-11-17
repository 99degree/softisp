//! Direct `isp.*` emission, one stage at a time, verified by difference.
//!
//! Each fusion-capable block can emit either primitive ONNX ops or a single
//! fused `isp.*` Extra op. "It renders something" is not the question — a stub
//! shader that copies its input also renders something. The question is whether
//! the fused form computes the same pixels as the primitive form it replaces.
//!
//! Two things have to be true for a step to be judged fairly, and both were
//! wrong in the first version of this probe:
//!
//!   * the comparison is against the *previous* step, so exactly one change is
//!     under test;
//!   * the chain shape is unchanged. `DemosaicCcmBlock` means "demosaic and CCM
//!     together" — its shader applies `CCM x demosaic` weights — so fusing it
//!     has to *replace* both blocks. Leaving a separate `CcmBlock` in the chain
//!     applies the colour matrix twice and makes the step look broken.
//!
//! So the sweep keeps the chain shape fixed (Demosaic + Ccm + Ee + Ldci +
//! Display) and flips one block at a time, then a separate comparison checks the
//! demosaic+CCM fusion against the pair it replaces.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_fusion_stepwise --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use std::time::Instant;

use cam_isp::blocks::*;
use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::pipeline::{BlockOpsetMode, GraphComposer, IspBlock};

fn mode(on: bool) -> BlockOpsetMode {
    if on {
        BlockOpsetMode::MnnIspMin
    } else {
        BlockOpsetMode::Primitive
    }
}

/// Which fused ops are on for one measurement.
#[derive(Clone, Copy, Default)]
struct Fuse {
    ccm: bool,
    ee: bool,
    ldci: bool,
    display: bool,
}

struct Frame {
    data: Vec<u8>,
    infer_ms: f64,
}

/// Sensor geometry the probe renders at. Small enough to iterate on, large
/// enough that a per-pixel bug cannot hide behind padding.
const W: u32 = 640;
const H: u32 = 480;

fn base_blocks() -> Vec<Box<dyn IspBlock>> {
    vec![
        Box::new(
            RawInputPackedBlock::new()
                .with_concrete_width(W as i64)
                .with_concrete_height(H as i64),
        ),
        Box::new(UnpackBlock::new().with_concrete_width(W as i64)),
        Box::new(NormalizeBlock::new()),
        Box::new(CfaBlockPacked::new()),
    ]
}

fn render_with(mut blocks: Vec<Box<dyn IspBlock>>) -> Option<Frame> {
    let sample: u16 = 600;
    GraphComposer::wire_blocks(&mut blocks);

    let head = blocks.remove(0);
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    if let Err(e) = engine.build(head, blocks, None, 21) {
        eprintln!("  build failed: {e}");
        return None;
    }

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

    engine.process(&params).ok()?;

    let t0 = Instant::now();
    let f = engine.process(&params).ok()?;
    let wall = t0.elapsed().as_secs_f64() * 1000.0;
    Some(Frame {
        data: f.data,
        infer_ms: if f.inference_duration_ns > 0 {
            f.inference_duration_ns as f64 / 1e6
        } else {
            wall
        },
    })
}

/// The sweep chain: Demosaic + Ccm + Ee + Ldci + Display, one flag per block.
fn render(fuse: Fuse) -> Option<Frame> {
    let mut blocks = base_blocks();
    blocks.push(Box::new(DemosaicBlock::new(2)));
    blocks.push(Box::new(
        CcmBlock::with_instance("ccm").with_opset_mode(mode(fuse.ccm)),
    ));
    blocks.push(Box::new(EeBlock::new().with_opset_mode(mode(fuse.ee))));
    blocks.push(Box::new(LdciBlock::new().with_opset_mode(mode(fuse.ldci))));
    blocks.push(Box::new(
        DisplayBlock::new(W)
            .with_output_format(OutputFormat::FloatRgb)
            .with_opset_mode(if fuse.display { 2 } else { 0 }),
    ));
    render_with(blocks)
}

/// Demosaic and CCM as one fused op, replacing both blocks.
fn render_demosaic_ccm(fused: bool) -> Option<Frame> {
    let mut blocks = base_blocks();
    blocks.push(Box::new(
        DemosaicCcmBlock::new(2)
            .with_concrete_dims(H as i64, W as i64)
            .with_opset_mode(mode(fused)),
    ));
    blocks.push(Box::new(
        DisplayBlock::new(W).with_output_format(OutputFormat::FloatRgb),
    ));
    render_with(blocks)
}

fn floats(data: &[u8]) -> Vec<f32> {
    data.as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect()
}

/// (mean absolute difference, peak absolute difference).
fn diff(a: &[u8], b: &[u8]) -> Option<(f64, f64)> {
    let (va, vb) = (floats(a), floats(b));
    if va.len() != vb.len() || va.is_empty() {
        return None;
    }
    let mut sum = 0.0f64;
    let mut peak = 0.0f64;
    for (x, y) in va.iter().zip(vb.iter()) {
        let d = (*x as f64 - *y as f64).abs();
        sum += d;
        if d > peak {
            peak = d;
        }
    }
    Some((sum / va.len() as f64, peak))
}

fn stats(data: &[u8]) -> (f64, f64, usize) {
    let v = floats(data);
    if v.is_empty() {
        return (0.0, 0.0, 0);
    }
    let mean = v.iter().map(|x| *x as f64).sum::<f64>() / v.len() as f64;
    let peak = v.iter().cloned().fold(f32::MIN, f32::max) as f64;
    let nonfinite = v.iter().filter(|x| !x.is_finite()).count();
    (mean, peak, nonfinite)
}

/// Classify one step against its reference.
fn verdict(nonf: usize, mean: f64, peak: f64, dmean: f64, dpeak: f64) -> &'static str {
    if nonf > 0 {
        "NON-FINITE"
    } else if peak <= 0.2 || mean <= 0.05 {
        "BLANK"
    } else if dmean > 0.02 || dpeak > 0.25 {
        "DIFFERS"
    } else {
        "ok"
    }
}

#[test]
#[ignore]
fn fusion_one_stage_at_a_time() {
    // Without a logger the engine's warnings about coefficient uploads go
    // nowhere, and a fused stage that silently keeps its zeroed buffer looks
    // identical to one whose shader is wrong.
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    eprintln!(
        "\n{:<18} {:>9} {:>8} {:>8} {:>9} {:>9}  {}",
        "fused stages", "infer ms", "mean", "peak", "mean|Δ|", "peak|Δ|", "verdict"
    );

    let Some(base) = render(Fuse::default()) else {
        panic!("baseline (all primitive) failed to run");
    };
    let (bm, bp, bn) = stats(&base.data);
    eprintln!(
        "{:<18} {:>9.1} {:>8.4} {:>8.3} {:>9} {:>9}  {}",
        "none (baseline)", base.infer_ms, bm, bp, "-", "-", "reference"
    );
    assert!(bn == 0 && bp > 0.2, "baseline is not a picture");

    let steps: [(&str, Fuse); 4] = [
        (
            "+ccm",
            Fuse {
                ccm: true,
                ..Default::default()
            },
        ),
        (
            "+ee",
            Fuse {
                ccm: true,
                ee: true,
                ..Default::default()
            },
        ),
        (
            "+ldci",
            Fuse {
                ccm: true,
                ee: true,
                ldci: true,
                ..Default::default()
            },
        ),
        (
            "+display",
            Fuse {
                ccm: true,
                ee: true,
                ldci: true,
                display: true,
            },
        ),
    ];

    let mut prev = base;
    for (label, fuse) in steps {
        let Some(cur) = render(fuse) else {
            eprintln!("{label:<18} BUILD/RUN FAILED — stopping here");
            break;
        };
        let (m, p, nonf) = stats(&cur.data);
        let Some((dm, dp)) = diff(&prev.data, &cur.data) else {
            eprintln!("{label:<18} frame size changed — stopping here");
            break;
        };
        eprintln!(
            "{label:<18} {:>9.1} {:>8.4} {:>8.3} {:>9.4} {:>9.4}  {}",
            cur.infer_ms,
            m,
            p,
            dm,
            dp,
            verdict(nonf, m, p, dm, dp)
        );
        let v = verdict(nonf, m, p, dm, dp);
        prev = cur;
        // Once a stage blanks the frame, every later step is comparing against
        // nothing. Stop and say so, rather than reporting a cascade of
        // BLANK/DIFFERS that all describe the same failure.
        if v == "BLANK" || v == "NON-FINITE" {
            eprintln!(
                "
{label} left the frame blank — later steps are measured against that,                  so the sweep stops here. Fix this stage before judging the next."
            );
            break;
        }
    }

    // ── demosaic + CCM as one op, against the pair it replaces ──────────
    //
    // Separate sweep because the chain shape changes: two blocks become one.
    eprintln!(
        "\n{:<18} {:>9} {:>8} {:>8} {:>9} {:>9}  {}",
        "demosaic path", "infer ms", "mean", "peak", "mean|Δ|", "peak|Δ|", "verdict"
    );
    let Some(pair) = render(Fuse::default()) else {
        panic!("demosaic pair baseline failed");
    };
    let (pm, pp, _) = stats(&pair.data);
    eprintln!(
        "{:<18} {:>9.1} {:>8.4} {:>8.3} {:>9} {:>9}  {}",
        "demosaic + ccm", pair.infer_ms, pm, pp, "-", "-", "reference"
    );
    match render_demosaic_ccm(true) {
        Some(fused) => {
            let (m, p, nonf) = stats(&fused.data);
            match diff(&pair.data, &fused.data) {
                Some((dm, dp)) => eprintln!(
                    "{:<18} {:>9.1} {:>8.4} {:>8.3} {:>9.4} {:>9.4}  {}",
                    "isp.demosaic_ccm",
                    fused.infer_ms,
                    m,
                    p,
                    dm,
                    dp,
                    verdict(nonf, m, p, dm, dp)
                ),
                None => eprintln!("{:<18} frame size changed", "isp.demosaic_ccm"),
            }
        }
        None => eprintln!("{:<18} BUILD/RUN FAILED", "isp.demosaic_ccm"),
    }

    eprintln!(
        "\nDIFFERS is not automatically a bug — fused and primitive forms may\n\
         legitimately disagree at boundaries (edge replication vs zero padding).\n\
         BLANK or NON-FINITE always is."
    );
}
