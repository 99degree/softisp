//! How much of the inference time is per-op dispatch overhead?
//!
//! Individual stage timings drift more than the stage costs (the same probe run
//! twice gave +146 ms and -12 ms for the same stage), so this measures one
//! unambiguous quantity two ways:
//!
//!   at 64x48    — 3 kPx of pixel work, i.e. none. Whatever a chain costs here
//!                 is fixed overhead: command-buffer submission, descriptor
//!                 updates and the GPU->CPU fence wait, once per op.
//!   at 648x485  — real pixel work, for comparison.
//!
//! Configurations are interleaved and repeated so thermal drift and scheduler
//! noise cancel instead of being attributed to a stage.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_op_overhead --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use std::time::Instant;

use cam_isp::blocks::*;
use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::pipeline::{BlockOpsetMode, GraphComposer, IspBlock};

/// Chain with `extras` blocks between demosaic and display.
fn time_chain(w: u32, h: u32, extras: usize) -> Option<f64> {
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
    ];
    for i in 0..extras {
        blocks.push(match i % 3 {
            0 => Box::new(CcmBlock::with_instance("ccm")),
            1 => Box::new(ToneBlock::new()),
            _ => Box::new(EeBlock::new().with_opset_mode(BlockOpsetMode::Primitive)),
        });
    }
    blocks.push(Box::new(
        DisplayBlock::new(w).with_output_format(OutputFormat::FloatRgb),
    ));
    GraphComposer::wire_blocks(&mut blocks);

    let head = blocks.remove(0);
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    engine.build(head, blocks, None, 21).ok()?;

    let mut buf = vec![0u8; (w as usize) * (h as usize) * 2];
    for c in buf.as_chunks_mut::<2>().0 {
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
    engine.process(&params).ok()?;

    // Median of 9 timed runs.
    let mut samples = Vec::new();
    for _ in 0..9 {
        let t0 = Instant::now();
        let f = engine.process(&params).ok()?;
        let _ = t0.elapsed();
        samples.push(f.inference_duration_ns as f64 / 1e6);
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Some(samples[samples.len() / 2])
}

#[test]
#[ignore]
fn per_op_overhead() {
    cam_isp::init();

    // (label, width, height, extra blocks)
    let configs = [
        ("64x48   0 extra", 64u32, 48u32, 0usize),
        ("64x48   6 extra", 64, 48, 6),
        ("64x48  12 extra", 64, 48, 12),
        ("648x485 0 extra", 648, 485, 0),
        ("648x485 12 extra", 648, 485, 12),
    ];

    // Interleave two passes; keep both so drift is visible.
    let mut pass1 = Vec::new();
    let mut pass2 = Vec::new();
    for round in 0..2 {
        for (label, w, h, extras) in configs {
            if let Some(ms) = time_chain(w, h, extras) {
                if round == 0 {
                    pass1.push((label, extras, w * h, ms));
                } else {
                    pass2.push((label, extras, w * h, ms));
                }
            }
        }
    }

    eprintln!(
        "\n{:<18} {:>6} {:>10} {:>10}",
        "chain", "extra", "pass1 ms", "pass2 ms"
    );
    for (a, b) in pass1.iter().zip(pass2.iter()) {
        eprintln!("{:<18} {:>6} {:>10.1} {:>10.1}", a.0, a.1, a.3, b.3);
    }

    // Slope of ms against extra-block count at 64x48 — pure dispatch overhead,
    // since 3 kPx of pixels cost nothing.
    let tiny: Vec<(usize, f64)> = pass1
        .iter()
        .chain(pass2.iter())
        .filter(|(_, _, pixels, _)| *pixels <= 64 * 48)
        .map(|(_, extras, _, ms)| (*extras, *ms))
        .collect();
    if tiny.len() >= 2 {
        let n = tiny.len() as f64;
        let sx: f64 = tiny.iter().map(|(x, _)| *x as f64).sum();
        let sy: f64 = tiny.iter().map(|(_, y)| *y).sum();
        let sxx: f64 = tiny.iter().map(|(x, _)| (*x as f64) * (*x as f64)).sum();
        let sxy: f64 = tiny.iter().map(|(x, y)| (*x as f64) * y).sum();
        let slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        let intercept = (sy - slope * sx) / n;
        eprintln!(
            "\nper-op overhead (64x48, least squares over {} points): \
             {slope:.2} ms/op, fixed {intercept:.1} ms",
            tiny.len()
        );
    }
}
