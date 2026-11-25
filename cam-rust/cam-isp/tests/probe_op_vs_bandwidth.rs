//! Distinguish "bandwidth-bound" from "per-dispatch latency bound".
//!
//! The MED chain is 132 ops and takes ~390 ms to infer at 1296x970. Both
//! explanations predict that:
//!
//!   * bandwidth — 132 ops x 40 MB of 4-channel f32 traffic = 5.3 GB, which at
//!     the 13.7 GB/s this implies is exactly mobile LPDDR's ceiling;
//!   * dispatch  — 132 kernels x ~3 ms each is the same 390 ms.
//!
//! They are separated by pixel count versus op count. This times chains of
//! 2, 4 and 8 ops at the *same* resolution. If the cost is dispatch, it grows
//! with op count. If it is bandwidth, it grows too — so it also times one chain
//! at 1/4 the pixels, where bandwidth-bound work must drop ~4x.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_op_vs_bandwidth --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use std::time::Instant;

use cam_isp::blocks::*;
use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::pipeline::{GraphComposer, IspBlock};

fn time_chain(tag: &str, w: u32, h: u32, extra: Vec<Box<dyn IspBlock>>) {
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
    blocks.extend(extra);
    blocks.push(Box::new(
        DisplayBlock::new(w).with_output_format(OutputFormat::FloatRgb),
    ));
    GraphComposer::wire_blocks(&mut blocks);

    let head = blocks.remove(0);
    let n_ops = blocks.len();
    let mut engine = match cam_isp::engine::select_engine_by_name("mnn_vulkan") {
        Some(e) => e,
        None => return,
    };
    if engine.build(head, blocks, None, 21).is_err() {
        eprintln!("{tag:<34} BUILD FAIL");
        return;
    }

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
        input_buffer: None,
        output_buffer: None,
        input_ahb: None,
        output_ahb: None,
    };

    // Warm up (first call allocates and resizes).
    if engine.process(&params).is_err() {
        eprintln!("{tag:<34} RUN FAIL");
        return;
    }

    let n = 3;
    let t0 = Instant::now();
    let mut last = None;
    for _ in 0..n {
        last = engine.process(&params).ok();
    }
    let per = t0.elapsed().as_secs_f64() * 1000.0 / n as f64;

    let infer = last
        .as_ref()
        .map(|f| f.inference_duration_ns as f64 / 1e6)
        .unwrap_or(0.0);
    let mpx = (w / 2) as f64 * (h / 2) as f64 / 1e6;
    eprintln!(
        "{tag:<34} {n_ops:>3} blocks  wall {per:7.2} ms   infer {infer:7.2} ms   {mpx:5.3} Mpx   {:.2} ms/block",
        per / n_ops as f64
    );
}

#[test]
#[ignore]
fn ops_or_bandwidth() {
    cam_isp::init();

    let (w, h) = (1296u32, 970u32);
    let (qw, qh) = (648u32, 485u32);

    // Same resolution, growing block count.
    time_chain("5 blocks  @1296x970", w, h, vec![]);
    time_chain(
        "7 blocks  @1296x970",
        w,
        h,
        vec![
            Box::new(CcmBlock::with_instance("ccm")),
            Box::new(ToneBlock::new()),
        ],
    );
    time_chain(
        "11 blocks @1296x970",
        w,
        h,
        vec![
            Box::new(CcmBlock::with_instance("ccm")),
            Box::new(ToneBlock::new()),
            Box::new(SaturationBlock::new_default()),
            Box::new(EeBlock::new().with_opset_mode(cam_isp::pipeline::BlockOpsetMode::Primitive)),
            Box::new(GammaBlock::new(2.2)),
            Box::new(SharpenBlock::new(0.5)),
        ],
    );

    // Tiny frames: whatever cost remains cannot be pixel work, so it is the
    // per-inference fixed overhead (driver submit + GPU->CPU sync).
    eprintln!();
    time_chain("5 blocks  @256x192  (tiny)", 256, 192, vec![]);

    // Tiny frames with a growing op count: pixel work is ~0 here, so any growth
    // is per-op dispatch overhead as opposed to per-inference overhead.
    eprintln!();
    time_chain("5 blocks  @64x48 (tiny)", 64, 48, vec![]);
    time_chain(
        "7 blocks  @64x48 (tiny)",
        64,
        48,
        vec![
            Box::new(CcmBlock::with_instance("ccm")),
            Box::new(ToneBlock::new()),
        ],
    );
    time_chain(
        "11 blocks @64x48 (tiny)",
        64,
        48,
        vec![
            Box::new(CcmBlock::with_instance("ccm")),
            Box::new(ToneBlock::new()),
            Box::new(SaturationBlock::new_default()),
            Box::new(EeBlock::new().with_opset_mode(cam_isp::pipeline::BlockOpsetMode::Primitive)),
            Box::new(GammaBlock::new(2.2)),
            Box::new(SharpenBlock::new(0.5)),
        ],
    );

    // Quarter the pixels, same chain as the first case.
    eprintln!();
    time_chain("5 blocks  @648x485  (1/4 px)", qw, qh, vec![]);
    time_chain(
        "11 blocks @648x485  (1/4 px)",
        qw,
        qh,
        vec![
            Box::new(CcmBlock::with_instance("ccm")),
            Box::new(ToneBlock::new()),
            Box::new(SaturationBlock::new_default()),
            Box::new(EeBlock::new().with_opset_mode(cam_isp::pipeline::BlockOpsetMode::Primitive)),
            Box::new(GammaBlock::new(2.2)),
            Box::new(SharpenBlock::new(0.5)),
        ],
    );
}
