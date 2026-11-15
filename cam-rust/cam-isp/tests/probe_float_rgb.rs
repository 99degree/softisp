//! Is the Conv-free display path (`FloatRgb`) producing correct values?
//!
//! `FloatRgb` emits `Pow + Clip` → `[1,3,H,W]` f32 in [0,1], no convolution.
//! Every `Bgra`/`Rgba`/… format instead emits a `Conv(1x1)` that applies the
//! format permutation and the ×255 scale — and on Vulkan that Conv's kernel
//! comes out zero regardless of whether the weights are graph inputs or
//! initializers, so those formats render black.
//!
//! If `FloatRgb` is correct, the fix is to do the permutation in Rust and skip
//! the Conv entirely.

#![cfg(feature = "mnn")]

use cam_isp::blocks::*;
use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::pipeline::{GraphComposer, IspBlock};

#[test]
#[ignore]
fn float_rgb_is_correct() {
    cam_isp::init();

    let (w, h) = (64u32, 48u32);
    // Mid-grey Bayer: the demosaic should give R=G=B~0.5.
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
        Box::new(DisplayBlock::new(w).with_output_format(OutputFormat::FloatRgb)),
    ];
    GraphComposer::wire_blocks(&mut blocks);

    let head = blocks.remove(0);
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").expect("mnn_vulkan");
    // Leave fp16 storage at its production default: the device runs with it
    // enabled, so a probe that silently disables it proves nothing about the
    // preview.
    engine.build(head, blocks, None, 21).expect("build");

    let mut buf = vec![0u8; (w * h * 2) as usize];
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

    let f = engine.process(&params).expect("process");
    // Demosaic halves the Bayer frame: 64x48 in becomes 32x24 RGB out. The
    // `IspFrame` dimensions for a raw-output format echo the caller's target,
    // so derive the plane size from the byte count instead.
    let fw = (w / 2) as usize;
    let fh = (h / 2) as usize;
    let n = fw * fh;
    eprintln!(
        "frame reported {}x{}, data {} bytes = {} f32 (expect {})",
        f.width,
        f.height,
        f.data.len(),
        f.data.len() / 4,
        n * 3
    );

    let vals: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    eprintln!("f32 values: {} (expect {})", vals.len(), n * 3);

    if vals.len() >= n * 3 {
        for (i, name) in ["R", "G", "B"].iter().enumerate() {
            let plane = &vals[i * n..(i + 1) * n];
            let mean: f64 = plane.iter().map(|v| *v as f64).sum::<f64>() / n as f64;
            let min = plane.iter().cloned().fold(f32::MAX, f32::min);
            let max = plane.iter().cloned().fold(f32::MIN, f32::max);
            eprintln!(
                "  {name}: mean {mean:.4} min {min:.4} max {max:.4} first4 {:?}",
                &plane[..4]
            );
        }
        eprintln!();
        eprintln!("=> planar [1,3,H,W] f32 in [0,1]. Host-side BGRA would be:");
        let (r, g, b) = (vals[0], vals[n], vals[2 * n]);
        eprintln!(
            "   pixel0 BGRA = ({},{},{},255) from R={r:.3} G={g:.3} B={b:.3}",
            (b * 255.0) as u8,
            (g * 255.0) as u8,
            (r * 255.0) as u8
        );
    }
}
