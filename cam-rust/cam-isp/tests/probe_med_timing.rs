//! Where does the 850 ms go? Report whether it is the GPU or the driver.
//!
//! The device log shows ~850 ms per frame inside `engine.process()` at
//! 2592x1940, against GPU-side numbers two orders of magnitude smaller. This
//! builds the real MED chain and times:
//!
//!   prep   — set_extra_inputs + tensor upload, all CPU
//!   infer  — the MNN session run (GPU + driver submission)
//!   total  — the whole call
//!
//! The engine already reports those three on the returned `IspFrame`. If
//! `infer` dominates, the question is dispatch geometry (workgroup size) or op
//! count. If `prep` dominates, it is the host-side upload path.
//!
//! Run on a device (needs the real Vulkan driver):
//!   cargo test -p cam-isp --test probe_med_timing --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::profile::PipelineProfile;

#[test]
#[ignore]
fn where_the_time_goes() {
    cam_isp::init();

    // The sensor size the app actually uses.
    let (w, h) = (2592u32, 1940u32);
    let sample: u16 = 512;

    let mut profile = PipelineProfile::MED;
    // opset_mode 2 = MnnIspMin, which emits `isp.*` Extra ops that MNN's
    // VulkanFuse runs as SPIR-V shaders instead of the standard ONNX op set.
    // MED defaults to 0 (Primitive) → every stage is a separate ONNX op.
    if let Ok(m) = std::env::var("PROBE_OPSET") {
        profile.opset_mode = m.parse().unwrap_or(0);
    }
    profile.output_format = OutputFormat::FloatRgb;
    eprintln!(
        "opset_mode = {} ({})",
        profile.opset_mode,
        match profile.opset_mode {
            0 => "Primitive",
            1 => "Custom",
            2 => "MnnIspMin",
            _ => "?",
        }
    );

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")
        .expect("mnn_vulkan engine not registered");
    engine.build(head, blocks, None, 21).expect("build");

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

    // First call builds/resizes the per-slot tensors; time only steady state.
    let mut last = None;
    for i in 0..4 {
        let t0 = std::time::Instant::now();
        let frame = engine.process(&params).expect("process");
        let wall = t0.elapsed();
        eprintln!(
            "frame {i}: wall {:8.1} ms   prep {:8.1} ms   infer {:8.1} ms   out {}x{} {} B",
            wall.as_secs_f64() * 1000.0,
            frame.prep_duration_ns as f64 / 1e6,
            frame.inference_duration_ns as f64 / 1e6,
            frame.width,
            frame.height,
            frame.data.len()
        );
        last = Some(frame);
    }

    let f = last.expect("no frame");
    let total = f.prep_duration_ns as f64 / 1e6 + f.inference_duration_ns as f64 / 1e6;
    if total > 0.0 {
        eprintln!(
            "\ninference is {:.0}% of the measured pipeline",
            100.0 * f.inference_duration_ns as f64 / 1e6 / total
        );
    }
    eprintln!(
        "pixels: {} ({} Mpx), infer throughput {:.1} Mpx/s",
        (w / 2) as u64 * (h / 2) as u64,
        (w / 2) as f64 * (h / 2) as f64 / 1e6,
        ((w / 2) as f64 * (h / 2) as f64 / 1e6) / (f.inference_duration_ns as f64 / 1e9).max(1e-9)
    );
}
