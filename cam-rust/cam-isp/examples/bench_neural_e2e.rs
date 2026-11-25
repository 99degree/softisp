//! End-to-end neural controller + ISP pipeline benchmark.
//!
//! Measures the complete per-frame latency of running the neural controller
//! (ISP rectifier, MNN CPU) followed by the full ISP pipeline (MNN/Vulkan GPU).
//!
//! Flow per frame:
//!   1. NeuralController::analyze_and_update() → IspParams (MNN CPU, ~1 ms)
//!   2. IspEngine::process() → IspFrame (MNN/Vulkan GPU, full-frame pixels)
//!
//! Reports avg/p50/p99 latency and FPS for HD, FHD, and 4K resolutions.
//!
//! Run:
//!   cargo run --release --example bench_neural_e2e -p cam-isp --features rectifier,mnn

use cam_isp::blocks::*;
use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::MnnBackend;
use cam_isp::neural_controller::NeuralController;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use std::time::Instant;

fn build_chain(w: u32, h: u32) -> (Box<dyn IspBlock>, Vec<Box<dyn IspBlock>>) {
    let full_w = w as i64;
    let full_h = h as i64;
    let pipe_w = 1920u32;
    let pipe_h = 1080u32;
    let post_w = 960u32;
    let post_h = 540u32;

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        // 1. INT16 input: [1,1,H,W] — sensor native format
        Box::new(RawInput16Block::new().with_concrete_dims(full_h, full_w)),
        // 2. Unpack CFA via Conv: NativeInt16 + fast_unpack
        Box::new(
            UnpackCfaBlock::new()
                .with_mode(cam_isp::blocks::UnpackMode::NativeInt16)
                .with_fast_unpack(true)
                .with_concrete_dims(full_h, full_w)
                .with_downscale(4)
                .with_sensor_max(1023.0)
                .with_blc(true),
        ),
        // 3. DemosaicCcm at FHD: [1,4,1080,960] → [1,3,1080,960]
        Box::new(DemosaicCcmBlock::new(0).with_concrete_dims(pipe_h as i64, post_w as i64)),
        // 4–7. Cosmetic post-processing at FHD
        Box::new(FcsBlock::new()),
        Box::new(LdciBlock::new()),
        Box::new(EeBlock::new()),
        // 8. Display output (FloatRgb: [1,3,H,W] f32 RGB [0,1])
        Box::new(
            DisplayBlock::new(post_w)
                .with_output_format(OutputFormat::FloatRgb)
                .with_concrete_dims(post_h as i64, post_w as i64),
        ),
    ];

    GraphComposer::wire_blocks(&mut blocks);
    let head = blocks.remove(0);
    (head, blocks)
}

fn build_engine(
    head: Box<dyn IspBlock>,
    aux: Vec<Box<dyn IspBlock>>,
) -> Box<dyn IspEngine> {
    let mut engine = cam_isp::mnnengine::MnnEngine::new(MnnBackend::Vulkan);
    engine.build(head, aux, None, 21).expect("engine.build failed");
    Box::new(engine)
}

fn main() {
    cam_isp::init();

    // Initialize neural controller (MNN CPU)
    let mut neural_ctrl = NeuralController::with_default_model();
    let neural_model_loaded = neural_ctrl.has_model();
    println!(
        "Neural controller: {}",
        if neural_model_loaded {
            "loaded (default model)"
        } else {
            "fallback to rule-based (no model)"
        }
    );

    // Build two identical engines for benchmarking (single and dual)
    let (head_a, aux_a) = build_chain(3840, 2160);
    let (head_b, aux_b) = build_chain(3840, 2160);

    let engine_a = build_engine(head_a, aux_a);
    let engine_b = build_engine(head_b, aux_b);

    let mut dual = cam_isp::dual_engine::DualGpuEngine::new(engine_a, engine_b);

    let sensor_w = 3840u32;
    let sensor_h = 2160u32;
    let n_frames = 50;

    // Generate 4K Bayer test data as INT16
    let mut raw_buf = Vec::with_capacity((sensor_w * sensor_h * 2) as usize);
    let mut rng_state = 42u64;
    for _ in 0..sensor_w * sensor_h {
        rng_state = rng_state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let val = (rng_state >> 22) as u16 & 0x3FF; // 10-bit
        raw_buf.extend_from_slice(&val.to_le_bytes());
    }
    let raw = raw_buf;

    // Shared params
    let mut params = ProcessParams::new(sensor_w, sensor_h, &raw);
    params.target_width = 1920;
    params.target_height = 1080;
    params.sensor_max = 1023.0;
    params.output_format = OutputFormat::FloatRgb;

    // Warmup (3 frames)
    for _ in 0..3 {
        let _ = neural_ctrl.analyze_and_update(&dummy_frame(sensor_w, sensor_h, &raw));
        let _ = dual.next().process(&params);
    }

    // Timed run: measure end-to-end per-frame latency
    let mut sum_ms = 0.0f64;
    let mut min_ms = f64::INFINITY;
    let mut max_ms = 0.0f64;
    let mut neural_sum_ms = 0.0f64;
    let mut errors = 0u32;

    println!("\nRunning {} frames end-to-end (neural controller + MNN/Vulkan pipeline)...\n", n_frames);

    let t0 = Instant::now();
    for i in 0..n_frames {
        // Step 1: Neural controller inference (CPU)
        let t_neural = Instant::now();
        let dummy = dummy_frame(sensor_w, sensor_h, &raw);
        let isp_params = neural_ctrl.analyze_and_update(&dummy);
        let neural_ms = t_neural.elapsed().as_secs_f64() * 1000.0;
        neural_sum_ms += neural_ms;

        // Step 2: ISP pipeline inference (MNN/Vulkan GPU)
        // Inject neural controller outputs into ProcessParams
        let mut frame_params = params.clone();
        params.ccm_matrix = Some(isp_params.ccm.matrix);
        params.awb_gains = Some([isp_params.wb.r, isp_params.wb.g, isp_params.wb.b]);
        params.isp_params = Some(isp_params);

        let t_frame = Instant::now();
        let result = dual.next().process(&params);
        let frame_ms = t_frame.elapsed().as_secs_f64() * 1000.0;
        let total_ms = neural_ms + frame_ms;

        match result {
            Ok(frame) => {
                sum_ms += total_ms;
                min_ms = min_ms.min(total_ms);
                max_ms = max_ms.max(total_ms);
                if i < 5 || i == n_frames - 1 {
                    println!(
                        "  [{:2}] {:4}×{:<4}  neural={:6.1}ms  isp={:6.1}ms  total={:7.1}ms  ({} bytes)",
                        i,
                        frame.width,
                        frame.height,
                        neural_ms,
                        frame_ms,
                        total_ms,
                        frame.data.len()
                    );
                }
            }
            Err(e) => {
                errors += 1;
                println!(
                    "  [{:2}] ERROR: {}  ({:.1}ms neural)",
                    i, e, neural_ms
                );
            }
        }
    }
    let total_elapsed = t0.elapsed().as_secs_f64();

    println!();
    println!("=== End-to-End Neural Controller + ISP Pipeline Results ===");
    println!("  Frames:       {}", n_frames);
    println!("  Errors:       {}", errors);
    println!("  Neural model: {}", if neural_model_loaded { "loaded" } else { "fallback (rule-based)" });
    println!("  Neural avg:   {:.2} ms/frame", neural_sum_ms / n_frames as f64);
    println!("  ISP avg:      {:.2} ms/frame", (sum_ms - neural_sum_ms) / n_frames as f64);
    println!("  Total avg:    {:.2} ms/frame", sum_ms / n_frames as f64);
    println!("  Min total:    {:.2} ms", min_ms);
    println!("  Max total:    {:.2} ms", max_ms);
    println!("  Throughput:   {:.1} FPS", n_frames as f64 / total_elapsed);
    println!("  (wall-clock total: {:.2} s)", total_elapsed);

    let avg_total = sum_ms / n_frames as f64;
    if avg_total < 33.3 {
        println!("  ✅ Exceeds 30fps target!");
    } else {
        println!("  ❌ Misses 30fps target (33.3ms/frame)");
    }

    // Compare against single engine (no neural controller overhead)
    println!();
    println!("  To compare with pure pipeline (no neural controller):");
    println!("    cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn");
}

/// Create a dummy IspFrame for neural controller metadata extraction.
fn dummy_frame(w: u32, h: u32, raw: &[u8]) -> cam_isp::pipeline::IspFrame {
    cam_isp::pipeline::IspFrame {
        params: cam_isp::isp_params::IspParams::default(),
        data: raw.to_vec(),
        width: w,
        height: h,
        format: cam_types::FrameFormat::RawSensor,
        float_data: None,
        aux: None,
        timestamp_ns: 0,
        prep_duration_ns: 0,
        inference_duration_ns: 0,
        total_duration_ns: 0,
        external_data: None,
    }
}
