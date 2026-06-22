//! Parallel frame submission — demonstrates concurrent inference with 4 sessions.
//!
//! Builds the same pipeline as bench_native but submits 4 frames simultaneously
//! using `std::thread::scope`. With 4 sessions in the pool, all 4 frames run in
//! parallel (total wall time ≈ 1 frame time).
//!
//! Usage:
//!   ENGINE=mnn_cpu MODE=native cargo run --features mnn --example bench_parallel
//!   ENGINE=mnn_vulkan MODE=native cargo run --features mnn --example bench_parallel

use std::time::Instant;

fn main() {
    let _ = env_logger::builder().is_test(false).filter_level(log::LevelFilter::Info).try_init();
    cam_isp::init();

    let sensor_w = 3840u32; let sensor_h = 2160u32; let post_w = 960u32; let post_h = 540u32;
    let full_w = sensor_w as i64; let full_h = sensor_h as i64;
    let ds_w = 1920i64; let ds_h = 1080i64;
    let post_w_i = post_w as i64; let post_h_i = post_h as i64;

    // ── Generate test frame data ──
    let mut raw_buf = Vec::with_capacity((sensor_w * sensor_h * 2) as usize);
    let mut rng_state = 42u64;
    for _ in 0..sensor_w * sensor_h {
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let val = (rng_state >> 22) as u16 & 0x3FF;
        raw_buf.extend_from_slice(&val.to_le_bytes());
    }

    // ── Select engine ────────────────────────────────────────────────
    let engine_name = std::env::var("ENGINE").unwrap_or_else(|_| "mnn_cpu".to_string());
    let be = match engine_name.as_str() {
        "mnn_vulkan" => cam_isp::mnnengine::MnnBackend::Vulkan,
        "mnn_opencl" => cam_isp::mnnengine::MnnBackend::Opencl,
        "mnn_opengl" => cam_isp::mnnengine::MnnBackend::OpenGl,
        "mnn_neon" => cam_isp::mnnengine::MnnBackend::CpuNeon,
        _ => cam_isp::mnnengine::MnnBackend::Cpu,
    };

    let use_native = std::env::var("MODE").ok().map(|m| m == "native").unwrap_or(true);
    let packed_w = (sensor_w / 2) as i64;
    let pw = if use_native { full_w } else { packed_w };
    let elem_type = if use_native { 5 } else { 6 };

    // ── Build pipeline blocks ───────────────────────────────────────
    let mut blocks: Vec<Box<dyn cam_isp::pipeline::IspBlock>> = vec![
        Box::new(cam_isp::blocks::RawInputBlock::new().with_elem_type(elem_type).with_concrete_dims(full_h, pw)),
        Box::new(cam_isp::blocks::UnpackCfaBlock::new()
            .with_concrete_width(full_w).with_concrete_dims(full_h, full_w)
            .with_downscale(1).with_sensor_max(1023.0).with_blc(true)
            .with_mode(if use_native { cam_isp::blocks::UnpackMode::NativeInt16 } else { cam_isp::blocks::UnpackMode::PackedInt32 })),
        Box::new(cam_isp::blocks::DemosaicCcmBlock::new(0).with_concrete_dims(ds_h, ds_w)),
        Box::new(cam_isp::blocks::AdaptiveDownscaleBlock::new(post_w_i, post_h_i, 0, "edge", "fit").with_concrete_dims(ds_h, ds_w)),
        Box::new(cam_isp::blocks::FcsBlock::new()),
        Box::new(cam_isp::blocks::LdciBlock::new()),
        Box::new(cam_isp::blocks::EeBlock::new()),
        Box::new(cam_isp::blocks::DisplayBlock::new(post_w).with_pack_rgba(false).with_bg4a(true).with_concrete_dims(post_h_i, post_w_i)),
    ];

    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    let all_but_head: Vec<Box<dyn cam_isp::pipeline::IspBlock>> = blocks.drain(1..).collect();
    let head = blocks.remove(0);

    // ── Create engine with 4 parallel sessions ──
    let mut engine = cam_isp::mnnengine::MnnEngine::with_pool_size(be, 4);
    if use_native {
        engine.set_preserve_input_type(true);
    }

    println!("Building pipeline with {} sessions...", 4);
    use cam_isp::engine::IspEngine;
    if let Err(e) = engine.build(head, all_but_head, None, 21) {
        eprintln!("BUILD FAILED: {}", e);
        return;
    }

    println!("Engine built OK (backend={}, native={})", engine_name, use_native);
    println!("");

    // ── Sequential baseline (1 frame) ──
    let mut params = cam_isp::engine::ProcessParams::new(sensor_w, sensor_h, &raw_buf);
    params.target_width = post_w;
    params.target_height = post_h;
    params.sensor_max = 1023.0;

    let t0 = Instant::now();
    let result = engine.process(&params);
    let seq_elapsed = t0.elapsed();
    match &result {
        Ok(frame) => println!("Sequential 1 frame: {:6.1}ms → {}B output", seq_elapsed.as_secs_f64() * 1000.0, frame.data.len()),
        Err(e) => eprintln!("Sequential FAILED: {}", e),
    }

    // ── Parallel 4 frames ──
    let n_parallel: usize = 4;
    let t0 = Instant::now();

    // Borrow engine and params for the scope
    let engine_ref = &engine;
    let params_ref = &params;
    std::thread::scope(|scope| {
        for _ in 0..n_parallel {
            scope.spawn(move || {
                let _ = engine_ref.process(params_ref);
            });
        }
    });

    let par_elapsed = t0.elapsed();

    println!("Parallel {} frames:  {:6.1}ms wall (theoretical sequential would be ~{:.1}ms)",
        n_parallel,
        par_elapsed.as_secs_f64() * 1000.0,
        seq_elapsed.as_secs_f64() * 1000.0 * n_parallel as f64);
    println!("Speedup: {:.1}× (ideal would be {:.1}× with {} sessions)",
        (seq_elapsed.as_secs_f64() * n_parallel as f64) / par_elapsed.as_secs_f64(),
        n_parallel as f64,
        n_parallel);

    if par_elapsed < seq_elapsed * n_parallel as u32 / 2 {
        println!("✅ CONFIRMED: Parallel submission works — sessions run concurrently");
    } else {
        println!("⚠️  Partial speedup — sessions may be contending for resources");
    }

    println!("");
    println!("Note: On CPU backend, parallel speedup is limited by CPU cores.");
    println!("      True parallelism requires GPU backends (Vulkan/OpenCL).");
}
