//! Benchmark: Interpolation demosaic (FHD) on Vulkan GPU
//! Pipeline: RawInput(INT32) → BayerDemosaic → FCS → Display
//!
//! Usage: HW=1920x1080 ALGO=bilinear cargo run --release --example bench_interp_demosaic
//!   ALGO: binning | bilinear | mhc  (default: bilinear)
//!   HW:   WxH  (default: 1920x1080)

use std::time::Instant;
use cam_isp::blocks::{BayerDemosaicBlock, DemosaicAlgo};

fn main() {
    let _ = env_logger::builder().is_test(false).filter_level(log::LevelFilter::Info).try_init();
    cam_isp::init();

    let hw = std::env::var("HW").unwrap_or_else(|_| "1920x1080".to_string());
    let parts: Vec<i64> = hw.split('x').map(|s| s.parse().unwrap()).collect();
    let (w, h) = (parts[0], parts[1]);
    let algo = DemosaicAlgo::from_str(&std::env::var("ALGO").unwrap_or_else(|_| "bilinear".to_string()));
    let oh = algo.output_height(h);
    let ow = algo.output_width(w);

    eprintln!("=== Bayer Demosaic Benchmark ===");
    eprintln!("  Algo:   {}", algo.as_str());
    eprintln!("  Input:  {}x{} INT32 Bayer", w, h);
    eprintln!("  Output: {}x{} RGB F32", ow, oh);

    // Build pipeline
    let mut blocks: Vec<Box<dyn cam_isp::pipeline::IspBlock>> = vec![
        Box::new(cam_isp::blocks::RawInputBlock::new().with_elem_type(6).with_concrete_dims(h, w)),
        Box::new(BayerDemosaicBlock::new()
            .with_algorithm(algo)
            .with_sensor_max(1023.0)
            .with_concrete_dims(h, w)),
        Box::new(cam_isp::blocks::FcsBlock::new()),
        Box::new(cam_isp::blocks::DisplayBlock::new(ow as u32).with_concrete_dims(oh, ow)),
    ];

    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    let mut all = blocks;
    let head = all.remove(0);

    let engine_name = std::env::var("ENGINE").unwrap_or_else(|_| "mnn".to_string());
    let mut engine = match cam_isp::engine::select_engine_by_name(&engine_name) {
        Some(e) => e,
        None => match cam_isp::engine::select_engine() { Some(e) => e, None => Box::new(cam_isp::cpu::CpuEngine::new()) },
    };

    eprintln!("  Engine: {}", engine.backend_name());

    let result = engine.build(head, all, None, 17);
    if let Err(ref e) = result {
        eprintln!("  Build FAILED: {}", e);
        return;
    }
    eprintln!("  Build OK");

    // Generate test data as bytes
    let pixel_count = (w * h) as usize;
    let mut raw_buf = vec![0u8; pixel_count * 4];
    let mut rng = 42u64;
    for i in 0..pixel_count {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let val = ((rng >> 22) as u32) & 0x3FF;
        raw_buf[i*4..(i+1)*4].copy_from_slice(&val.to_le_bytes());
    }

    let params = cam_isp::engine::ProcessParams::new(w as u32, h as u32, &raw_buf);
    let mut params = params;
    params.sensor_max = 1023.0;

    // Warmup
    for _ in 0..3 {
        let _ = engine.process(&params);
    }

    // Benchmark
    let iterations = 20;
    let mut times = Vec::new();
    for i in 0..iterations {
        let t0 = Instant::now();
        let result = engine.process(&params);
        let elapsed = t0.elapsed();
        times.push(elapsed);
        if i == 0 {
            match &result {
                Ok(frame) => eprintln!("  frame 1: {:.1}ms  output {}x{} {} bytes",
                    elapsed.as_secs_f64() * 1000.0, frame.width, frame.height, frame.data.len()),
                Err(e) => eprintln!("  frame 1: ERROR: {}", e),
            }
        }
    }

    times.sort();
    let avg: std::time::Duration = times.iter().sum::<std::time::Duration>() / times.len() as u32;
    let median = times[times.len() / 2];
    let fps = 1.0 / avg.as_secs_f64();
    let mpixels = (w * h) as f64 / 1_000_000.0;

    eprintln!("\n=== Results ({}x{} = {:.1} MP) ===", w, h, mpixels);
    eprintln!("  Algo:    {}", algo.as_str());
    eprintln!("  Average: {:.1}ms/frame", avg.as_secs_f64() * 1000.0);
    eprintln!("  Median:  {:.1}ms/frame", median.as_secs_f64() * 1000.0);
    eprintln!("  FPS:     {:.1}", fps);
    eprintln!("  MP/s:    {:.1}", mpixels / avg.as_secs_f64());
}
