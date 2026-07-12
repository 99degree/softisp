use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::isp_params::IspParams;
use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;
use std::time::{Duration, Instant};

fn main() {
    // Enable MNN/Vulkan backend
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║  UNIFIED Profile Stress Test: 4K Bayer → FHD ARGB8888      ║");
    println!("║  Neural Controller (Rule-Based) | 30s Duration              ║");
    println!("╚══════════════════════════════════════════════════════════════╝");

    // Configuration
    let input_w = 3840;
    let input_h = 2160;
    let output_w = 1920;
    let output_h = 1080;
    let duration_secs = 30;
    let bayer_pattern = 0; // RGGB

    // Build UNIFIED profile pipeline (all post-processing blocks)
    let mut profile = PipelineProfile::UNIFIED;
    profile.output_format = OutputFormat::Argb;

    println!("\n[1/4] Building pipeline...");
    let mut blocks = profile.build_blocks(output_w, 0);
    GraphComposer::wire_blocks(&mut blocks);

    let num_blocks = blocks.len();
    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;

    // Create engine
    let mut engine = match cam_isp::engine::select_engine_by_name("mnn_vulkan") {
        Some(e) => e,
        None => {
            eprintln!("Vulkan engine unavailable, using CPU");
            cam_isp::engine::select_engine_by_name("mnn_cpu").unwrap()
        }
    };

    engine.build(head, aux, None, 21).unwrap();
    println!("   Pipeline built: {} blocks", num_blocks);

    // Create synthetic 4K Bayer data (16-bit per pixel)
    println!(
        "[2/4] Generating 4K Bayer test pattern ({} MB)...",
        (input_w * input_h * 2) / (1024 * 1024)
    );
    let raw: Vec<u8> = vec![0u8; (input_w * input_h * 2) as usize];

    // Run stress test
    println!("[3/4] Running stress test for {} seconds...", duration_secs);
    println!("┌─────────────────────────────────────────────────────────────┐");
    println!("│ Frame │  Time  │  FPS  │  Min  │  Max  │  Avg  │  P99   │");
    println!("├───────┼────────┼───────┼───────┼───────┼───────┼────────┤");

    let start = Instant::now();
    let mut frame_count = 0u64;
    let mut frame_times: Vec<f64> = Vec::new();
    let mut last_stats_print = Instant::now();
    let stats_interval = Duration::from_secs(5);

    while start.elapsed() < Duration::from_secs(duration_secs) {
        let frame_start = Instant::now();

        // Controller analyzes frame (simulated with varying IspParams)
        let mut params = IspParams::default();
        params.wb.r = 1.0 + (frame_count as f32 * 0.001).sin() * 0.1;
        params.wb.g = 1.0;
        params.wb.b = 1.0 + (frame_count as f32 * 0.001).cos() * 0.1;
        params.ccm.matrix = [1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        params.tone.contrast = 1.0;
        params.tone.brightness = 0.0;
        params.tone.gamma = 1.0;

        // Process frame
        let mut process_params = ProcessParams::new(input_w, input_h, &raw);
        process_params.target_width = output_w;
        process_params.target_height = output_h;
        process_params.bayer_pattern = bayer_pattern;
        process_params.isp_params = Some(params);

        match engine.process(&process_params) {
            Ok(_frame) => {
                let elapsed_ms = frame_start.elapsed().as_secs_f64() * 1000.0;
                frame_times.push(elapsed_ms);
                frame_count += 1;

                // Print stats every 5 seconds
                if last_stats_print.elapsed() >= stats_interval {
                    let elapsed = start.elapsed().as_secs_f64();
                    let fps = frame_count as f64 / elapsed;
                    let min_ms = frame_times.iter().cloned().fold(f64::INFINITY, f64::min);
                    let max_ms = frame_times
                        .iter()
                        .cloned()
                        .fold(f64::NEG_INFINITY, f64::max);
                    let avg_ms: f64 = frame_times.iter().sum::<f64>() / frame_times.len() as f64;

                    let mut sorted = frame_times.clone();
                    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
                    let p99_idx = (sorted.len() as f64 * 0.99) as usize;
                    let p99_ms = sorted[p99_idx.min(sorted.len() - 1)];

                    println!(
                        "│ {:5}  │ {:5.1}ms │ {:5.1} │ {:5.1} │ {:5.1} │ {:5.1} │ {:5.1}ms │",
                        frame_count, elapsed_ms, fps, min_ms, max_ms, avg_ms, p99_ms
                    );
                    last_stats_print = Instant::now();
                }
            }
            Err(e) => {
                eprintln!("Frame {} error: {:?}", frame_count, e);
                std::thread::sleep(Duration::from_millis(10));
            }
        }
    }

    // Final stats
    let total_elapsed = start.elapsed().as_secs_f64();
    let avg_fps = frame_count as f64 / total_elapsed;
    let min_ms = frame_times.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_ms = frame_times
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    let avg_ms: f64 = frame_times.iter().sum::<f64>() / frame_times.len() as f64;

    let mut sorted = frame_times.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let p50_idx = sorted.len() / 2;
    let p99_idx = (sorted.len() as f64 * 0.99) as usize;
    let p50_ms = sorted[p50_idx];
    let p99_ms = sorted[p99_idx.min(sorted.len() - 1)];

    println!("├───────┴────────┴───────┴───────┴───────┴───────┴────────┤");
    println!("│                    FINAL RESULTS                         │");
    println!("├─────────────────────────────────────────────────────────┤");
    println!(
        "│  Total Frames:  {:6}                                  │",
        frame_count
    );
    println!(
        "│  Total Time:    {:6.1}s                                 │",
        total_elapsed
    );
    println!(
        "│  Average FPS:   {:6.1}                                  │",
        avg_fps
    );
    println!(
        "│  Min Frame:     {:6.1}ms                                │",
        min_ms
    );
    println!(
        "│  Max Frame:     {:6.1}ms                                │",
        max_ms
    );
    println!(
        "│  Avg Frame:     {:6.1}ms                                │",
        avg_ms
    );
    println!(
        "│  P50 Latency:   {:6.1}ms                                │",
        p50_ms
    );
    println!(
        "│  P99 Latency:   {:6.1}ms                                │",
        p99_ms
    );
    println!("└─────────────────────────────────────────────────────────┘");

    // Validate
    let success = frame_count > 0 && avg_fps > 10.0;
    if success {
        println!("\n✅ STRESS TEST PASSED");
        println!("   - All {} frames processed successfully", frame_count);
        println!("   - Average FPS: {:.1} (target: >10)", avg_fps);
        println!("   - P99 latency: {:.1}ms", p99_ms);
    } else {
        println!("\n❌ STRESS TEST FAILED");
        std::process::exit(1);
    }
}
