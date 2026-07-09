use isp_rectifier::{FrameMetadata, AutoExposure, AutoFocus, AutoWhiteBalance, OptimizedInference};
use std::time::Instant;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🚀 ISP Rectifier Rust Demo");
    println!("==========================");

    // Initialize inference engine
    let model_path = "fusedispcontroller_int8.onnx";
    let mut optimizer = OptimizedInference::new(model_path, true)?;
    
    println!("✅ Model loaded: {}", model_path);

    // Simulate camera frame metadata
    let metadata = FrameMetadata {
        histogram: vec![100; 256],  // Normalized histogram
        cct: 5500.0,                // Daylight
        wb_gains: [1.2, 1.0, 0.95], // Current WB
        ae: AutoExposure {
            exposure_time: 0.033,
            iso_gain: 2.0,
            target_brightness: 0.5,
        },
        af: AutoFocus {
            position: 0.5,
            sharpness: 0.75,
        },
        awb: AutoWhiteBalance {
            gains: [1.2, 1.0, 0.95],
            confidence: 0.9,
        },
        brightness: 0.6,
        contrast: 0.7,
        noise_level: 0.08,
        timestamp: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_nanos() as u64,
    };

    // Run inference
    let start = Instant::now();
    let params = optimizer.optimize(&metadata)?;
    let latency = start.elapsed();

    // Display results
    println!("\n📊 Inference Results:");
    println!("  Latency: {:.2} ms", latency.as_secs_f64() * 1000.0);
    println!("  WB Gains: [{:.3}, {:.3}, {:.3}]", 
             params.wb_r_gain, params.wb_g_gain, params.wb_b_gain);
    println!("  CCM:");
    for row in &params.ccm {
        println!("    [{:.3}, {:.3}, {:.3}]", row[0], row[1], row[2]);
    }
    println!("  Tone Curve: {:?}", params.tone_curve_lut);
    println!("  Zoom Factor: {:.3}", params.zoom_factor);

    // Apply to ISP registers
    let registers = isp_rectifier::inject_registers(&params, &Default::default());
    println!("\n🎛️  ISP Register Values:");
    println!("  WB R: 0x{:04X} ({:.3})", registers.wb_r_gain, params.wb_r_gain);
    println!("  WB G: 0x{:04X} ({:.3})", registers.wb_g_gain, params.wb_g_gain);
    println!("  WB B: 0x{:04X} ({:.3})", registers.wb_b_gain, params.wb_b_gain);
    println!("  CCM_00: 0x{:04X}", registers.ccm_00);
    println!("  Tone LUT: {:?}", registers.tone_lut);
    println!("  Zoom: 0x{:04X} ({:.3})", registers.zoom_scale, params.zoom_factor);

    // Benchmark
    println!("\n⚡ Benchmarking (100 runs)...");
    let mut total = std::time::Duration::ZERO;
    for _ in 0..100 {
        let start = Instant::now();
        let _ = optimizer.optimize(&metadata)?;
        total += start.elapsed();
    }
    println!("  Average: {:.2} ms", total.as_secs_f64() * 1000.0 / 100.0);
    println!("  FPS: {:.1f}", 1000.0 / (total.as_secs_f64() * 1000.0 / 100.0));

    println!("\n✅ Demo completed successfully!");
    Ok(())
}