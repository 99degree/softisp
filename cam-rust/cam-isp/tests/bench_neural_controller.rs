//! Performance benchmark for neural controller ONNX models

use cam_isp::neural_controller::NeuralController;
use cam_isp::controller_api::ControllerApi;
use cam_isp::isp_params::IspParams;
use cam_isp::pipeline::IspFrame;
use cam_types::FrameFormat;
use std::time::Instant;

fn create_test_frame() -> IspFrame {
    IspFrame {
        width: 1920,
        height: 1080,
        data: vec![128; 1920 * 1080 * 2],
        format: FrameFormat::RawSensor,
        float_data: None,
        aux: None,
        params: IspParams::default(),
        timestamp_ns: 1000,
        prep_duration_ns: 0,
        inference_duration_ns: 0,
        total_duration_ns: 0,
    }
}

#[test]
fn bench_neural_controller_mock() {
    let mut ctrl = NeuralController::with_mock_model();
    let frame = create_test_frame();
    
    // Warmup
    for _ in 0..10 {
        let _ = ctrl.analyze_and_update(&frame);
    }
    
    // Benchmark
    let iterations = 100;
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = ctrl.analyze_and_update(&frame);
    }
    let elapsed = start.elapsed();
    
    let avg_us = elapsed.as_micros() as f64 / iterations as f64;
    let fps = 1_000_000.0 / avg_us;
    
    println!("\n=== Neural Controller Mock Model Benchmark ===");
    println!("Iterations: {}", iterations);
    println!("Total time: {:?}", elapsed);
    println!("Average: {:.2} µs/frame ({:.0} FPS)", avg_us, fps);
    println!("==============================================\n");
}

#[test]
fn bench_rule_based_controller() {
    let mut ctrl = cam_isp::controller_api::Controller::rule_based();
    let frame = create_test_frame();
    
    // Warmup
    for _ in 0..10 {
        let _ = ctrl.analyze_and_update(&frame);
    }
    
    // Benchmark
    let iterations = 100;
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = ctrl.analyze_and_update(&frame);
    }
    let elapsed = start.elapsed();
    
    let avg_us = elapsed.as_micros() as f64 / iterations as f64;
    let fps = 1_000_000.0 / avg_us;
    
    println!("\n=== Rule-Based Controller Benchmark ===");
    println!("Iterations: {}", iterations);
    println!("Total time: {:?}", elapsed);
    println!("Average: {:.2} µs/frame ({:.0} FPS)", avg_us, fps);
    println!("=======================================\n");
}

#[test]
fn bench_comparison() {
    let frame = create_test_frame();
    let iterations = 100;
    
    // Neural controller
    let mut neural = NeuralController::with_mock_model();
    for _ in 0..10 { let _ = neural.analyze_and_update(&frame); }
    let start = Instant::now();
    for _ in 0..iterations { let _ = neural.analyze_and_update(&frame); }
    let neural_time = start.elapsed();
    
    // Rule-based controller
    let mut rule_based = cam_isp::controller_api::Controller::rule_based();
    for _ in 0..10 { let _ = rule_based.analyze_and_update(&frame); }
    let start = Instant::now();
    for _ in 0..iterations { let _ = rule_based.analyze_and_update(&frame); }
    let rule_time = start.elapsed();
    
    let neural_us = neural_time.as_micros() as f64 / iterations as f64;
    let rule_us = rule_time.as_micros() as f64 / iterations as f64;
    let speedup = rule_us / neural_us;
    
    println!("\n=== Controller Comparison Benchmark ===");
    println!("Iterations: {}", iterations);
    println!("Neural:     {:.2} µs/frame ({:.0} FPS)", neural_us, 1_000_000.0 / neural_us);
    println!("Rule-Based: {:.2} µs/frame ({:.0} FPS)", rule_us, 1_000_000.0 / rule_us);
    println!("Speedup:    {:.2}x", speedup);
    println!("======================================\n");
}
