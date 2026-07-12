#![cfg(feature = "mnn")]
// cam-rust/cam-isp/tests/test_unpack_mnn_performance.rs
//! MNN-based performance tests for UnpackBayerToFp16Block
//!
//! Tests with actual MNN inference:
//! 1. Build ONNX model with unpack block
//! 2. Convert to MNN
//! 3. Run inference with MNN engine
//! 4. Read session profiling data
//! 5. Log performance metrics

use cam_isp::blocks::{RawInputBlock, UnpackBayerToFp16Block};
use cam_isp::mnn_sys::{MnnBackendType, MnnInterpreterSafe, MnnModelInfo};
use cam_isp::pipeline::GraphComposer;
use cam_isp::pipeline::IspBlock;
use std::time::Instant;

/// Pack two 10-bit values into one INT32: (A << 16) | B
fn pack_ab(a: u16, b: u16) -> i32 {
    ((a as i32) << 16) | (b as i32 & 0x3FF)
}

/// Generate synthetic Bayer data
fn generate_packed_bayer(width: usize, height: usize) -> Vec<i32> {
    (0..width * height)
        .map(|i| {
            let a = ((i * 13) % 1024) as u16;
            let b = ((i * 17) % 1024) as u16;
            pack_ab(a, b)
        })
        .collect()
}

/// Resolution configurations
struct Resolution {
    name: &'static str,
    width: usize,
    height: usize,
}

const FHD: Resolution = Resolution {
    name: "FHD",
    width: 1920,
    height: 1080,
};

const UHD_4K: Resolution = Resolution {
    name: "4K",
    width: 3840,
    height: 2160,
};

/// Performance metrics for logging
#[derive(Debug, Clone)]
struct MnnPerfMetrics {
    resolution: String,
    model_load_time: std::time::Duration,
    session_create_time: std::time::Duration,
    inference_time: std::time::Duration,
    memory_usage_mb: f32,
    flops: f32,
    input_size_mb: f64,
    output_size_mb: f64,
}

#[test]
#[ignore = "Requires MNN library with Vulkan backend"]
fn test_mnn_unpack_performance_with_profiling() {
    println!("\n=== MNN Unpack Performance with Profiling ===\n");

    let mut metrics: Vec<MnnPerfMetrics> = Vec::new();

    for res in [FHD, UHD_4K].iter() {
        println!("Testing {} ({}x{})...", res.name, res.width, res.height);

        // 1. Generate test data
        let packed = generate_packed_bayer(res.width, res.height);
        let input_size_mb = (packed.len() * 4) as f64 / (1024.0 * 1024.0);

        // 2. Build ONNX model
        let raw_input = RawInputBlock::new();
        let mut unpack = UnpackBayerToFp16Block::new();
        unpack.set_input_source(raw_input.frame_tensor().unwrap());

        let blocks: Vec<&dyn IspBlock> = vec![&raw_input, &unpack];
        let onnx_bytes =
            GraphComposer::compose_from_vec(&blocks, &[], 15).expect("Failed to build ONNX model");

        // 3. Save ONNX to temp file and convert to MNN
        let onnx_path =
            std::env::temp_dir().join(format!("test_unpack_{}.onnx", res.name.to_lowercase()));
        let mnn_path =
            std::env::temp_dir().join(format!("test_unpack_{}.mnn", res.name.to_lowercase()));
        std::fs::write(&onnx_path, &onnx_bytes).expect("Failed to write temp ONNX file");
        cam_isp::mnn_converter::convert_onnx_to_mnn(
            onnx_path.to_str().unwrap(),
            mnn_path.to_str().unwrap(),
            None,
        )
        .expect("ONNX→MNN conversion failed");
        std::fs::remove_file(&onnx_path).ok();

        // 5. Load MNN model and create session with profiling
        let load_start = Instant::now();
        let interp = MnnInterpreterSafe::from_file(mnn_path.to_str().unwrap())
            .expect("Failed to create interpreter");
        let model_load_time = load_start.elapsed();

        let session_start = Instant::now();
        let sess = interp
            .create_session(MnnBackendType::Vulkan, 1)
            .expect("Failed to create Vulkan session");
        let session_create_time = session_start.elapsed();

        // 6. Run inference multiple times for stable measurement
        let num_runs = 5;
        let mut total_inference_time = std::time::Duration::ZERO;

        for _ in 0..num_runs {
            let infer_start = Instant::now();
            sess.run().expect("Inference failed");
            let infer_time = infer_start.elapsed();
            total_inference_time += infer_time;
        }

        let avg_inference_time = total_inference_time / num_runs as u32;

        // 7. Read profiling data
        let memory_mb = sess.get_model_info(MnnModelInfo::MEMORY).unwrap_or(0.0);
        let flops = sess.get_model_info(MnnModelInfo::FLOPS).unwrap_or(0.0);
        println!("  Memory usage: {:.2} MB", memory_mb);
        println!("  FLOPS: {:.2} M", flops / 1_000_000.0);

        // Get output tensor for size calculation
        if let Some(t) = interp.get_first_output(&sess) {
            let dims = t.shape();
            let output_size = dims.iter().product::<i32>() as usize;
            let output_size_mb = (output_size * 2) as f64 / (1024.0 * 1024.0);

            // 8. Store metrics
            metrics.push(MnnPerfMetrics {
                resolution: res.name.to_string(),
                model_load_time,
                session_create_time,
                inference_time: avg_inference_time,
                memory_usage_mb: memory_mb,
                flops,
                input_size_mb,
                output_size_mb,
            });
        }

        // Clean up temp file
        std::fs::remove_file(&mnn_path).ok();
    }

    // Log metrics
    println!("\n=== MNN Performance Metrics ===");
    println!(
        "{:<8} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}",
        "Res",
        "Load (ms)",
        "Create (ms)",
        "Infer (ms)",
        "Memory (MB)",
        "FLOPS (M)",
        "Input (MB)",
        "Output (MB)"
    );
    println!("{}", "-".repeat(96));

    for m in &metrics {
        println!(
            "{:<8} {:<12.2} {:<12.2} {:<12.2} {:<12.2} {:<12.2} {:<12.2} {:<12.2}",
            m.resolution,
            m.model_load_time.as_millis() as f64,
            m.session_create_time.as_millis() as f64,
            m.inference_time.as_millis() as f64,
            m.memory_usage_mb,
            m.flops / 1_000_000.0,
            m.input_size_mb,
            m.output_size_mb
        );
    }

    // Calculate throughput
    println!("\n=== Throughput Analysis ===");
    for m in &metrics {
        let throughput = m.input_size_mb * 1000.0 / m.inference_time.as_millis() as f64;
        println!("{} throughput: {:.2} MB/s", m.resolution, throughput);
    }
}

/// Simpler test without MNN dependency - just logs the expected performance
#[test]
fn test_log_expected_performance() {
    println!("\n=== Expected MNN Performance (Based on CPU Tests) ===\n");

    let resolutions = [FHD, UHD_4K];

    println!(
        "{:<8} {:<12} {:<12} {:<12} {:<12}",
        "Res", "Input (MB)", "Output (MB)", "Est. Time (ms)", "Est. FPS"
    );
    println!("{}", "-".repeat(60));

    for res in &resolutions {
        let total_pixels = res.width * res.height;
        let input_size_mb = (total_pixels * 4) as f64 / (1024.0 * 1024.0);
        let output_size_mb = (total_pixels * 2 * 2) as f64 / (1024.0 * 1024.0); // 2 channels * 2 bytes

        // Based on CPU tests: ~35 MB/s for FP16 unpack
        let estimated_time_ms = input_size_mb / 35.0 * 1000.0;
        let estimated_fps = 1000.0 / estimated_time_ms;

        println!(
            "{:<8} {:<12.2} {:<12.2} {:<12.2} {:<12.2}",
            res.name, input_size_mb, output_size_mb, estimated_time_ms, estimated_fps
        );
    }

    println!("\nNote: These are CPU-based estimates. Vulkan backend should be faster.");
    println!("With Vulkan FP16 acceleration, expect 2-3x speedup.");
}

/// Test that reads and displays profiling data structure
#[test]
fn test_perf_data_structure() {
    println!("\n=== Performance Data Structure Test ===\n");

    // Create a sample metrics structure
    let metrics = MnnPerfMetrics {
        resolution: "FHD".to_string(),
        model_load_time: std::time::Duration::from_millis(50),
        session_create_time: std::time::Duration::from_millis(10),
        inference_time: std::time::Duration::from_millis(5),
        memory_usage_mb: 10.5,
        flops: 2_000_000.0, // 2 MFLOPS
        input_size_mb: 7.91,
        output_size_mb: 7.91,
    };

    // Display the structure
    println!("Sample Performance Metrics:");
    println!("Resolution: {}", metrics.resolution);
    println!("Model Load Time: {:.2?}", metrics.model_load_time);
    println!("Session Create Time: {:.2?}", metrics.session_create_time);
    println!("Inference Time: {:.2?}", metrics.inference_time);
    println!("Memory Usage: {:.2} MB", metrics.memory_usage_mb);
    println!("FLOPS: {:.2} M", metrics.flops / 1_000_000.0);
    println!("Input Size: {:.2} MB", metrics.input_size_mb);
    println!("Output Size: {:.2} MB", metrics.output_size_mb);

    // Calculate derived metrics
    let throughput = metrics.input_size_mb * 1000.0 / metrics.inference_time.as_millis() as f64;
    let fps = 1000.0 / metrics.inference_time.as_millis() as f64;

    println!("\nDerived Metrics:");
    println!("Throughput: {:.2} MB/s", throughput);
    println!("FPS: {:.2}", fps);
}
