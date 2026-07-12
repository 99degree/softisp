// cam-rust/cam-isp/tests/test_unpack_performance.rs
//! Performance and correctness tests for UnpackBayerToFp16Block
//!
//! Tests:
//! 1. Unpack performance for FHD (1920x1080) and 4K (3840x2160) resolutions
//! 2. FP16 vs FP32 output comparison
//! 3. Performance logging and profiling

use cam_isp::blocks::{RawInputBlock, UnpackBayerToFp16Block};
use cam_isp::pipeline::GraphComposer;
use cam_isp::pipeline::IspBlock;
use half::f16;
use std::time::Instant;

/// Pack two 10-bit values into one INT32: (A << 16) | B
fn pack_ab(a: u16, b: u16) -> i32 {
    ((a as i32) << 16) | (b as i32 & 0x3FF)
}

/// Generate synthetic Bayer data for a given resolution
fn generate_packed_bayer(width: usize, height: usize) -> Vec<i32> {
    (0..width * height)
        .map(|i| {
            // Simple pattern: A = (i * 13) % 1024, B = (i * 17) % 1024
            let a = ((i * 13) % 1024) as u16;
            let b = ((i * 17) % 1024) as u16;
            pack_ab(a, b)
        })
        .collect()
}

/// Unpack INT32 to FP16 (2 channels)
fn unpack_to_fp16(packed: &[i32], width: usize, height: usize) -> Vec<f16> {
    let total_pixels = width * height;
    let mut output = vec![f16::ZERO; total_pixels * 2]; // 2 channels

    for i in 0..total_pixels {
        let a_16 = ((packed[i] >> 16) & 0xFFFF) as u16;
        let b_16 = (packed[i] & 0xFFFF) as u16;

        let a_10 = a_16 & 0x3FF;
        let b_10 = b_16 & 0x3FF;

        output[i * 2] = f16::from_f32(a_10 as f32 / 1023.0);
        output[i * 2 + 1] = f16::from_f32(b_10 as f32 / 1023.0);
    }

    output
}

/// Unpack INT32 to FP32 (2 channels)
fn unpack_to_fp32(packed: &[i32], width: usize, height: usize) -> Vec<f32> {
    let total_pixels = width * height;
    let mut output = vec![0.0f32; total_pixels * 2]; // 2 channels

    for i in 0..total_pixels {
        let a_16 = ((packed[i] >> 16) & 0xFFFF) as u16;
        let b_16 = (packed[i] & 0xFFFF) as u16;

        let a_10 = a_16 & 0x3FF;
        let b_10 = b_16 & 0x3FF;

        output[i * 2] = a_10 as f32 / 1023.0;
        output[i * 2 + 1] = b_10 as f32 / 1023.0;
    }

    output
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

const RESOLUTIONS: [Resolution; 2] = [FHD, UHD_4K];

#[test]
fn test_unpack_performance_fhd_4k() {
    println!("\n=== Unpack Performance Test ===\n");

    for res in &RESOLUTIONS {
        let total_pixels = res.width * res.height;
        let input_size_mb = (total_pixels * 4) as f64 / (1024.0 * 1024.0); // INT32 input

        println!("Resolution: {} ({}x{})", res.name, res.width, res.height);
        println!("Input size: {:.2} MB", input_size_mb);

        // Generate test data
        let packed = generate_packed_bayer(res.width, res.height);

        // Test FP16 unpack
        let start_fp16 = Instant::now();
        let fp16_output = unpack_to_fp16(&packed, res.width, res.height);
        let fp16_time = start_fp16.elapsed();
        let fp16_output_size_mb = (fp16_output.len() * 2) as f64 / (1024.0 * 1024.0); // FP16 = 2 bytes

        // Test FP32 unpack
        let start_fp32 = Instant::now();
        let fp32_output = unpack_to_fp32(&packed, res.width, res.height);
        let fp32_time = start_fp32.elapsed();
        let fp32_output_size_mb = (fp32_output.len() * 4) as f64 / (1024.0 * 1024.0); // FP32 = 4 bytes

        println!(
            "FP16 unpack: {:.2?} ({:.2} MB output)",
            fp16_time, fp16_output_size_mb
        );
        println!(
            "FP32 unpack: {:.2?} ({:.2} MB output)",
            fp32_time, fp32_output_size_mb
        );

        // Calculate throughput
        let fp16_throughput = input_size_mb * 1000.0 / fp16_time.as_millis() as f64;
        let fp32_throughput = input_size_mb * 1000.0 / fp32_time.as_millis() as f64;

        println!("FP16 throughput: {:.2} MB/s", fp16_throughput);
        println!("FP32 throughput: {:.2} MB/s", fp32_throughput);
        println!();

        // Verify correctness
        for i in 0..total_pixels.min(100) {
            let fp16_a = fp16_output[i * 2].to_f32();
            let fp16_b = fp16_output[i * 2 + 1].to_f32();
            let fp32_a = fp32_output[i * 2];
            let fp32_b = fp32_output[i * 2 + 1];

            assert!(
                (fp16_a - fp32_a).abs() < 1e-3,
                "FP16 vs FP32 mismatch at pixel {} A: {} vs {}",
                i,
                fp16_a,
                fp32_a
            );
            assert!(
                (fp16_b - fp32_b).abs() < 1e-3,
                "FP16 vs FP32 mismatch at pixel {} B: {} vs {}",
                i,
                fp16_b,
                fp32_b
            );
        }
    }
}

#[test]
fn test_onnx_model_building_performance() {
    println!("\n=== ONNX Model Building Performance ===\n");

    for res in &RESOLUTIONS {
        println!("Resolution: {} ({}x{})", res.name, res.width, res.height);

        // Create blocks
        let raw_input = RawInputBlock::new();
        let mut unpack = UnpackBayerToFp16Block::new();
        unpack.set_input_source(raw_input.frame_tensor().unwrap());

        let blocks: Vec<&dyn IspBlock> = vec![&raw_input, &unpack];

        // Time ONNX model building
        let start = Instant::now();
        let onnx_bytes =
            GraphComposer::compose_from_vec(&blocks, &[], 15).expect("Failed to build ONNX model");
        let build_time = start.elapsed();

        println!("ONNX build time: {:.2?}", build_time);
        println!(
            "Model size: {} bytes ({:.2} KB)",
            onnx_bytes.len(),
            onnx_bytes.len() as f64 / 1024.0
        );
        println!();
    }
}

#[test]
fn test_fp16_fp32_memory_usage() {
    println!("\n=== Memory Usage Comparison ===\n");

    for res in &RESOLUTIONS {
        let total_pixels = res.width * res.height;

        println!("Resolution: {} ({}x{})", res.name, res.width, res.height);
        println!("Total pixels: {}", total_pixels);

        // Input: packed INT32
        let input_size_bytes = total_pixels * 4; // 4 bytes per INT32
        let input_size_mb = input_size_bytes as f64 / (1024.0 * 1024.0);

        // Output: FP16 (2 channels)
        let fp16_size_bytes = total_pixels * 2 * 2; // 2 channels * 2 bytes per FP16
        let fp16_size_mb = fp16_size_bytes as f64 / (1024.0 * 1024.0);

        // Output: FP32 (2 channels)
        let fp32_size_bytes = total_pixels * 2 * 4; // 2 channels * 4 bytes per FP32
        let fp32_size_mb = fp32_size_bytes as f64 / (1024.0 * 1024.0);

        println!("Input (INT32): {:.2} MB", input_size_mb);
        println!(
            "Output (FP16): {:.2} MB ({:.1}% of input)",
            fp16_size_mb,
            fp16_size_mb / input_size_mb * 100.0
        );
        println!(
            "Output (FP32): {:.2} MB ({:.1}% of input)",
            fp32_size_mb,
            fp32_size_mb / input_size_mb * 100.0
        );
        println!(
            "FP16 vs FP32 savings: {:.1}%",
            (1.0 - (fp16_size_mb / fp32_size_mb)) * 100.0
        );
        println!();
    }
}

#[test]
fn test_prepare_fp16_fp32_buffers() {
    println!("\n=== Buffer Preparation Test ===\n");

    for res in &RESOLUTIONS {
        println!("Preparing buffers for {}...", res.name);

        // Generate packed input
        let packed = generate_packed_bayer(res.width, res.height);

        // Prepare FP16 buffer
        let start_fp16 = Instant::now();
        let fp16_buffer = unpack_to_fp16(&packed, res.width, res.height);
        let fp16_time = start_fp16.elapsed();

        // Prepare FP32 buffer
        let start_fp32 = Instant::now();
        let fp32_buffer = unpack_to_fp32(&packed, res.width, res.height);
        let fp32_time = start_fp32.elapsed();

        println!(
            "FP16 buffer: {} elements, {:.2?} to prepare",
            fp16_buffer.len(),
            fp16_time
        );
        println!(
            "FP32 buffer: {} elements, {:.2?} to prepare",
            fp32_buffer.len(),
            fp32_time
        );

        // Verify first few values
        for i in 0..5 {
            let fp16_val = fp16_buffer[i].to_f32();
            let fp32_val = fp32_buffer[i];
            println!("  Value {}: FP16={:.6}, FP32={:.6}", i, fp16_val, fp32_val);
        }

        println!();
    }
}

/// Performance metrics storage for logging
#[derive(Debug, Clone)]
struct PerfMetrics {
    resolution: String,
    unpack_fp16_time: std::time::Duration,
    unpack_fp32_time: std::time::Duration,
    onnx_build_time: std::time::Duration,
    input_size_mb: f64,
    fp16_output_size_mb: f64,
    fp32_output_size_mb: f64,
}

#[test]
fn test_performance_logging() {
    println!("\n=== Performance Logging Test ===\n");

    let mut metrics: Vec<PerfMetrics> = Vec::new();

    for res in &RESOLUTIONS {
        let total_pixels = res.width * res.height;
        let input_size_mb = (total_pixels * 4) as f64 / (1024.0 * 1024.0);

        // Generate data
        let packed = generate_packed_bayer(res.width, res.height);

        // Measure FP16 unpack
        let start_fp16 = Instant::now();
        let fp16_output = unpack_to_fp16(&packed, res.width, res.height);
        let fp16_time = start_fp16.elapsed();
        let fp16_size_mb = (fp16_output.len() * 2) as f64 / (1024.0 * 1024.0);

        // Measure FP32 unpack
        let start_fp32 = Instant::now();
        let fp32_output = unpack_to_fp32(&packed, res.width, res.height);
        let fp32_time = start_fp32.elapsed();
        let fp32_size_mb = (fp32_output.len() * 4) as f64 / (1024.0 * 1024.0);

        // Measure ONNX building
        let raw_input = RawInputBlock::new();
        let mut unpack = UnpackBayerToFp16Block::new();
        unpack.set_input_source(raw_input.frame_tensor().unwrap());
        let blocks: Vec<&dyn IspBlock> = vec![&raw_input, &unpack];

        let start_onnx = Instant::now();
        let _ = GraphComposer::compose_from_vec(&blocks, &[], 15).unwrap();
        let onnx_time = start_onnx.elapsed();

        metrics.push(PerfMetrics {
            resolution: res.name.to_string(),
            unpack_fp16_time: fp16_time,
            unpack_fp32_time: fp32_time,
            onnx_build_time: onnx_time,
            input_size_mb,
            fp16_output_size_mb: fp16_size_mb,
            fp32_output_size_mb: fp32_size_mb,
        });
    }

    // Log metrics
    println!("Performance Metrics Summary:");
    println!(
        "{:<8} {:<12} {:<12} {:<12} {:<12} {:<12} {:<12}",
        "Res", "FP16 (ms)", "FP32 (ms)", "ONNX (ms)", "Input (MB)", "FP16 (MB)", "FP32 (MB)"
    );
    println!("{}", "-".repeat(80));

    for m in &metrics {
        println!(
            "{:<8} {:<12.2} {:<12.2} {:<12.2} {:<12.2} {:<12.2} {:<12.2}",
            m.resolution,
            m.unpack_fp16_time.as_millis() as f64,
            m.unpack_fp32_time.as_millis() as f64,
            m.onnx_build_time.as_millis() as f64,
            m.input_size_mb,
            m.fp16_output_size_mb,
            m.fp32_output_size_mb
        );
    }

    // Calculate speedups
    println!("\nSpeedup Analysis:");
    for m in &metrics {
        let fp16_fp32_ratio = m.unpack_fp32_time.as_secs_f64() / m.unpack_fp16_time.as_secs_f64();
        println!(
            "{} FP16 vs FP32: {:.2}x faster",
            m.resolution, fp16_fp32_ratio
        );
    }
}
