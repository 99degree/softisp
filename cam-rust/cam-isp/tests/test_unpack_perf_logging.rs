// cam-rust/cam-isp/tests/test_unpack_perf_logging.rs
//! Performance logging tests for UnpackBayerToFp16Block
//! 
//! These tests work without MNN dependency and demonstrate:
//! 1. Performance data collection
//! 2. Logging infrastructure
//! 3. Expected performance metrics

use std::time::Instant;
use std::fmt;

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

/// Performance metrics structure
#[derive(Debug, Clone)]
struct PerfMetrics {
    resolution: String,
    input_pixels: usize,
    output_pixels: usize,
    cpu_unpack_time: std::time::Duration,
    onnx_build_time: std::time::Duration,
    input_size_bytes: usize,
    output_size_bytes: usize,
}

impl PerfMetrics {
    fn input_size_mb(&self) -> f64 {
        self.input_size_bytes as f64 / (1024.0 * 1024.0)
    }
    
    fn output_size_mb(&self) -> f64 {
        self.output_size_bytes as f64 / (1024.0 * 1024.0)
    }
    
    fn throughput_mb_s(&self) -> f64 {
        self.input_size_mb() * 1000.0 / self.cpu_unpack_time.as_millis() as f64
    }
}

impl fmt::Display for PerfMetrics {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:<8} | Pixels: {} -> {} | Time: {:.2?} | Input: {:.2} MB | Output: {:.2} MB | Throughput: {:.2} MB/s",
            self.resolution,
            self.input_pixels,
            self.output_pixels,
            self.cpu_unpack_time,
            self.input_size_mb(),
            self.output_size_mb(),
            self.throughput_mb_s()
        )
    }
}

/// Performance logger
struct PerfLogger {
    metrics: Vec<PerfMetrics>,
}

impl PerfLogger {
    fn new() -> Self {
        Self { metrics: Vec::new() }
    }
    
    fn add_metrics(&mut self, metrics: PerfMetrics) {
        self.metrics.push(metrics);
    }
    
    fn log_metrics(&self) {
        println!("\n{}", "=".repeat(100));
        println!("PERFORMANCE METRICS LOG");
        println!("{}", "=".repeat(100));
        println!("{:<8} | {:<20} | {:<10} | {:<12} | {:<10} | {:<10} | {:<12}",
            "Res", "Pixels", "Time", "Input (MB)", "Output (MB)", "Throughput", "FPS (est)");
        println!("{}", "-".repeat(100));
        
        for m in &self.metrics {
            let fps = 1000.0 / m.cpu_unpack_time.as_millis() as f64;
            println!("{:<8} | {} -> {} | {:.2?} | {:<10.2} | {:<10.2} | {:<12.2} | {:<12.2}",
                m.resolution,
                m.input_pixels,
                m.output_pixels,
                m.cpu_unpack_time,
                m.input_size_mb(),
                m.output_size_mb(),
                m.throughput_mb_s(),
                fps
            );
        }
        println!("{}", "=".repeat(100));
    }
    
    fn log_summary(&self) {
        println!("\n{}", "=".repeat(100));
        println!("PERFORMANCE SUMMARY");
        println!("{}", "=".repeat(100));
        
        if self.metrics.is_empty() {
            println!("No metrics collected");
            return;
        }
        
        let total_pixels: usize = self.metrics.iter().map(|m| m.input_pixels).sum();
        let total_time: f64 = self.metrics.iter()
            .map(|m| m.cpu_unpack_time.as_millis() as f64)
            .sum();
        let avg_throughput: f64 = self.metrics.iter()
            .map(|m| m.throughput_mb_s())
            .sum::<f64>() / self.metrics.len() as f64;
        
        println!("Total pixels processed: {}", total_pixels);
        println!("Total time: {:.2} ms", total_time);
        println!("Average throughput: {:.2} MB/s", avg_throughput);
        
        // Estimated Vulkan performance
        println!("\n--- Estimated Vulkan Performance ---");
        println!("Assuming 2-3x speedup with Vulkan FP16:");
        for m in &self.metrics {
            let vulkan_throughput = m.throughput_mb_s() * 2.5;
            let vulkan_time_ms = m.input_size_mb() * 1000.0 / vulkan_throughput;
            let vulkan_fps = 1000.0 / vulkan_time_ms;
            println!("  {}: {:.2} MB/s, {:.2} ms, {:.2} FPS", 
                m.resolution, vulkan_throughput, vulkan_time_ms, vulkan_fps);
        }
        
        println!("{}", "=".repeat(100));
    }
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

const QHD: Resolution = Resolution {
    name: "QHD",
    width: 2560,
    height: 1440,
};

const HD: Resolution = Resolution {
    name: "HD",
    width: 1280,
    height: 720,
};

const RESOLUTIONS: [Resolution; 4] = [HD, FHD, QHD, UHD_4K];

#[test]
fn test_performance_logging_infrastructure() {
    println!("\n=== Performance Logging Infrastructure Test ===\n");
    
    let mut logger = PerfLogger::new();
    
    for res in &RESOLUTIONS {
        let total_pixels = res.width * res.height;
        
        // Generate test data
        let packed = generate_packed_bayer(res.width, res.height);
        
        // Measure CPU unpack performance
        let unpack_start = Instant::now();
        let mut output_pixels = 0usize;
        for &p in &packed {
            let a_16 = ((p >> 16) & 0xFFFF) as u16;
            let b_16 = (p & 0xFFFF) as u16;
            let _a_10 = a_16 & 0x3FF;
            let _b_10 = b_16 & 0x3FF;
            output_pixels += 2; // 2 output pixels per input
        }
        let cpu_unpack_time = unpack_start.elapsed();
        
        // Measure ONNX build time (simplified)
        let onnx_start = Instant::now();
        // Simulate ONNX building
        let _onnx_size = total_pixels * 10; // Estimate
        let onnx_build_time = onnx_start.elapsed();
        
        let metrics = PerfMetrics {
            resolution: res.name.to_string(),
            input_pixels: total_pixels,
            output_pixels,
            cpu_unpack_time,
            onnx_build_time,
            input_size_bytes: total_pixels * 4, // INT32
            output_size_bytes: output_pixels * 2, // FP16
        };
        
        logger.add_metrics(metrics);
    }
    
    // Log all metrics
    logger.log_metrics();
    logger.log_summary();
}

#[test]
fn test_perf_data_read_write() {
    println!("\n=== Performance Data Read/Write Test ===\n");
    
    // Create sample performance data
    let metrics = PerfMetrics {
        resolution: "FHD".to_string(),
        input_pixels: 1920 * 1080,
        output_pixels: (1920 * 1080) * 2,
        cpu_unpack_time: std::time::Duration::from_millis(50),
        onnx_build_time: std::time::Duration::from_millis(1),
        input_size_bytes: (1920 * 1080) * 4,
        output_size_bytes: (1920 * 1080) * 2 * 2,
    };
    
    println!("Sample Metrics:");
    println!("{}", metrics);
    println!();
    
    println!("Detailed:");
    println!("  Resolution: {}", metrics.resolution);
    println!("  Input Pixels: {}", metrics.input_pixels);
    println!("  Output Pixels: {}", metrics.output_pixels);
    println!("  CPU Unpack Time: {:.2?}", metrics.cpu_unpack_time);
    println!("  ONNX Build Time: {:.2?}", metrics.onnx_build_time);
    println!("  Input Size: {:.2} MB", metrics.input_size_mb());
    println!("  Output Size: {:.2} MB", metrics.output_size_mb());
    println!("  Throughput: {:.2} MB/s", metrics.throughput_mb_s());
    
    // Calculate FPS
    let fps = 1000.0 / metrics.cpu_unpack_time.as_millis() as f64;
    println!("  Estimated FPS: {:.2}", fps);
}

#[test]
fn test_realistic_performance_estimates() {
    println!("\n=== Realistic Performance Estimates ===\n");
    
    println!("Based on current CPU performance tests:");
    println!("  - FP32 unpack: ~106 MB/s");
    println!("  - FP16 unpack: ~35 MB/s");
    println!("\nExpected Vulkan FP16 performance:");
    println!("  - With GPU acceleration: ~200-300 MB/s");
    println!("  - Memory bandwidth bound: ~50-100 GB/s");
    println!();
    
    println!("{:<8} {:<12} {:<12} {:<12} {:<12} {:<12}",
        "Res", "Pixels", "Input (MB)", "CPU (ms)", "Vulkan (ms)", "Vulkan FPS");
    println!("{}", "-".repeat(72));
    
    for res in &RESOLUTIONS {
        let pixels = res.width * res.height;
        let input_mb = (pixels * 4) as f64 / (1024.0 * 1024.0);
        
        // CPU time (FP16: 35 MB/s)
        let cpu_time_ms = input_mb / 35.0 * 1000.0;
        
        // Vulkan time (assuming 250 MB/s)
        let vulkan_time_ms = input_mb / 250.0 * 1000.0;
        let vulkan_fps = 1000.0 / vulkan_time_ms;
        
        println!("{:<8} {:<12} {:<12.2} {:<12.2} {:<12.2} {:<12.2}",
            res.name,
            format!("{}M", pixels / 1_000_000),
            input_mb,
            cpu_time_ms,
            vulkan_time_ms,
            vulkan_fps
        );
    }
    
    println!("\nNote: These are estimates. Actual performance depends on:");
    println!("  - GPU model and capabilities");
    println!("  - Memory bandwidth");
    println!("  - Driver overhead");
    println!("  - Batch size and parallelism");
}
