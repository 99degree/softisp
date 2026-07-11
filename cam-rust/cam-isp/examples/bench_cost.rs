//! Benchmark: pipeline cost estimator.
//!
//! Measures how fast PipelineBuilder::cost() runs for different resolutions.
//!
//! Usage:
//!   cargo run --example bench_cost -p cam-isp --features mnn

use cam_isp::pipeline::PipelineBuilder;
use std::time::Instant;

fn main() {
    println!("Pipeline Cost Estimator Benchmark");
    println!("=================================\n");

    let resolutions = [
        ("HD", 1280, 720),
        ("FHD", 1920, 1080),
        ("4K", 3840, 2160),
    ];

    for (name, w, h) in &resolutions {
        let builder = PipelineBuilder::new(*w, *h)
            .unpack()
            .demosaic_binning()
            .gamma(2.2)
            .sharpen(0.5)
            .contrast(1.3)
            .display();

        // Warm up
        let _ = builder.cost();

        // Benchmark
        let start = Instant::now();
        let iterations = 10000;
        for _ in 0..iterations {
            let _ = builder.cost();
        }
        let elapsed = start.elapsed();

        let (flops, mem) = builder.cost();
        println!("{} ({}x{}):", name, w, h);
        println!("  FLOPs: {:.2} G", flops as f64 / 1e9);
        println!("  Memory: {:.2} MB", mem as f64 / 1e6);
        println!("  Cost time: {:.3} µs/iter ({} iterations)\n",
            elapsed.as_secs_f64() * 1e6 / iterations as f64, iterations);
    }

    // Compare pipeline variants
    println!("Pipeline Variant Comparison (FHD):");
    println!("==================================\n");

    let variants = [
        ("Minimal", PipelineBuilder::new(1920, 1080).unpack().display()),
        ("Basic", PipelineBuilder::new(1920, 1080).unpack().demosaic_binning().display()),
        ("Full", PipelineBuilder::new(1920, 1080)
            .unpack().demosaic_binning().gamma(2.2).sharpen(0.5).contrast(1.3).display()),
    ];

    for (name, builder) in &variants {
        let (flops, mem) = builder.cost();
        println!("  {}: {:.2} G FLOPs, {:.2} MB", name, flops as f64 / 1e9, mem as f64 / 1e6);
    }
}
