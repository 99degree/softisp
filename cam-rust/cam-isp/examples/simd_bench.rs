//! SIMD Backend Benchmark
//!
//! Measures speedup of SIMD-accelerated operations.
//! Usage: cargo run --release --example simd_bench -p cam-isp

use cam_isp::simd::{best_backend, SimdEngine, scalar::Scalar};
use std::time::Instant;

fn bench<F>(name: &str, mut f: F, iterations: usize)
where
    F: FnMut(),
{
    // Warmup
    for _ in 0..3 { f(); }

    let start = Instant::now();
    for _ in 0..iterations { f(); }
    let total = start.elapsed();
    let avg_secs = total.as_secs_f64() / iterations as f64;
    let avg = std::time::Duration::from_secs_f64(avg_secs);
    let fps = if avg_secs > 0.0 { 1.0 / avg_secs } else { 0.0 };

    println!("{:<25} {:>8.1?}  {:>8.1} ops/s", name, avg, fps);
}

fn main() {
    let backend = best_backend();
    println!("SIMD Backend: {}\n", backend.name());

    // 1. normalize_u16_to_f32
    let size = 1024 * 1024;
    let input: Vec<u16> = (0..size).map(|i| i as u16).collect();
    let mut out1 = vec![0.0f32; size];
    let mut out2 = vec![0.0f32; size];

    println!("--- normalize_u16_to_f32 ({} elements) ---", size);
    bench("scalar", || {
        Scalar.normalize_u16_to_f32(&input, &mut out1, 65535.0);
    }, 100);
    bench(backend.name(), || {
        backend.normalize_u16_to_f32(&input, &mut out2, 65535.0);
    }, 100);

    // 2. apply_ccm
    let rgb_size = size * 3;
    let rgb: Vec<f32> = (0..rgb_size).map(|i| (i % 1000) as f32 / 1000.0).collect();
    let ccm = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

    println!("\n--- apply_ccm ({} RGB pixels) ---", size);
    let mut ccm1 = vec![0.0f32; rgb_size];
    bench("scalar", || {
        ccm1 = Scalar.apply_ccm(&rgb, &ccm);
    }, 50);
    let mut ccm2 = vec![0.0f32; rgb_size];
    bench(backend.name(), || {
        ccm2 = backend.apply_ccm(&rgb, &ccm);
    }, 50);

    // 3. apply_ae_gain
    println!("\n--- apply_ae_gain ({} RGB pixels) ---", size);
    let gain = 1.2;
    bench("scalar", || {
        ccm1 = Scalar.apply_ae_gain(&rgb, gain);
    }, 50);
    bench(backend.name(), || {
        ccm2 = backend.apply_ae_gain(&rgb, gain);
    }, 50);

    // 4. display_output
    let w = 1024;
    let h = 1024;
    let rgb_data: Vec<f32> = (0..w * h * 3).map(|_| 0.5).collect();
    println!("\n--- display_output ({}×{} → BGRA) ---", w, h);
    bench("scalar", || {
        let _ = Scalar.display_output(&rgb_data, w, h, w);
    }, 10);
    bench(backend.name(), || {
        let _ = backend.display_output(&rgb_data, w, h, w);
    }, 10);

    println!();
}
