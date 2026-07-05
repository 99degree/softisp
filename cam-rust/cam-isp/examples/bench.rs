//! Pipeline benchmark — measures per-frame and per-component timing.
//!
//! Usage:
//!   cargo run --example bench -p cam-isp
//!   cargo run --example bench -p cam-isp -- --frames 100
//!   cargo run --example bench -p cam-isp -- --frames 50 --width 64 --height 48

use std::time::{Duration, Instant};
use cam_isp::engine::IspEngine;
use cam_isp::cpu::CpuEngine;
use cam_isp::blocks::RawInputBlock;

/// Generate a raw Bayer test pattern.
fn make_raw(width: u32, height: u32) -> Vec<u8> {
    let mut raw = Vec::with_capacity((width * height * 2) as usize);
    for y in 0..height {
        for x in 0..width {
            let val: u16 = if y % 2 == 0 {
                if x % 2 == 0 { 2000 } else { 4000 }
            } else {
                if x % 2 == 0 { 4000 } else { 6000 }
            };
            raw.extend_from_slice(&val.to_le_bytes());
        }
    }
    raw
}

fn run_bench(frames: u32, width: u32, height: u32) {
    cam_isp::init();

    let raw = make_raw(width, height);
    let mut engine = CpuEngine::new();
    let head = RawInputBlock::new();
    engine.build(Box::new(head), vec![], None, 21).unwrap();

    // Warmup: 3 frames
    for _ in 0..3 {
        let _ = engine.process(&cam_isp::engine::ProcessParams::new(width, height, &raw));
    }

    // Timed run
    let mut total = Duration::ZERO;
    let mut min = Duration::MAX;
    let mut max = Duration::ZERO;

    for i in 0..frames {
        let start = Instant::now();
        let result = engine.process(&cam_isp::engine::ProcessParams::new(width, height, &raw));
        let elapsed = start.elapsed();
        total += elapsed;
        if elapsed < min { min = elapsed; }
        if elapsed > max { max = elapsed; }

        if let Ok(frame) = result {
            // Show per-frame timing for first 3 and last 2
            if i < 3 || i >= frames - 2 {
                let mbps = (frame.data.len() as f64 / 1_000_000.0) / elapsed.as_secs_f64().max(1e-9);
                println!("  Frame {:3}: {:>8.1?}  {:>4}×{:<4}  {:>6.1?}  {:.1} MB/s",
                    i, elapsed, frame.width, frame.height,
                    frame.aux.as_ref().and_then(|a| a.cct).unwrap_or(0.0) as u32,
                    mbps);
            }
        }
    }

    let avg = total / frames;
    let fps = frames as f64 / total.as_secs_f64();
    let raw_mb = (width * height * 2) as f64 / 1_000_000.0;
    let throughput = raw_mb * fps;

    println!();
    println!("─── {}×{}  {} frames ───", width, height, frames);
    println!("  Min:    {:>8.1?}", min);
    println!("  Avg:    {:>8.1?}", avg);
    println!("  Max:    {:>8.1?}", max);
    println!("  Total:  {:>8.1?}", total);
    println!("  FPS:    {:>8.1}", fps);
    println!("  Throughput: {:.1} MB/s (raw), {:.1} MB/s (output)", throughput, throughput * 2.0);
}

fn main() {
    let frames: u32 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(10);
    let width: u32 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(32);
    let height: u32 = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(32);

    println!("CpuEngine Benchmark: {} frames of {}×{}", frames, width, height);
    println!("───");
    run_bench(frames, width, height);

    // Additional size benchmarks
    println!();
    println!("─── Size Scaling ───");
    for &(w, h) in &[(16, 16), (32, 32), (64, 48), (128, 96)] {
        let raw = make_raw(w, h);
        let mut engine = CpuEngine::new();
        let head = RawInputBlock::new();
        engine.build(Box::new(head), vec![], None, 21).unwrap();

        // Warmup
        for _ in 0..2 {
            let _ = engine.process(&cam_isp::engine::ProcessParams::new(w, h, &raw));
        }

        let start = Instant::now();
        let n = 20 / (w * h).max(1);
        let n = n.clamp(5, 50);
        for _ in 0..n {
            let _ = engine.process(&cam_isp::engine::ProcessParams::new(w, h, &raw));
        }
        let elapsed = start.elapsed();
        let avg = elapsed / n;
        let fps = n as f64 / elapsed.as_secs_f64();
        println!("  {:>3}×{:<3}  {:>8.1?} avg  {:>6.1} fps", w, h, avg, fps);
    }
}
