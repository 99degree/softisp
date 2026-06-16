//! Compare unpack+norm+cfa (separate) vs UnpackCfaBlock (fused).
//! Uses the same build/process pattern as bench_blocks.

use std::time::{Instant, Duration};
use cam_isp::pipeline::{IspBlock, GraphComposer};
use cam_isp::blocks::*;
use cam_isp::mnnengine::{MnnEngine, MnnBackend};
use cam_isp::engine::IspEngine;

fn bench_separate(h: u32, w_full: u32) -> (f64, String) {
    let w_packed = w_full / 2;
    let h_i64 = h as i64;
    let w_i64 = w_full as i64;
    let pw_i64 = w_packed as i64;
    let n_runs = 10;

    // Build blocks: raw → unpack → norm → cfa
    let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();
    blocks.push(Box::new(RawInputBlock::new()
        .with_elem_type(6).with_concrete_dims(h_i64, pw_i64)));
    blocks.push(Box::new(UnpackBlock::new().with_concrete_dims(h_i64, w_i64)));
    blocks.push(Box::new(NormalizeBlock::new()));
    blocks.push(Box::new(CfaBlock::new().with_concrete_dims(h_i64, w_i64)));

    GraphComposer::wire_blocks(&mut blocks);
    let mut all = blocks;
    let head = all.remove(0);
    let aux = all;

    let mut engine = MnnEngine::new(MnnBackend::Cpu);
    engine.build(head, aux, None, 21).unwrap();

    // Test buffer
    let mut buf = vec![0u8; (w_full * h * 2) as usize];
    for y in 0..h {
        for x in 0..w_full {
            let off = (y * w_full + x) as usize * 2;
            let val = (x ^ y) as u16;
            buf[off] = val as u8;
            buf[off + 1] = (val >> 8) as u8;
        }
    }

    let mut totals = Vec::new();

    for run in 0..n_runs {
        let result = engine.process(&cam_isp::engine::ProcessParams::new(w_full, h, &buf));
        match result {
            Ok(frame) => {
                let ms = frame.inference_duration_ns as f64 / 1_000_000.0;
                totals.push(ms);
                if run == 0 { println!("  separate first run: {:.1}ms", ms); }
            }
            Err(e) => eprintln!("  separate error: {}", e),
        }
    }

    let avg = if totals.len() > 1 {
        totals[1..].iter().sum::<f64>() / (totals.len() - 1) as f64
    } else { 0.0 };
    (avg, format!("{:.2}ms ({:.1}fps)", avg, 1000.0 / avg.max(0.01)))
}

fn bench_fused(h: u32, w_full: u32) -> (f64, String) {
    let w_packed = w_full / 2;
    let h_i64 = h as i64;
    let w_i64 = w_full as i64;
    let pw_i64 = w_packed as i64;
    let n_runs = 10;

    // Build blocks: raw → UnpackCfaBlock
    let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();
    blocks.push(Box::new(RawInputBlock::new()
        .with_elem_type(6).with_concrete_dims(h_i64, pw_i64)));
    blocks.push(Box::new(UnpackCfaBlock::new().with_concrete_dims(h_i64, w_i64)));

    GraphComposer::wire_blocks(&mut blocks);
    let mut all = blocks;
    let head = all.remove(0);
    let aux = all;

    let mut engine = MnnEngine::new(MnnBackend::Cpu);
    engine.build(head, aux, None, 21).unwrap();

    let mut buf = vec![0u8; (w_full * h * 2) as usize];
    for y in 0..h {
        for x in 0..w_full {
            let off = (y * w_full + x) as usize * 2;
            let val = (x ^ y) as u16;
            buf[off] = val as u8;
            buf[off + 1] = (val >> 8) as u8;
        }
    }

    let mut totals = Vec::new();

    for run in 0..n_runs {
        let result = engine.process(&cam_isp::engine::ProcessParams::new(w_full, h, &buf));
        match result {
            Ok(frame) => {
                let ms = frame.inference_duration_ns as f64 / 1_000_000.0;
                totals.push(ms);
                if run == 0 { println!("  fused first run: {:.1}ms", ms); }
            }
            Err(e) => eprintln!("  fused error: {}", e),
        }
    }

    let avg = if totals.len() > 1 {
        totals[1..].iter().sum::<f64>() / (totals.len() - 1) as f64
    } else { 0.0 };
    (avg, format!("{:.2}ms ({:.1}fps)", avg, 1000.0 / avg.max(0.01)))
}

fn main() {
    let _ = std::env::set_var("RUST_LOG", "warn");
    cam_isp::init();

    let args: Vec<String> = std::env::args().collect();
    let (w, h) = if args.len() > 2 {
        (args[1].parse().unwrap_or(640), args[2].parse().unwrap_or(480))
    } else {
        (640u32, 480u32)
    };

    println!("=== Unpack+Norm+CFA: Separate vs Fused ===");
    println!("Resolution: {}x{}", w, h);
    println!();

    let (sep_ms, sep_str) = bench_separate(h, w);
    let (fused_ms, fused_str) = bench_fused(h, w);

    println!();
    println!("=== Results ===");
    println!("  Separate (unpack→norm→cfa, 3 sessions): {}", sep_str);
    println!("  Fused    (UnpackCfaBlock, 1 session):    {}", fused_str);
    if sep_ms > 0.0 && fused_ms > 0.0 {
        let ratio = sep_ms / fused_ms;
        println!("  Ratio: {:.2}x", ratio);
        if ratio > 1.0 {
            println!("  ✅ Fused is {:.0}% faster!", (ratio - 1.0) * 100.0);
        } else {
            println!("  ❌ Separate is {:.0}% faster!", (1.0 / ratio - 1.0) * 100.0);
        }
    }
}
