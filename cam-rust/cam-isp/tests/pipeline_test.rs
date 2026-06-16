//! End-to-end pipeline validation tests.
//!
//! Two test categories:
//! - **Controller tests**: Use tiny 4×4 Bayer input. The CpuEngine runs its full
//!   16-stage pipeline but on a minimal frame so it completes fast. Validates
//!   AWB/AE/CCT/tone convergence across multiple frames.
//! - **Pipeline tests**: Use small 8×8 input. Verifies basic pixel-level properties
//!   (gradient direction, gray balance, non-empty output).

use cam_isp::cpu::CpuEngine;
use cam_isp::engine::IspEngine;
use cam_isp::profile::PipelineProfile;
use cam_isp::fused::FusedPipeline;

// ─── Raw data generators ───────────────────────────────────────────────

/// Generate a flat gray raw Bayer frame (INT16 LE) at a given intensity (0..1).
fn gray_raw(width: u32, height: u32, intensity: f32) -> Vec<u8> {
    let val = (intensity * 65535.0) as u16;
    let mut raw = Vec::with_capacity((width * height * 2) as usize);
    for _y in 0..height {
        for _x in 0..width {
            raw.extend_from_slice(&val.to_le_bytes());
        }
    }
    raw
}

/// Generate a gradient Bayer frame (left=dark, right=bright).
fn gradient_raw(width: u32, height: u32) -> Vec<u8> {
    let mut raw = Vec::with_capacity((width * height * 2) as usize);
    for _y in 0..height {
        for x in 0..width {
            let intensity = x as f32 / width as f32;
            let val = (intensity * 65535.0) as u16;
            raw.extend_from_slice(&val.to_le_bytes());
        }
    }
    raw
}

/// Generate a color Bayer frame (BGGR pattern).
fn color_raw(width: u32, height: u32, r: u16, g: u16, b: u16) -> Vec<u8> {
    let mut raw = Vec::with_capacity((width * height * 2) as usize);
    for y in 0..height {
        for x in 0..width {
            let val = if y % 2 == 0 {
                if x % 2 == 0 { b } else { g }
            } else {
                if x % 2 == 0 { g } else { r }
            };
            raw.extend_from_slice(&val.to_le_bytes());
        }
    }
    raw
}

// ─── Controller tests (tiny 4×4 input, fast) ─────────────────────────

/// Test that CpuEngine processes a single 4×4 gray frame without error.
#[test]
fn test_controller_gray_4x4() {
    cam_isp::init();
    let w = 4u32;
    let h = 4u32;
    let raw = gray_raw(w, h, 0.5);

    let mut engine = CpuEngine::new();
    engine.build(Box::new(cam_isp::blocks::RawInputBlock::new()), vec![], None, 21).unwrap();

    let result = engine.process(&cam_isp::engine::ProcessParams::new(w, h, &raw)).expect("Process failed");

    assert_eq!(result.width, w);
    assert_eq!(result.height, h);
    assert!(!result.data.is_empty(), "Output frame should not be empty");
}

/// Test AWB convergence across 10 frames with warm light input (4×4).
#[test]
fn test_controller_convergence() {
    cam_isp::init();
    let w = 4u32;
    let h = 4u32;
    let warm_raw = color_raw(w, h, 60000, 2000, 1000);

    let mut engine = CpuEngine::new();
    engine.build(Box::new(cam_isp::blocks::RawInputBlock::new()), vec![], None, 21).unwrap();

    for _ in 0..10 {
        let _result = engine.process(&cam_isp::engine::ProcessParams::new(w, h, &warm_raw)).expect("Process failed");
    }

    let ctrl = engine.controller.lock().unwrap();
    assert!(ctrl.frame_count >= 10, "Should have processed 10 frames, got {}", ctrl.frame_count);
    let changed = (ctrl.awb_gains[0] - 1.0).abs() > 0.01
        || (ctrl.awb_gains[2] - 1.0).abs() > 0.01;
    assert!(changed, "AWB should have adapted: gains={:.3} {:.3} {:.3}",
        ctrl.awb_gains[0], ctrl.awb_gains[1], ctrl.awb_gains[2]);
    assert!(ctrl.estimated_cct.is_some(), "CCT should be estimated");
}

// ─── Pipeline pixel-level tests (8×8 input) ───────────────────────────

/// Test that a flat gray input produces balanced RGB output.
#[test]
fn test_pipeline_gray_balance() {
    cam_isp::init();
    let w = 8u32;
    let h = 8u32;
    let raw = gray_raw(w, h, 0.5);

    let mut engine = CpuEngine::new();
    engine.build(Box::new(cam_isp::blocks::RawInputBlock::new()), vec![], None, 21).unwrap();

    let result = engine.process(&cam_isp::engine::ProcessParams::new(w, h, &raw)).expect("Process failed");

    assert_eq!(result.width, w);
    assert_eq!(result.height, h);
    assert!(!result.data.is_empty());

    // Gray input → R,G,B averages should be similar
    let pixel_count = (w * h) as u64;
    let mut totals = [0u64; 3];
    for y in 0..h {
        for x in 0..w {
            let idx = ((y * w + x) * 4) as usize;
            totals[0] += result.data[idx] as u64;
            totals[1] += result.data[idx + 1] as u64;
            totals[2] += result.data[idx + 2] as u64;
        }
    }
    let avgs: [u8; 3] = [
        (totals[0] / pixel_count) as u8,
        (totals[1] / pixel_count) as u8,
        (totals[2] / pixel_count) as u8,
    ];
    let max_diff = (avgs[0] as i16 - avgs[1] as i16).abs()
        .max((avgs[1] as i16 - avgs[2] as i16).abs())
        .max((avgs[0] as i16 - avgs[2] as i16).abs());
    assert!(max_diff < 80,
        "Gray balance: R={} G={} B={}, max_diff={}", avgs[0], avgs[1], avgs[2], max_diff);
}

/// Test that a gradient produces brighter right side than left.
#[test]
fn test_pipeline_gradient() {
    cam_isp::init();
    let w = 8u32;
    let h = 4u32;
    let raw = gradient_raw(w, h);

    let mut engine = CpuEngine::new();
    engine.build(Box::new(cam_isp::blocks::RawInputBlock::new()), vec![], None, 21).unwrap();

    let result = engine.process(&cam_isp::engine::ProcessParams::new(w, h, &raw)).expect("Process failed");

    let left_end = (w / 4) as usize;
    let right_start = (w * 3 / 4) as usize;
    let mut left_sum = 0u64;
    let mut right_sum = 0u64;
    let mut left_count = 0u64;
    let mut right_count = 0u64;
    for y in 0..h as usize {
        for x in 0..w as usize {
            let idx = (y * w as usize + x) * 4;
            let lum = result.data[idx] as u64 + result.data[idx + 1] as u64 + result.data[idx + 2] as u64;
            if x < left_end { left_sum += lum; left_count += 1; }
            else if x >= right_start { right_sum += lum; right_count += 1; }
        }
    }
    let left_avg = left_sum / left_count.max(1);
    let right_avg = right_sum / right_count.max(1);
    assert!(right_avg > left_avg,
        "Gradient: right {} should be > left {}", right_avg, left_avg);
}

/// Test pipeline handles edge cases (black/white frames) on tiny input.
#[test]
fn test_pipeline_edge_cases() {
    cam_isp::init();
    let w = 4u32;
    let h = 4u32;

    let mut engine = CpuEngine::new();
    engine.build(Box::new(cam_isp::blocks::RawInputBlock::new()), vec![], None, 21).unwrap();

    // Black frame
    let black = gray_raw(w, h, 0.0);
    let result = engine.process(&cam_isp::engine::ProcessParams::new(w, h, &black)).expect("Black frame failed");
    assert!(!result.data.is_empty());

    // White frame
    let white = gray_raw(w, h, 0.95);
    let result = engine.process(&cam_isp::engine::ProcessParams::new(w, h, &white)).expect("White frame failed");
    assert!(!result.data.is_empty());
    let max_val = result.data.iter().take((w * h * 4) as usize).max().copied().unwrap_or(0);
    assert!(max_val > 100, "White frame max pixel should be bright, got {}", max_val);
}

// ─── Profile / FusedPipeline build tests (no processing) ──────────────

/// Test FusedPipeline builds with LITE profile.
#[test]
fn test_profile_pipeline_integration() {
    cam_isp::init();
    let blocks = PipelineProfile::LITE.build_blocks(32, 0);
    let pipeline = FusedPipeline::build(blocks, 32);
    assert!(pipeline.is_ok());
}

/// Test FusedPipeline builds with TEST profile.
#[test]
fn test_profile_test_build() {
    cam_isp::init();
    let blocks = PipelineProfile::TEST.build_blocks(8, 2);
    let pipeline = FusedPipeline::build(blocks, 8);
    if let Err(ref e) = pipeline {
        eprintln!("FusedPipeline::build error: {}", e);
    }
    assert!(pipeline.is_ok(), "FusedPipeline::build failed");
}
