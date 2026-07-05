//! ISP Pipeline Demo — demonstrates the CpuEngine processing a synthetic RAW frame
//! through the full ISP pipeline and saving the result as a PNG.
//!
//! Supports multi-frame processing to show AWB/AE convergence over time.
//!
//! Run:
//!   cargo run --example pipeline -p cam-isp -- --frames 30 --verbose --out out.png

use std::path::PathBuf;
use clap::Parser;

use cam_isp::cpu::CpuEngine;
use cam_isp::engine::IspEngine;

#[derive(Parser, Debug)]
struct Args {
    /// Output PNG file path (last frame saved here)
    #[clap(long, short, default_value = "isp_output.png")]
    out: PathBuf,

    /// Image width
    #[clap(long, default_value_t = 128)]
    width: u32,

    /// Image height
    #[clap(long, default_value_t = 96)]
    height: u32,

    /// Sensor maximum value (e.g., 65535 for 16-bit, 4095 for 12-bit)
    #[clap(long, default_value_t = 65535.0)]
    sensor_max: f32,

    /// Number of frames to process (shows AWB/AE convergence)
    #[clap(long, short, default_value_t = 1)]
    frames: u32,

    /// Black level (per-channel, comma-separated)
    #[clap(long, default_value = "64,64,64,64")]
    black_level: String,

    /// Print per-frame convergence stats
    #[clap(long)]
    verbose: bool,
}

fn generate_raw_bayer_frame(width: u32, height: u32) -> Vec<u8> {
    let mut raw = Vec::with_capacity((width * height * 2) as usize);

    for y in 0..height {
        for x in 0..width {
            let normalized_x = x as f32 / width as f32;
            let normalized_y = y as f32 / height as f32;

            // Scene: blue sky (top), green grass (middle), brown ground (bottom)
            let (r, g, b): (u16, u16, u16) = if normalized_y < 0.3 {
                // Blue sky with slight gradient
                let blue = (0.4 + 0.3 * (1.0 - normalized_x)) * 65535.0;
                let green = (0.2 + 0.1 * normalized_x.sin()) * 65535.0;
                let red = (0.15 + 0.05 * normalized_x) * 65535.0;
                (red as u16, green as u16, blue as u16)
            } else if normalized_y < 0.7 {
                // Green grass with red flower in center
                if (normalized_x - 0.5).abs() < 0.1 && (normalized_y - 0.5).abs() < 0.15 {
                    (60000, 2000, 1000) // Red flower
                } else {
                    (3000, 40000, 2000) // Green grass
                }
            } else {
                // Brown ground
                let red = (0.25 + 0.05 * normalized_x) * 65535.0;
                let green = (0.35 - 0.1 * normalized_x) * 65535.0;
                let blue = (0.1 + 0.05 * normalized_x) * 65535.0;
                (red as u16, green as u16, blue as u16)
            };

            // BGGR Bayer pattern
            let bayer_val: u16 = if y % 2 == 0 {
                if x % 2 == 0 { b } else { g }
            } else {
                if x % 2 == 0 { g } else { r }
            };

            raw.extend_from_slice(&bayer_val.to_le_bytes());
        }
    }
    raw
}

/// Apply a warmth oscillation to the raw data to stress AWB tracking.
fn oscillate_raw(raw: &[u8], frame_idx: u32) -> Vec<u8> {
    let warmth = (frame_idx as f32 * 0.003).sin() * 0.15; // ±15% warmth
    let mut shifted = raw.to_vec();
    for chunk in shifted.chunks_exact_mut(2) {
        if chunk.len() == 2 {
            let val = u16::from_le_bytes([chunk[0], chunk[1]]);
            let adjusted = if frame_idx.is_multiple_of(2) {
                (val as f32 * (1.0 + warmth * 0.3)).min(65535.0) as u16
            } else {
                (val as f32 * (1.0 - warmth * 0.3)).min(65535.0) as u16
            };
            chunk.copy_from_slice(&adjusted.to_le_bytes());
        }
    }
    shifted
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    println!("ISP Pipeline Demo — {} frame(s), {}x{}", args.frames, args.width, args.height);

    // 1. Generate synthetic RAW Bayer data
    let raw_data = generate_raw_bayer_frame(args.width, args.height);
    println!("RAW data: {} bytes", raw_data.len());

    // 2. Parse black level
    let _blc_values: [f32; 4] = {
        let parts: Vec<f32> = args.black_level.split(',')
            .filter_map(|s| s.trim().parse().ok())
            .collect();
        if parts.len() == 4 {
            [parts[0], parts[1], parts[2], parts[3]]
        } else {
            [64.0, 64.0, 64.0, 64.0]
        }
    };

    // 3. Create CpuEngine
    let mut engine = CpuEngine::new();
    let head = cam_isp::blocks::RawInputBlock::new();
    engine.build(Box::new(head), vec![], None, 21)
        .expect("Build CpuEngine failed");

    let _tone_params = cam_types::ToneParams {
        contrast: 1.2,
        brightness: 0.05,
        gamma_recip: 2.2,
        saturation: 1.3,
        ..Default::default()
    };

    let mut total_elapsed = std::time::Duration::ZERO;
    let mut last_frame = None;

    for frame_idx in 0..args.frames {
        // Slightly vary scene to stress AWB tracking
        let frame_raw = if frame_idx == 0 {
            raw_data.clone()
        } else {
            oscillate_raw(&raw_data, frame_idx)
        };

        let start = std::time::Instant::now();
        let result = engine.process(&cam_isp::engine::ProcessParams::new(args.width, args.height, &frame_raw));
        let elapsed = start.elapsed();
        total_elapsed += elapsed;

        match result {
            Ok(frame) => {
                if args.verbose {
                    let cal = frame.aux.as_ref().and_then(|a| a.calibration_stats);
                    let ctrl = engine.controller.lock().unwrap();
                    let scene = cam_isp::scene::SceneCategory::classify(ctrl.avg_lum_mean, ctrl.estimated_cct.unwrap_or(5500) as u32);
                    print!("Frame {:3}: {:6.1?} | AWB [{:.3} {:.3} {:.3}] | CCT {:>4?} | AE {:.3}",
                        frame_idx, elapsed,
                        ctrl.awb_gains[0], ctrl.awb_gains[1], ctrl.awb_gains[2],
                        ctrl.estimated_cct.unwrap_or(0),
                        ctrl.get_effective_exposure_gain());
                    if let Some(cs) = cal {
                        print!(" | Cal: qm=[{:.4},{:.4},{:.4},{:.4}] noise={:.5} lum={:.4}",
                            cs[0], cs[1], cs[2], cs[3], cs[21], cs[20]);
                    }
                    let avg_lum = (ctrl.avg_r + ctrl.avg_g + ctrl.avg_b) / 3.0;
                    println!(" | {} | Luma: {:.4}", scene.name(), avg_lum);
                }
                last_frame = Some(frame);
            }
            Err(e) => eprintln!("Frame {} failed: {}", frame_idx, e),
        }
    }

    // Summary
    let avg_time = total_elapsed / args.frames.max(1);
    println!("───");
    println!("Processed {} frame(s) | Avg {:?}/frame | Total {:?}",
        args.frames, avg_time, total_elapsed);

    {
        let ctrl = engine.controller.lock().unwrap();
        let scene = cam_isp::scene::SceneCategory::classify(ctrl.avg_lum_mean, ctrl.estimated_cct.unwrap_or(5500) as u32);
        println!("Final | AWB: [{:.3} {:.3} {:.3}]  CCT: {:?}  AE: {:.3}  Scene: {}",
            ctrl.awb_gains[0], ctrl.awb_gains[1], ctrl.awb_gains[2],
            ctrl.estimated_cct, ctrl.get_effective_exposure_gain(),
            scene.name());
        println!("      | Means: R={:.4} G={:.4} B={:.4} (target R={:.1} G={:.1} B={:.1})",
            ctrl.avg_r, ctrl.avg_g, ctrl.avg_b,
            ctrl.target_r, ctrl.target_g, ctrl.target_b);
    }

    // 4. Save last frame as PNG
    if let Some(frame) = last_frame {
        println!("───");
        println!("Output: {}x{} RGBA ({} bytes)", frame.width, frame.height, frame.data.len());

        let img = match image::ImageBuffer::<image::Rgba<u8>, Vec<u8>>::from_raw(
            frame.width, frame.height, frame.data.clone(),
        ) {
            Some(img) => img,
            None => { eprintln!("Failed to create image"); return; }
        };

        if let Err(e) = img.save(&args.out) {
            eprintln!("Failed to save PNG: {}", e);
        } else {
            println!("Saved last frame to {}", args.out.display());
        }
    }
}
