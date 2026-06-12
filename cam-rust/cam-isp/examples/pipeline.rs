//! ISP Pipeline Demo — demonstrates the CpuEngine processing a synthetic RAW frame
//! through the full 9-block ISP pipeline and saving the result as a PNG.
//!
//! Usage:
//!   cargo run --example pipeline -- cam-rust/cam-isp/examples
//!   ./out.png  (output file)
//!
//! Run:
//!   cargo run --example pipeline -- --out /tmp/isp_output.png

use std::path::PathBuf;
use clap::Parser;

use cam_isp::cpu::CpuEngine;
use cam_isp::engine::IspEngine;

#[derive(Parser, Debug)]
struct Args {
    /// Output PNG file path
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

    /// Black level (per-channel, comma-separated)
    #[clap(long, default_value = "64,64,64,64")]
    black_level: String,
}

fn generate_raw_bayer_frame(width: u32, height: u32) -> Vec<u8> {
    let mut raw = Vec::with_capacity((width * height * 2) as usize);

    for y in 0..height {
        for x in 0..width {
            // Simulate a scene: blue sky (top), green grass (bottom), and a red object in center
            let normalized_x = x as f32 / width as f32;
            let normalized_y = y as f32 / height as f32;

            // Sky gradient (blue at top y<0.3)
            let (r, g, b): (u16, u16, u16) = if normalized_y < 0.3 {
                // Blue sky, some green/red from clouds
                let blue = (0.4 + 0.3 * (1.0 - normalized_x)) * 65535.0;
                let green = (0.2 + 0.1 * normalized_x.sin()) * 65535.0;
                let red = (0.15 + 0.05 * normalized_x) * 65535.0;
                (red as u16, green as u16, blue as u16)
            } else if normalized_y < 0.7 {
                // Transition zone: green grass with red flower in center
                if (normalized_x - 0.5).abs() < 0.1 && (normalized_y - 0.5).abs() < 0.15 {
                    // Red flower
                    (60000, 2000, 1000) // R, G, B
                } else {
                    // Green grass
                    (3000, 40000, 2000) // R, G, B
                }
            } else {
                // Brown/green ground
                let red = (0.25 + 0.05 * normalized_x) * 65535.0;
                let green = (0.35 - 0.1 * normalized_x) * 65535.0;
                let blue = (0.1 + 0.05 * normalized_x) * 65535.0;
                (red as u16, green as u16, blue as u16)
            };

            // Apply BGGR Bayer pattern
            let bayer_val: u16 = if y % 2 == 0 {
                if x % 2 == 0 { b } else { g } // B at even row, even col; G at even row, odd col
            } else {
                if x % 2 == 0 { g } else { r } // G at odd row, even col; R at odd row, odd col
            };

            // Add some sensor noise (±2%)
            // (For simplicity, we skip actual noise)
            raw.extend_from_slice(&bayer_val.to_le_bytes());
        }
    }
    raw
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    println!("ISP Pipeline Demo");
    println!("Generating {}x{} synthetic RAW frame...", args.width, args.height);

    // 1. Generate synthetic RAW Bayer data
    let raw_data = generate_raw_bayer_frame(args.width, args.height);

    // 2. Parse black level
    let blc_values: [f32; 4] = {
        let parts: Vec<f32> = args.black_level.split(',')
            .filter_map(|s| s.trim().parse().ok())
            .collect();
        if parts.len() == 4 {
            [parts[0], parts[1], parts[2], parts[3]]
        } else {
            [64.0, 64.0, 64.0, 64.0]
        }
    };

    // 3. Create CpuEngine and process
    let mut engine = CpuEngine::new();
    let head = cam_isp::blocks::RawInputBlock::new();
    engine.build(Box::new(head), vec![], None, 21)
        .expect("Build CpuEngine failed");

    let tone_params = cam_types::ToneParams {
        contrast: 1.2,
        brightness: 0.05,
        gamma_recip: 2.2,  // gamma = 1/2.2 ≈ 0.4545
        saturation: 1.3,
        ..Default::default()
    };

    println!("Running 9-block ISP pipeline on {} bytes...", raw_data.len());

    let start = std::time::Instant::now();
    let result = engine.process(
        args.width,
        args.height,
        args.width,
        &raw_data,
        args.sensor_max,
        args.width,  // target width (same as input)
        None,        // use default identity CCM
        &tone_params,
        Some(&[1.8, 1.0, 1.0, 2.0]),  // bayer gains (R, Gr, Gb, B)
        None,        // awb gains
        1.0,         // analog gain
        0.0,         // scene change
        None,        // lsc gains
        Some(&blc_values),
        None,        // warp grid
    );
    let elapsed = start.elapsed();

    match result {
        Ok(frame) => {
            println!("Pipeline complete in {:?}", elapsed);
            println!("Output: {}x{} RGBA ({} bytes)", frame.width, frame.height, frame.data.len());

            // 4. Save as PNG
            let img = match image::ImageBuffer::<image::Rgba<u8>, Vec<u8>>::from_raw(
                frame.width,
                frame.height,
                frame.data.clone(),
            ) {
                Some(img) => img,
                None => {
                    eprintln!("Failed to create image buffer from frame data");
                    return;
                }
            };

            if let Err(e) = img.save(&args.out) {
                eprintln!("Failed to save PNG to {}: {}", args.out.display(), e);
            } else {
                println!("Saved processed image to {}", args.out.display());
                println!("File size: {} bytes", std::fs::metadata(&args.out).map(|m| m.len()).unwrap_or(0));
            }
        }
        Err(e) => {
            eprintln!("Pipeline processing failed: {}", e);
        }
    }
}
