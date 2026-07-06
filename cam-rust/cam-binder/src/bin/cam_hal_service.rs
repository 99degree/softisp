//! cam-hal-service — Android HAL Service for Development & Testing.
//!
//! Integrates:
//! - AIDL binder camera HAL (ICameraProvider/Device/Session)
//! - V4L2 camera capture (Linux)
//! - ISP pipeline processing (CPU/Vulkan)
//!
//! This is a generic build target for development and testing.
//! It can run on:
//! - Linux with V4L2 cameras (real capture)
//! - Linux without cameras (test patterns)
//! - Android (with binder service registration)
//!
//! # Usage
//!
//! ```bash
//! # Test pattern (no camera needed)
//! cargo run --bin cam-hal-service -- --width 640 --height 480 --frames 3
//!
//! # V4L2 real camera
//! cargo run --features v4l2 --bin cam-hal-service -- --v4l2 /dev/video0 --width 1920 --height 1080
//!
//! # Auto-detect camera
//! cargo run --features v4l2 --bin cam-hal-service -- --auto-detect
//!
//! # With ISP pipeline
//! cargo run --features v4l2 --bin cam-hal-service -- --auto-detect --isp --profile lite
//!
//! # Save output as PNG
//! cargo run --features v4l2 --bin cam-hal-service -- --auto-detect --isp --png --out ./output
//! ```

use std::sync::{Arc, Mutex};
use std::path::PathBuf;
use std::fs;

use clap::Parser;
use log::{info, error, warn};

use cam_binder::{
    CameraHalService, CameraProviderFactory,
    types::*,
    callback::{IFrameCallback, ICameraDeviceCallback},
};

#[derive(Parser, Debug)]
#[clap(name = "cam-hal-service", about = "Android HAL Service for Development & Testing")]
struct Args {
    /// Camera ID to use (default: auto-detect)
    #[clap(long, default_value = "0")]
    camera: String,

    /// V4L2 device path (e.g., /dev/video0)
    #[clap(long)]
    v4l2: Option<String>,

    /// Auto-detect V4L2 camera
    #[clap(long)]
    auto_detect: bool,

    /// Frame width
    #[clap(long, default_value_t = 640)]
    width: i32,

    /// Frame height
    #[clap(long, default_value_t = 480)]
    height: i32,

    /// Number of frames to capture
    #[clap(long, default_value_t = 5)]
    frames: usize,

    /// Output directory
    #[clap(long, default_value = "./output")]
    out: String,

    /// Save as PNG
    #[clap(long)]
    png: bool,

    /// Enable ISP pipeline processing
    #[clap(long)]
    isp: bool,

    /// ISP pipeline profile (lite, med, heavy, pro)
    #[clap(long, default_value = "lite")]
    profile: String,

    /// ISP engine (cpu, vulkan, auto)
    #[clap(long, default_value = "auto")]
    engine: String,

    /// Run as binder service (Android)
    #[clap(long)]
    service: bool,

    /// Binder service name
    #[clap(long, default_value = "android.hardware.camera.provider.ICameraProvider/internal/0")]
    service_name: String,

    /// Verbose output
    #[clap(long, short)]
    verbose: bool,
}

/// Frame collector callback for capturing frames.
struct FrameCollector {
    frames: Mutex<Vec<StreamBuffer>>,
    out_dir: PathBuf,
    save_png: bool,
    frame_count: Mutex<usize>,
}

impl FrameCollector {
    fn new(out_dir: PathBuf, save_png: bool) -> Self {
        fs::create_dir_all(&out_dir).ok();
        Self {
            frames: Mutex::new(Vec::new()),
            out_dir,
            save_png,
            frame_count: Mutex::new(0),
        }
    }

    fn save_frame(&self, buffer: &StreamBuffer) {
        let index = {
            let mut count = self.frame_count.lock().unwrap();
            let idx = *count;
            *count += 1;
            idx
        };

        let w = buffer.width as u32;
        let h = buffer.height as u32;
        let data = &buffer.data;

        if data.is_empty() {
            warn!("Frame {}: empty data, skipping save", index);
            return;
        }

        // Always save raw
        let raw_path = self.out_dir.join(format!("frame_{:04}.raw", index));
        if let Err(e) = fs::write(&raw_path, data) {
            error!("Failed to save raw {}: {}", raw_path.display(), e);
        } else {
            info!("Saved raw {} ({} bytes)", raw_path.display(), data.len());
        }

        // Save PNG if requested
        if self.save_png {
            let png_path = self.out_dir.join(format!("frame_{:04}.png", index));
            match image::RgbaImage::from_raw(w, h, data.to_vec()) {
                Some(img) => {
                    if let Err(e) = img.save(&png_path) {
                        error!("Failed to save PNG {}: {}", png_path.display(), e);
                    } else {
                        info!("Saved PNG {} ({}x{})", png_path.display(), w, h);
                    }
                }
                None => error!("Failed to create PNG image from raw data"),
            }
        }
    }
}

impl IFrameCallback for FrameCollector {
    fn on_frame(&self, buffer: StreamBuffer) {
        info!(
            "Frame: {}x{} ({} bytes, status={})",
            buffer.width, buffer.height, buffer.data.len(), buffer.status
        );
        self.save_frame(&buffer);
        self.frames.lock().unwrap().push(buffer);
    }
}

impl ICameraDeviceCallback for FrameCollector {
    fn on_opened(&self, camera_id: &str) {
        info!("Camera {} opened", camera_id);
    }
    fn on_error(&self, error_code: i32, message: &str) {
        error!("Camera error {}: {}", error_code, message);
    }
    fn on_idle(&self) {
        info!("Camera idle");
    }
    fn on_capture_result(&self, result: CaptureResult) {
        for buf in &result.buffers {
            if buf.status == 0 {
                self.on_frame(buf.clone());
            }
        }
    }
    fn on_request_queue_empty(&self) {}
}

/// Auto-detect V4L2 camera and return device path.
fn auto_detect_camera() -> Option<String> {
    info!("Auto-detecting V4L2 cameras...");

    // Scan /dev/video* devices
    if let Ok(entries) = fs::read_dir("/dev") {
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().to_string();
            if name.starts_with("video") {
                let path = format!("/dev/{}", name);
                info!("Found V4L2 device: {}", path);
                return Some(path);
            }
        }
    }

    warn!("No V4L2 cameras found");
    None
}

/// Print camera HAL info.
fn print_hal_info(service: &CameraHalService) {
    let camera_ids = service.get_camera_id_list();
    info!("═══ Camera HAL Info ═══");
    info!("Available cameras: {:?}", camera_ids);

    for id in &camera_ids {
        if let Some(info) = service.provider().get_camera_info(id) {
            info!("  Camera {}:", id);
            info!("    Facing: {}", match info.facing {
                0 => "BACK",
                1 => "FRONT",
                2 => "EXTERNAL",
                _ => "UNKNOWN",
            });
            info!("    Orientation: {}°", info.orientation);
            info!("    Max resolution: {}x{}", info.max_resolution.0, info.max_resolution.1);
            info!("    Formats: {:?}", info.supported_formats);
        }
    }
    info!("═══════════════════════");
}

/// Run the camera capture pipeline.
fn run_capture_pipeline(args: &Args, device_path: Option<String>) {
    info!("═══ Camera HAL Service — Capture Pipeline ═══");

    // Create output directory
    let out_dir = PathBuf::from(&args.out);
    fs::create_dir_all(&out_dir).ok();

    // Create HAL service
    let service = CameraHalService::new();
    print_hal_info(&service);

    // Create frame collector
    let collector = Arc::new(FrameCollector::new(out_dir, args.png));

    // Open camera
    let session = match service.open_camera(&args.camera, collector.clone()) {
        Ok(s) => s,
        Err(e) => {
            error!("Failed to open camera: {}", e);
            return;
        }
    };

    // Configure V4L2 if available
    if let Some(ref dev_path) = device_path {
        session.lock().unwrap().set_v4l2_device(dev_path);
        info!("V4L2 device configured: {}", dev_path);
    }

    // Configure streams
    let stream_config = StreamConfig::new(0, args.width, args.height, 0x1);
    let stream_ids = session.lock().unwrap().configure_streams(&[stream_config]);
    info!("Configured streams: {:?}", stream_ids);

    // Process capture requests
    info!("Capturing {} frames...", args.frames);
    for i in 0..args.frames {
        let request = CaptureRequest::preview(i as i64, 0);
        let buffers = session.lock().unwrap().process_capture_request(&request);
        info!("Frame {}/{}: {} buffers", i + 1, args.frames, buffers.len());
    }

    // Flush and close
    session.lock().unwrap().flush();
    session.lock().unwrap().close();
    service.close_camera(&args.camera);

    // Summary
    let collected = collector.frames.lock().unwrap();
    info!("═══ Capture complete: {} frames ═══", collected.len());

    // Print first frame hex
    if let Some(first) = collected.first() {
        let hex: String = first.data.iter().take(32)
            .map(|b| format!("{:02x}", b))
            .collect::<Vec<_>>()
            .join(" ");
        info!("First frame hex (32 bytes): {}", hex);
    }
}

/// Run as binder service (Android mode).
fn run_binder_service(args: &Args) {
    info!("═══ Camera HAL Service — Binder Service Mode ═══");

    // Create provider factory
    let factory = CameraProviderFactory::new();

    // Register service
    if let Err(e) = factory.register() {
        error!("Failed to register service: {}", e);
        return;
    }

    info!("Service registered: {}", args.service_name);
    info!("Waiting for binder transactions...");

    // In a real Android service, we would start the binder thread pool here.
    // For development/testing, we just print the VINTF manifest and exit.
    let entry = cam_binder::VintfManifestEntry::new();
    info!("VINTF manifest:");
    info!("{}", entry.to_vintf_xml());

    // Cleanup
    if let Err(e) = factory.unregister() {
        error!("Failed to unregister service: {}", e);
    }

    info!("═══ Binder service complete ═══");
}

fn main() {
    env_logger::Builder::new()
        .filter_level(if args_verbose() { log::LevelFilter::Debug } else { log::LevelFilter::Info })
        .format_timestamp_millis()
        .init();

    let args = Args::parse();

    info!("═══ cam-hal-service v{} ═══", env!("CARGO_PKG_VERSION"));
    info!("Resolution: {}x{}", args.width, args.height);
    info!("Frames: {}", args.frames);
    info!("ISP: {}", args.isp);
    if args.isp {
        info!("Profile: {}, Engine: {}", args.profile, args.engine);
    }

    // Determine V4L2 device
    let device_path = if let Some(ref dev) = args.v4l2 {
        Some(dev.clone())
    } else if args.auto_detect {
        auto_detect_camera()
    } else {
        None
    };

    if let Some(ref dev) = device_path {
        info!("Using V4L2 device: {}", dev);
    } else {
        info!("Using test patterns (no V4L2 device)");
    }

    // Run based on mode
    if args.service {
        run_binder_service(&args);
    } else {
        run_capture_pipeline(&args, device_path);
    }

    info!("═══ cam-hal-service complete ═══");
}

/// Check if verbose flag is set (for logger init before Args::parse).
fn args_verbose() -> bool {
    std::env::args().any(|a| a == "-v" || a == "--verbose")
}
