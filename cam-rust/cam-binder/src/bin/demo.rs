//! Camera2-style demo app using the Camera HAL binder service.
//!
//! Flow:
//! 1. Enumerate cameras via ICameraProvider.getCameraIdList()
//! 2. Open camera via ICameraDevice.open(callback)
//! 3. Optionally set V4L2 device for real frames
//! 4. Configure streams via ICameraDeviceSession.configureStreams()
//! 5. Submit capture requests via ICameraDeviceSession.processCaptureRequest()
//! 6. Receive frames via ICameraDeviceCallback.onCaptureResult()
//! 7. Save frames as PNG or raw RGBA
//!
//! Usage:
//!   # Test pattern frames
//!   cargo run --bin cam-demo -- --width 640 --height 480 --frames 5 --out ./frames
//!
//!   # Real V4L2 capture (Linux with camera)
//!   cargo run --features v4l2 --bin cam-demo -- --v4l2 /dev/video0 --width 640 --height 480 --frames 3 --out ./frames --png

use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use clap::Parser;
use log::{error, info};

use cam_binder::{
    callback::{ICameraDeviceCallback, IFrameCallback},
    types::*,
    CameraHalService,
};

#[derive(Parser, Debug)]
#[clap(about = "Camera2-style demo app")]
struct Args {
    /// Camera ID to use (for provider lookup)
    #[clap(long, default_value = "0")]
    camera: String,

    /// V4L2 device path (e.g., /dev/video0) — enables real camera capture
    #[clap(long)]
    v4l2: Option<String>,

    /// Frame width
    #[clap(long, default_value_t = 640)]
    width: i32,

    /// Frame height
    #[clap(long, default_value_t = 480)]
    height: i32,

    /// Number of frames to capture
    #[clap(long, default_value_t = 5)]
    frames: usize,

    /// Output directory or file prefix
    #[clap(long, default_value = "./frames")]
    out: String,

    /// Save as PNG (requires image crate)
    #[clap(long)]
    png: bool,

    /// Save as raw RGBA (default if --png not set)
    #[clap(long)]
    raw: bool,

    /// Verbose output
    #[clap(long, short)]
    verbose: bool,
}

/// Frame collector callback.
struct FrameCollector {
    frames: Mutex<Vec<StreamBuffer>>,
    out_dir: PathBuf,
    save_png: bool,
    save_raw: bool,
}

impl FrameCollector {
    fn new(out_dir: PathBuf, save_png: bool, save_raw: bool) -> Self {
        fs::create_dir_all(&out_dir).ok();
        Self {
            frames: Mutex::new(Vec::new()),
            out_dir,
            save_png,
            save_raw,
        }
    }

    fn save_frame(&self, buffer: &StreamBuffer, index: usize) {
        let w = buffer.width as u32;
        let h = buffer.height as u32;
        let data = &buffer.data;

        if self.save_png {
            let filename = format!("frame_{:04}.png", index);
            let path = self.out_dir.join(&filename);
            match image::RgbaImage::from_raw(w, h, data.to_vec()) {
                Some(img) => {
                    if let Err(e) = img.save(&path) {
                        error!("Failed to save PNG {}: {}", path.display(), e);
                    } else {
                        info!("Saved PNG {} ({}x{})", path.display(), w, h);
                    }
                }
                None => error!("Failed to create PNG image from raw data"),
            }
        }

        if self.save_raw || !self.save_png {
            let filename = format!("frame_{:04}.raw", index);
            let path = self.out_dir.join(&filename);
            match fs::write(&path, &buffer.data) {
                Ok(_) => info!("Saved raw {} ({} bytes)", path.display(), buffer.data.len()),
                Err(e) => error!("Failed to save raw {}: {}", path.display(), e),
            }
        }
    }
}

impl IFrameCallback for FrameCollector {
    fn on_frame(&self, buffer: StreamBuffer) {
        let mut frames = self.frames.lock().unwrap();
        let index = frames.len();
        info!(
            "Frame {}: {}x{} ({} bytes, status={})",
            index,
            buffer.width,
            buffer.height,
            buffer.data.len(),
            buffer.status
        );
        self.save_frame(&buffer, index);
        frames.push(buffer);
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

fn main() {
    env_logger::init();
    let args = Args::parse();

    let width = args.width;
    let height = args.height;
    let num_frames = args.frames;
    let save_png = args.png || !args.raw;
    let save_raw = args.raw || !args.png;

    info!("═══ Camera2-Style Demo App ═══");
    info!(
        "Camera: {}, Resolution: {}x{}, Frames: {}",
        args.camera, width, height, num_frames
    );
    if let Some(ref dev) = args.v4l2 {
        info!("V4L2 device: {}", dev);
    }
    info!("Output: {}", args.out);
    info!("Format: {}", if save_png { "PNG" } else { "raw RGBA" });

    // ── Step 1: Create Camera HAL Service ──
    let service = CameraHalService::new();

    // ── Step 2: Enumerate cameras ──
    let camera_ids = service.get_camera_id_list();
    info!("Available cameras: {:?}", camera_ids);

    if !camera_ids.contains(&args.camera) {
        error!(
            "Camera {} not found! Available: {:?}",
            args.camera, camera_ids
        );
        return;
    }

    // ── Step 3: Print camera info ──
    if let Some(info) = service.provider().get_camera_info(&args.camera) {
        info!(
            "Camera info: facing={}, orientation={}, max_res={:?}",
            info.facing, info.orientation, info.max_resolution
        );
    }

    // ── Step 4: Open camera ──
    let callback = Arc::new(FrameCollector::new(
        PathBuf::from(&args.out),
        save_png,
        save_raw,
    ));

    let session = match service.open_camera(&args.camera, callback.clone()) {
        Ok(s) => s,
        Err(e) => {
            error!("Failed to open camera: {}", e);
            return;
        }
    };

    // ── Step 5: Configure V4L2 (if requested) ──
    if let Some(dev_path) = &args.v4l2 {
        session.lock().unwrap().set_v4l2_device(dev_path);
        info!("V4L2 device {} configured for real frame capture", dev_path);
    }

    // ── Step 6: Configure streams ──
    let stream_config = StreamConfig::new(0, width, height, 0x1);
    let stream_ids = session.lock().unwrap().configure_streams(&[stream_config]);
    info!("Configured streams: {:?}", stream_ids);

    // ── Step 7: Process capture requests ──
    for i in 0..num_frames {
        let request = CaptureRequest::preview(i as i64, 0);
        let buffers = session.lock().unwrap().process_capture_request(&request);
        info!("Frame {}/{}: {} buffers", i + 1, num_frames, buffers.len());
    }

    // ── Step 8: Flush and close ──
    session.lock().unwrap().flush();
    session.lock().unwrap().close();
    service.close_camera(&args.camera);

    // ── Summary ──
    let collected = callback.frames.lock().unwrap();
    info!(
        "═══ Capture complete: {} frames saved to {} ═══",
        collected.len(),
        args.out
    );

    // Print hex dump of first frame's first 32 bytes
    if let Some(first) = collected.first() {
        let hex: String = first
            .data
            .iter()
            .take(32)
            .map(|b| format!("{:02x}", b))
            .collect::<Vec<_>>()
            .join(" ");
        info!("First frame hex (32 bytes): {}", hex);
    }
}
