//! Camera2-style demo app using the Camera HAL binder service.
//!
//! This simulates an Android app using Camera2 API, but directly through
//! the Rust Camera HAL service (no Android framework needed).
//!
//! Flow (matches Camera2 API pattern):
//! 1. Enumerate cameras via ICameraProvider.getCameraIdList()
//! 2. Open camera via ICameraDevice.open(callback)
//! 3. Configure streams via ICameraDeviceSession.configureStreams()
//! 4. Submit capture requests via ICameraDeviceSession.processCaptureRequest()
//! 5. Receive frames via ICameraDeviceCallback.onCaptureResult()
//! 6. Save frames as raw RGBA or convert to PNG
//!
//! Usage:
//!   cargo run --bin cam-demo -- --width 640 --height 480 --frames 5 --out ./frames

use std::sync::{Arc, Mutex};
use std::path::PathBuf;
use std::fs;

use clap::Parser;
use log::{info, error};

use cam_binder::{
    CameraHalService,
    types::*,
    callback::{IFrameCallback, ICameraDeviceCallback},
};

#[derive(Parser, Debug)]
#[clap(about = "Camera2-style demo app")]
struct Args {
    /// Camera ID to use
    #[clap(long, default_value = "0")]
    camera: String,

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
    #[clap(long, default_value = "./frames")]
    out: String,

    /// Save as raw RGBA (no PNG conversion)
    #[clap(long)]
    raw: bool,
}

/// Frame collector callback.
struct FrameCollector {
    frames: Mutex<Vec<StreamBuffer>>,
    out_dir: PathBuf,
    save_raw: bool,
}

impl FrameCollector {
    fn new(out_dir: PathBuf, save_raw: bool) -> Self {
        fs::create_dir_all(&out_dir).ok();
        Self {
            frames: Mutex::new(Vec::new()),
            out_dir,
            save_raw,
        }
    }

    fn save_frame(&self, buffer: &StreamBuffer, index: usize) {
        let filename = if self.save_raw {
            format!("frame_{:04}.rgba", index)
        } else {
            format!("frame_{:04}.raw", index)
        };
        let path = self.out_dir.join(&filename);

        match fs::write(&path, &buffer.data) {
            Ok(_) => info!("Saved frame {} → {} ({} bytes)", index, path.display(), buffer.data.len()),
            Err(e) => error!("Failed to save frame {}: {}", index, e),
        }
    }
}

impl IFrameCallback for FrameCollector {
    fn on_frame(&self, buffer: StreamBuffer) {
        let mut frames = self.frames.lock().unwrap();
        let index = frames.len();
        info!(
            "Frame {}: {}x{} ({} bytes, status={})",
            index, buffer.width, buffer.height, buffer.data.len(), buffer.status
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

    info!("═══ Camera2-Style Demo App ═══");
    info!("Camera: {}, Resolution: {}x{}, Frames: {}",
        args.camera, args.width, args.height, args.frames);
    info!("Output: {}", args.out);

    // ── Step 1: Create Camera HAL Service ──
    let service = CameraHalService::new();

    // ── Step 2: Enumerate cameras ──
    let camera_ids = service.get_camera_id_list();
    info!("Available cameras: {:?}", camera_ids);

    if !camera_ids.contains(&args.camera) {
        error!("Camera {} not found! Available: {:?}", args.camera, camera_ids);
        return;
    }

    // ── Step 3: Print camera info ──
    if let Some(info) = service.provider().get_camera_info(&args.camera) {
        info!("Camera info: facing={}, orientation={}, max_res={:?}",
            info.facing, info.orientation, info.max_resolution);
    }

    // ── Step 4: Open camera ──
    let callback = Arc::new(FrameCollector::new(
        PathBuf::from(&args.out),
        args.raw,
    ));

    let session = match service.open_camera(&args.camera, callback.clone()) {
        Ok(s) => s,
        Err(e) => {
            error!("Failed to open camera: {}", e);
            return;
        }
    };

    // ── Step 5: Configure streams ──
    let stream_config = StreamConfig::new(0, args.width, args.height, 0x1);
    let stream_ids = session.lock().unwrap().configure_streams(&[stream_config]);
    info!("Configured streams: {:?}", stream_ids);

    // ── Step 6: Process capture requests ──
    for i in 0..args.frames {
        let request = CaptureRequest::preview(i as i64, 0);
        let buffers = session.lock().unwrap().process_capture_request(&request);
        info!("Frame {}/{}: {} buffers", i + 1, args.frames, buffers.len());
    }

    // ── Step 7: Flush and close ──
    session.lock().unwrap().flush();
    session.lock().unwrap().close();
    service.close_camera(&args.camera);

    // ── Summary ──
    let collected = callback.frames.lock().unwrap();
    info!("═══ Capture complete: {} frames saved to {} ═══", collected.len(), args.out);

    // Print a simple hex dump of first frame's first 32 bytes
    if let Some(first) = collected.first() {
        let hex: String = first.data.iter().take(32)
            .map(|b| format!("{:02x}", b))
            .collect::<Vec<_>>()
            .join(" ");
        info!("First frame hex (32 bytes): {}", hex);
    }
}
