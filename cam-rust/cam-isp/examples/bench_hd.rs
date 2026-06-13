//! HD resolution benchmark with the comprehensive benchmark model.
//! Tests each MNN backend at 540p, 720p, 1080p.
//! Run:
//!   cd cam-rust
//!   RUST_LOG=info LD_LIBRARY_PATH=$PWD/lib/aarch64 \
//!     cargo run --example bench_hd -p cam-isp --features mnn

use std::time::Instant;
use cam_isp::engine::IspEngine;

fn main() {
    #[cfg(feature = "env_logger")]
    env_logger::init();
    #[cfg(not(feature = "env_logger"))]
    let _ = std::env::set_var("RUST_LOG", "info");

    // Reuse cam_isp::init() which registers our benchmark
    cam_isp::init();

    let resolutions: [(u32, u32, &str); 3] = [
        (640, 480, "480p"),
        (960, 540, "540p"),
        (1280, 720, "720p"),
    ];

    let backends = ["opencl", "opengl", "vulkan", "cpu"];

    for &(w, h, label) in &resolutions {
        // Build model at this resolution
        let mnn = match cam_isp::mnnengine::MnnEngine::build_bench_model(w, h) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("  {:4}x{:4} ({:5}): model build FAILED: {}", w, h, label, e);
                continue;
            }
        };

        for &be_name in &backends {
            let be = match be_name {
                "vulkan" => cam_isp::mnnengine::MnnBackend::Vulkan,
                "opencl" => cam_isp::mnnengine::MnnBackend::Opencl,
                "opengl" => cam_isp::mnnengine::MnnBackend::OpenGl,
                "cpu" => cam_isp::mnnengine::MnnBackend::Cpu,
                _ => unreachable!(),
            };

            let budget = std::time::Duration::from_millis(2000);
            let deadline = Instant::now() + budget;
            let (tx, rx) = std::sync::mpsc::channel();
            let mnn_owned = mnn.clone();

            std::thread::spawn(move || {
                let mut engine = cam_isp::mnnengine::MnnEngine::new(be);
                engine.set_model_path(&mnn_owned);
                let head: Box<dyn cam_isp::pipeline::IspBlock> =
                    Box::new(cam_isp::blocks::RawInputBlock::new());
                if engine.build(head, vec![], None, 16).is_err() {
                    let _ = tx.send(None);
                    return;
                }

                let params = cam_isp::engine::default_tone_params();
                let frame_size = w as usize * h as usize * 2;
                let mut buf = vec![0u8; frame_size];
                for y in 0..h {
                    for x in 0..w {
                        let off = (y * w + x) as usize * 2;
                        let val = (x ^ y) as u16;
                        buf[off] = val as u8;
                        buf[off + 1] = (val >> 8) as u8;
                    }
                }

                let mut count = 0u32;
                while Instant::now() < deadline {
                    if engine.process(w, h, w, &buf, 1024.0, w, None, &params,
                        None, None, 1.0, 0.0, None, None, None).is_ok() {
                        count += 1;
                    } else {
                        break;
                    }
                }
                let _ = tx.send(Some(count));
            });

            let count = match rx.recv_timeout(std::time::Duration::from_secs(3)) {
                Ok(Some(c)) => c,
                _ => { eprintln!("  {:4}x{:4} {:>8}: CRASHED/TIMEOUT", w, h, be_name); continue; }
            };

            let elapsed = budget.saturating_sub(deadline.saturating_duration_since(Instant::now()));
            let secs = elapsed.as_secs_f64().max(0.001);
            let fps = count as f64 / secs;
            let mpix_s = (w as f64 * h as f64 * fps) / 1_000_000.0;
            eprintln!("  {:4}x{:4} {:>8}: {:5.1} fps ({:.1} Mpix/s)", w, h, be_name, fps, mpix_s);
        }

        let _ = std::fs::remove_file(&mnn);
        eprintln!();
    }
}
