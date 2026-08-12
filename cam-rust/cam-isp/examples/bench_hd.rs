//! HD resolution benchmark with the comprehensive benchmark model.
//! Tests each MNN backend at 540p, 720p, 1080p.
//! Run:
//!   cd cam-rust
//!   RUST_LOG=info LD_LIBRARY_PATH=$PWD/lib/aarch64 \
//!     cargo run --example bench_hd -p cam-isp --features mnn

use cam_isp::engine::IspEngine;
use std::time::Instant;

fn main() {
    let _ = env_logger::builder()
        .is_test(false)
        .filter_level(log::LevelFilter::Info)
        .try_init();

    // Reuse cam_isp::init() which registers our benchmark
    cam_isp::init();

    let resolutions: [(u32, u32, &str); 4] = [
        (640, 480, "480p"),
        (960, 540, "540p"),
        (1280, 720, "720p"),
        (1920, 1080, "1080p"),
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
                _ => unreachable!(),
            };

            let budget =
                std::time::Duration::from_millis(if w * h > 1280 * 720 { 8000 } else { 2000 });
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
                let mut total_prep_ns = 0u64;
                let mut total_infer_ns = 0u64;
                let mut total_total_ns = 0u64;
                while Instant::now() < deadline {
                    match engine.process(&cam_isp::engine::ProcessParams::new(w, h, &buf)) {
                        Ok(f) => {
                            total_prep_ns += f.prep_duration_ns;
                            total_infer_ns += f.inference_duration_ns;
                            total_total_ns += f.total_duration_ns;
                            count += 1;
                        }
                        Err(_) => break,
                    }
                }
                let _ = tx.send(Some((count, total_prep_ns, total_infer_ns, total_total_ns)));
            });

            let (count, total_prep_ns, total_infer_ns, total_total_ns) =
                match rx.recv_timeout(std::time::Duration::from_secs(if w * h > 1280 * 720 {
                    12
                } else {
                    3
                })) {
                    Ok(Some(t)) => t,
                    _ => {
                        eprintln!("  {:4}x{:4} {:>8}: CRASHED/TIMEOUT", w, h, be_name);
                        continue;
                    }
                };

            let elapsed = budget.saturating_sub(deadline.saturating_duration_since(Instant::now()));
            let secs = elapsed.as_secs_f64().max(0.001);
            let fps = count as f64 / secs;
            let mpix_s = (w as f64 * h as f64 * fps) / 1_000_000.0;
            let avg_prep_ms = if count > 0 {
                total_prep_ns as f64 / count as f64 / 1_000_000.0
            } else {
                0.0
            };
            let avg_infer_ms = if count > 0 {
                total_infer_ns as f64 / count as f64 / 1_000_000.0
            } else {
                0.0
            };
            let avg_total_ms = if count > 0 {
                total_total_ns as f64 / count as f64 / 1_000_000.0
            } else {
                0.0
            };
            eprintln!("  {:4}x{:4} {:>8}: {:5.1} fps ({:.1} Mpix/s) prep={:.1}ms infer={:.1}ms total={:.1}ms",
                w, h, be_name, fps, mpix_s, avg_prep_ms, avg_infer_ms, avg_total_ms);
        }

        let _ = std::fs::remove_file(&mnn);
        eprintln!();
    }
}
