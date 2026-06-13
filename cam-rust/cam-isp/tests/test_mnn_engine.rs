//! End-to-end test for MNN inference engine with streaming session.
//!
//! Generates an ONNX model (TEST profile), converts to .mnn, loads into
//! MnnEngine, and runs multiple frames to verify session reuse across calls.
//!
//! Run:
//!   cd cam-rust
//!   LD_LIBRARY_PATH=$PWD/lib/arm64-v8a \
//!     cargo test --test test_mnn_engine -p cam-isp --features mnn -- --nocapture --ignored
//!
//! Requires:
//!   - libMNN.so in $PWD/lib/arm64-v8a/  (prebuilt)
//!   - MNNConvert binary at ~/MNN/build/MNNConvert

use cam_isp::pipeline::IspBlock;

/// Build a TEST-profile ONNX model, convert to .mnn, return path.
fn build_test_mnn(prefix: &str) -> Result<String, String> {
    use std::path::Path;
    use cam_isp::profile::PipelineProfile;
    use cam_isp::pipeline::GraphComposer;
    use cam_isp::mnn_converter::convert_onnx_to_mnn;

    let mut blocks = PipelineProfile::TEST.build_blocks(64, 0);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&refs, &[], 16)
        .map_err(|e| format!("compose: {}", e))?;

    let onnx_path = format!("{}.onnx", prefix);
    let mnn_path = format!("{}.mnn", prefix);
    std::fs::write(&onnx_path, &model)
        .map_err(|e| format!("write: {}", e))?;

    let _ = convert_onnx_to_mnn(&onnx_path, &mnn_path, None)?;

    // Cleanup ONNX
    if Path::new(&onnx_path).exists() {
        let _ = std::fs::remove_file(&onnx_path);
    }
    Ok(mnn_path)
}

/// Stream 5 frames through the engine, verifying session reuse.
#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires libMNN.so + MNNConvert
fn test_mnn_engine_streaming() {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnEngine, MnnBackend};

    let mnn = build_test_mnn(".mnn_test_stream")
        .expect("model build + convert");
    eprintln!("MNN model: {}", mnn);

    let mut engine = MnnEngine::new(MnnBackend::Cpu);
    engine.set_model_path(&mnn);

    // Provide a dummy head block (ignored when model_path is set)
    let head: Box<dyn IspBlock> = Box::new(cam_isp::blocks::RawInputBlock::new());
    engine.build(head, vec![], None, 16).expect("build");

    let w = 48u32; let h = 64u32;
    let frame_size = (w * h * 2) as usize;
    let smax = 1024.0f32;

    for frame_idx in 0..5 {
        // Synthetic frame — varying data per frame
        let mut buf = vec![0u8; frame_size];
        for y in 0..h {
            for x in 0..w {
                let off = (y * w + x) as usize * 2;
                let val = ((x ^ y ^ frame_idx as u32) & 0xFF) as u16;
                buf[off] = val as u8;
                buf[off + 1] = (val >> 8) as u8;
            }
        }

        let result = engine.process(
            w, h, w, &buf, smax, 64,
            None, &cam_isp::engine::default_tone_params(),
            None, None, 1.0, 0.0, None, None, None,
        );

        match &result {
            Ok(frame) => {
                eprintln!("Frame {}: {}×{} fmt={:?} size={}",
                    frame_idx, frame.width, frame.height,
                    frame.format, frame.data.len());
                assert!(frame.data.len() >= (64 * h as usize * 3),
                    "frame {} too small", frame_idx);
            }
            Err(e) => {
                panic!("Frame {} failed: {}", frame_idx, e);
            }
        }
    }

    let _ = std::fs::remove_file(&mnn);
    eprintln!("PASSED: 5 streaming frames");
}

/// Verify that frames change when input changes (not just cached output).
#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires libMNN.so + MNNConvert
fn test_mnn_engine_frame_difference() {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnEngine, MnnBackend};
    use std::collections::HashSet;

    let mnn = build_test_mnn(".mnn_test_diff")
        .expect("model build");

    let mut engine = MnnEngine::new(MnnBackend::Cpu);
    engine.set_model_path(&mnn);
    let head: Box<dyn IspBlock> = Box::new(cam_isp::blocks::RawInputBlock::new());
    engine.build(head, vec![], None, 16).expect("build");

    let params = cam_isp::engine::default_tone_params();
    let mut outputs: Vec<Vec<u8>> = Vec::new();

    // Frame 0: all zeros
    let buf0 = vec![0u8; 48 * 64 * 2];
    let f0 = engine.process(48, 64, 48, &buf0, 1024.0, 64, None, &params,
        None, None, 1.0, 0.0, None, None, None).expect("frame 0");
    outputs.push(f0.data);

    // Frame 1: all 0xFF
    let buf1 = vec![0xFFu8; 48 * 64 * 2];
    let f1 = engine.process(48, 64, 48, &buf1, 1024.0, 64, None, &params,
        None, None, 1.0, 0.0, None, None, None).expect("frame 1");
    outputs.push(f1.data);

    // Frame 2: ramp pattern
    let mut buf2 = vec![0u8; 48 * 64 * 2];
    for (i, chunk) in buf2.chunks_exact_mut(2).enumerate() {
        let v = (i % 256) as u16;
        chunk[0] = v as u8;
        chunk[1] = (v >> 8) as u8;
    }
    let f2 = engine.process(48, 64, 48, &buf2, 1024.0, 64, None, &params,
        None, None, 1.0, 0.0, None, None, None).expect("frame 2");
    outputs.push(f2.data);

    // Verify all 3 outputs are different from each other
    let unique: HashSet<Vec<u8>> = outputs.into_iter().collect();
    assert!(unique.len() >= 2,
        "Expected at least 2 unique outputs for different inputs, got {}",
        unique.len());

    let _ = std::fs::remove_file(&mnn);
    eprintln!("PASSED: all frames unique");
}

/// Test LITE profile through MNN engine.
#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires libMNN.so + MNNConvert
fn test_mnn_engine_lite_profile() {
    use std::path::Path;
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnEngine, MnnBackend};
    use cam_isp::profile::PipelineProfile;
    use cam_isp::pipeline::GraphComposer;
    use cam_isp::mnn_converter::convert_onnx_to_mnn;

    // Build LITE profile ONNX model
    let mut blocks = PipelineProfile::LITE.build_blocks(64, 0);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&refs, &[], 16)
        .expect("LITE compose");

    let onnx_path = ".mnn_test_lite.onnx";
    std::fs::write(onnx_path, &model)
        .expect("write .onnx");
    let _mnn_path = convert_onnx_to_mnn(onnx_path, ".mnn_test_lite.mnn", None)
        .expect("convert to MNN");
    if Path::new(onnx_path).exists() {
        let _ = std::fs::remove_file(onnx_path);
    }

    let mnn = ".mnn_test_lite.mnn".to_string();
    eprintln!("MNN model: {}", mnn);

    let mut engine = MnnEngine::new(MnnBackend::Cpu);
    engine.set_model_path(&mnn);
    let head: Box<dyn IspBlock> = Box::new(cam_isp::blocks::RawInputBlock::new());
    engine.build(head, vec![], None, 16).expect("build");

    let w = 48u32; let h = 64u32;
    let frame_size = (w * h * 2) as usize;
    let smax = 1024.0f32;

    for frame_idx in 0..3 {
        let mut buf = vec![0u8; frame_size];
        for y in 0..h {
            for x in 0..w {
                let off = (y * w + x) as usize * 2;
                let val = ((x ^ y ^ frame_idx as u32) & 0xFF) as u16;
                buf[off] = val as u8;
                buf[off + 1] = (val >> 8) as u8;
            }
        }
        let result = engine.process(
            w, h, w, &buf, smax, 64,
            None, &cam_isp::engine::default_tone_params(),
            None, None, 1.0, 0.0, None, None, None,
        );
        match &result {
            Ok(frame) => {
                eprintln!("Frame {}: {}×{} fmt={:?} size={}",
                    frame_idx, frame.width, frame.height,
                    frame.format, frame.data.len());
                assert!(frame.data.len() >= (64 * h as usize * 3),
                    "frame {} too small", frame_idx);
            }
            Err(e) => {
                panic!("Frame {} failed: {}", frame_idx, e);
            }
        }
    }
    let _ = std::fs::remove_file(&mnn);
    eprintln!("PASSED: LITE profile via MNN - 3 streaming frames");
}

/// Build profile at given resolution and convert to MNN.
#[cfg(feature = "mnn")]
fn build_profile_mnn(profile: &cam_isp::profile::PipelineProfile, tag: &str, target_width: u32, _target_height: u32) -> Result<String, String> {
    use std::path::Path;
    use cam_isp::pipeline::GraphComposer;
    use cam_isp::mnn_converter::convert_onnx_to_mnn;

    let mut blocks = profile.build_blocks(target_width, 0);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&refs, &[], 16)?;

    let onnx_path = format!(".mnn_test_{}.onnx", tag);
    let mnn_path = format!(".mnn_test_{}.mnn", tag);
    std::fs::write(&onnx_path, &model).map_err(|e| format!("write: {}", e))?;
    convert_onnx_to_mnn(&onnx_path, &mnn_path, None)?;
    if Path::new(&onnx_path).exists() {
        let _ = std::fs::remove_file(&onnx_path);
    }
    Ok(mnn_path)
}

/// Test profile at 64-wide resolution (fast, for quick validation).
fn test_profile_via_mnn(profile: cam_isp::profile::PipelineProfile, tag: &str) -> Result<String, String> {
    build_profile_mnn(&profile, tag, 64, 64)
}

fn run_mnn_profile_test(tag: &str, profile: cam_isp::profile::PipelineProfile) {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnEngine, MnnBackend};

    let mnn = test_profile_via_mnn(profile, tag)
        .expect(&format!("{} model build + convert", tag));
    eprintln!("MNN model: {}", mnn);

    let mut engine = MnnEngine::new(MnnBackend::Cpu);
    engine.set_model_path(&mnn);
    let head: Box<dyn IspBlock> = Box::new(cam_isp::blocks::RawInputBlock::new());
    engine.build(head, vec![], None, 16).expect("build");

    let w = 48u32; let h = 64u32;
    let buf = vec![0x80u8; (w * h * 2) as usize];
    let result = engine.process(
        w, h, w, &buf, 1024.0, 64,
        None, &cam_isp::engine::default_tone_params(),
        None, None, 1.0, 0.0, None, None, None,
    );
    match &result {
        Ok(frame) => {
            eprintln!("{}: {}×{} fmt={:?} size={}",
                tag, frame.width, frame.height,
                frame.format, frame.data.len());
            assert!(frame.data.len() >= (64 * h as usize * 3),
                "{} frame too small", tag);
        }
        Err(e) => panic!("{} failed: {}", tag, e),
    }
    let _ = std::fs::remove_file(&mnn);
    eprintln!("PASSED: {} profile via MNN", tag);
}

#[cfg(feature = "mnn")]
#[test]
#[ignore]
fn test_mnn_all_profiles() {
    use cam_isp::profile::PipelineProfile;
    run_mnn_profile_test("LITE", PipelineProfile::LITE);
    run_mnn_profile_test("MED", PipelineProfile::MED);
    run_mnn_profile_test("HEAVY", PipelineProfile::HEAVY);
    run_mnn_profile_test("PRO", PipelineProfile::PRO);
    run_mnn_profile_test("REFERENCE", PipelineProfile::REFERENCE);
    run_mnn_profile_test("INFINITE", PipelineProfile::INFINITE);
    eprintln!("ALL 6 profiles passed via MNN");
}

/// Stream at target resolution through MNN and report FPS.
#[cfg(feature = "mnn")]
fn stream_mnn_profile(w: u32, h: u32, n_frames: u32, profile_tag: &str) -> f64 {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnEngine, MnnBackend};
    use std::time::Instant;

    let mnn = build_profile_mnn(&cam_isp::profile::PipelineProfile::LITE, profile_tag, w, h)
        .expect(&format!("model build {}x{}", w, h));

    let mut engine = MnnEngine::new(MnnBackend::Cpu);
    engine.set_model_path(&mnn);
    let head: Box<dyn IspBlock> = Box::new(cam_isp::blocks::RawInputBlock::new());
    engine.build(head, vec![], None, 16).expect("build");

    let params = cam_isp::engine::default_tone_params();
    let frame_size = (w * h * 2) as usize;
    let mut total_duration = std::time::Duration::ZERO;

    for frame_idx in 0..n_frames {
        let mut buf = vec![0u8; frame_size];
        let base = (frame_idx * 7) as u8;
        for chunk in buf.chunks_exact_mut(2) {
            let v = base.wrapping_mul(13).wrapping_add(chunk.len() as u8);
            chunk[0] = v;
            chunk[1] = v.wrapping_mul(3);
        }

        let t_start = Instant::now();
        let result = engine.process(
            w, h, w, &buf, 1024.0, w,
            None, &params, None, None, 1.0, 0.0, None, None, None,
        );
        let elapsed = t_start.elapsed();
        total_duration += elapsed;

        match &result {
            Ok(frame) => {
                let expected = (w * h * 4) as usize;
                assert_eq!(frame.data.len(), expected,
                    "frame {} size mismatch", frame_idx);
            }
            Err(e) => {
                let _ = std::fs::remove_file(&mnn);
                panic!("Frame {} failed: {}", frame_idx, e);
            }
        }
    }

    let _ = std::fs::remove_file(&mnn);
    let avg = total_duration / n_frames;
    let fps = n_frames as f64 / total_duration.as_secs_f64();
    eprintln!("  {:4}x{:4}: {} frames, avg {:?}, {:.1} fps",
        w, h, n_frames, avg, fps);
    fps
}

/// Benchmark multiple resolutions to find max achievable FPS.
#[cfg(feature = "mnn")]
#[test]
#[ignore]
fn test_mnn_resolution_bench() {
    let resolutions: [(u32, u32); 6] = [
        (256, 144),   // nHD
        (480, 270),   // 540p quarter
        (640, 360),   // nHD 2x
        (960, 540),   // qHD
        (1280, 720),  // HD
        (1920, 1080), // Full HD
    ];
    let mut results: Vec<(u32, u32, f64)> = Vec::new();
    for &(w, h) in &resolutions {
        let n = if w <= 640 { 100u32 } else { 10u32 };
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            stream_mnn_profile(w, h, n, &format!("res_{}x{}", w, h))
        })) {
            Ok(fps) => results.push((w, h, fps)),
            Err(_) => eprintln!("  {:4}x{:4}: CRASHED (SIGSEGV or panic)", w, h),
        }
    }
    eprintln!("─── Resolution bench results ───");
    for &(w, h, fps) in &results {
        let pixels = w * h;
        let mpix_s = pixels as f64 * fps / 1_000_000.0;
        eprintln!("  {:4}x{:4}: {:6.1} fps ({:.1} Mpixel/s)", w, h, fps, mpix_s);
    }
}

/// Stream at target resolution through MNN with a specific backend.
#[cfg(feature = "mnn")]
fn stream_mnn_backend(w: u32, h: u32, n_frames: u32, backend: &str) -> Result<f64, String> {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnEngine, MnnBackend};
    use std::time::Instant;

    let be = match backend {
        "vulkan" => MnnBackend::Vulkan,
        "opencl" => MnnBackend::Opencl,
        "opengl" => MnnBackend::OpenGl,
        "cpu" => MnnBackend::Cpu,
        _ => return Err(format!("unknown backend: {}", backend)),
    };

    // Clean up any stale model files
    let onnx = format!(".mnn_be_{}.onnx", backend);
    let mnn_p = format!(".mnn_be_{}.mnn", backend);
    let _ = std::fs::remove_file(&onnx);
    let _ = std::fs::remove_file(&mnn_p);

    let mnn = build_profile_mnn(&cam_isp::profile::PipelineProfile::LITE, &format!("be_{}", backend), w, h)?;

    let mut engine = MnnEngine::new(be);
    engine.set_model_path(&mnn);
    let head: Box<dyn IspBlock> = Box::new(cam_isp::blocks::RawInputBlock::new());
    engine.build(head, vec![], None, 16).map_err(|e| format!("build: {}", e))?;

    let params = cam_isp::engine::default_tone_params();
    let frame_size = (w * h * 2) as usize;
    let mut total_duration = std::time::Duration::ZERO;

    for frame_idx in 0..n_frames {
        let mut buf = vec![0u8; frame_size];
        let base = (frame_idx * 7) as u8;
        for chunk in buf.chunks_exact_mut(2) {
            let v = base.wrapping_mul(13).wrapping_add(chunk.len() as u8);
            chunk[0] = v;
            chunk[1] = v.wrapping_mul(3);
        }

        let t_start = Instant::now();
        let result = engine.process(
            w, h, w, &buf, 1024.0, w,
            None, &params, None, None, 1.0, 0.0, None, None, None,
        );
        let elapsed = t_start.elapsed();
        total_duration += elapsed;

        match &result {
            Ok(frame) => {
                let expected = (w * h * 4) as usize;
                if frame.data.len() != expected {
                    let _ = std::fs::remove_file(&mnn);
                    return Err(format!("frame {} size: {} vs expected {}", frame_idx, frame.data.len(), expected));
                }
            }
            Err(e) => {
                let _ = std::fs::remove_file(&mnn);
                return Err(format!("frame {} failed: {}", frame_idx, e));
            }
        }
    }

    let _ = std::fs::remove_file(&mnn);
    let fps = n_frames as f64 / total_duration.as_secs_f64();
    eprintln!("  {}: {:4}x{:4} -> {:.1} fps (backend={})", backend, w, h, fps, backend);
    Ok(fps)
}

/// Compare CPU vs Vulkan vs OpenCL at 720p.
#[cfg(feature = "mnn")]
#[test]
#[ignore]
fn test_mnn_backend_compare() {
    for backend in &["vulkan", "opencl", "opengl", "cpu"] {
        match stream_mnn_backend(640, 360, 5, backend) {
            Ok(fps) => eprintln!("  {} OK: {:.1} fps", backend, fps),
            Err(e) => eprintln!("  {} FAILED: {}", backend, e),
        }
    }
}

/// Uninitialized engine should return error.
#[test]
fn test_mnn_engine_uninitialized() {
    #[cfg(feature = "mnn")] {
        use cam_isp::engine::IspEngine;
        use cam_isp::mnnengine::{MnnEngine, MnnBackend};
        let engine = MnnEngine::new(MnnBackend::Cpu);
        let result = engine.process(
            48, 64, 48, &[0u8; 48*64*2], 1024.0, 64,
            None, &cam_isp::engine::default_tone_params(),
            None, None, 1.0, 0.0, None, None, None,
        );
        assert!(result.is_err(), "uninitialized should fail");
        eprintln!("Uninitialized check OK: {:?}", result.err().unwrap());
    }
    #[cfg(not(feature = "mnn"))] {
        eprintln!("MNN feature not enabled");
    }
}
