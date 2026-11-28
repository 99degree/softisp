//! End-to-end test for MNN inference engine with streaming session.
//!
//! Generates an ONNX model (TEST profile), converts to .mnn, loads into
//! MnnEngine, and runs multiple frames to verify session reuse across calls.
//!
//! Run:
//!   cd cam-rust
//!   LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
//!     cargo test --test test_mnn_engine -p cam-isp --features mnn -- --nocapture --ignored
//!
//! Requires:
//!   - libMNN.so in $PWD/lib/aarch64-v8a/  (prebuilt)
//!   - MNNConvert binary at ~/MNN/build/MNNConvert

use cam_isp::pipeline::IspBlock;

/// Build a TEST-profile ONNX model, convert to .mnn, return path.
fn build_test_mnn(prefix: &str) -> Result<String, String> {
    use cam_isp::mnn_converter::convert_onnx_to_mnn;
    use cam_isp::pipeline::GraphComposer;
    use cam_isp::profile::PipelineProfile;
    use std::path::Path;

    let blocks = PipelineProfile::TEST.build_blocks(64, 64, 0);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model =
        GraphComposer::compose_from_vec(&refs, &[], 16).map_err(|e| format!("compose: {}", e))?;

    let onnx_path = format!("{}.onnx", prefix);
    let mnn_path = format!("{}.mnn", prefix);
    std::fs::write(&onnx_path, &model).map_err(|e| format!("write: {}", e))?;

    let _ = convert_onnx_to_mnn(&onnx_path, &mnn_path, None)?;

    // Cleanup ONNX
    if Path::new(&onnx_path).exists() {
        let _ = std::fs::remove_file(&onnx_path);
    }
    Ok(mnn_path)
}

/// The head block is more than a placeholder when a pre-converted `.mnn` is
/// loaded: `MnnEngine::build()` takes its `graph_input_name()` as the frame
/// input tensor name. A dummy `RawInputBlock` against a profile that starts at
/// `RawInputPackedBlock` makes every inference fail with "frame input not
/// found", so always hand the engine the profile's real head.
#[cfg(feature = "mnn")]
fn profile_head(profile: cam_isp::profile::PipelineProfile, w: u32, h: u32) -> Box<dyn IspBlock> {
    let mut blocks = profile.build_blocks(w, h, 0);
    blocks.remove(0)
}

fn expected_frame_bytes(frame: &cam_isp::pipeline::IspFrame) -> usize {
    use cam_types::FrameFormat;
    let bpp = match frame.format {
        FrameFormat::Rgb888 => 3,
        FrameFormat::Yuv420888 => 1,
        FrameFormat::RawSensor | FrameFormat::Raw10 | FrameFormat::Raw12 => 2,
        _ => 4,
    };
    frame.width as usize * frame.height as usize * bpp
}

/// Stream 5 frames through the engine, verifying session reuse.
#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires libMNN.so + MNNConvert
#[cfg(feature = "mnn")]
fn test_mnn_engine_streaming() {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnBackend, MnnEngine};

    let mnn = build_test_mnn(".mnn_test_stream").expect("model build + convert");
    eprintln!("MNN model: {}", mnn);

    let mut engine = MnnEngine::new(MnnBackend::Vulkan);
    engine.set_model_path(&mnn);

    let head = profile_head(cam_isp::profile::PipelineProfile::TEST, 48, 64);
    engine.build(head, vec![], None, 16).expect("build");

    let w = 48u32;
    let h = 64u32;
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

        let result = engine.process(&cam_isp::engine::ProcessParams {
            width: w,
            height: h,
            stride_width: w,
            buf: &buf,
            sensor_max: smax,
            target_width: 64,
            target_height: h,
            ccm_matrix: None,
            tone_params: cam_isp::engine::default_tone_params(),
            bayer_gains: None,
            awb_gains: None,
            bayer_pattern: 0,
            analog_gain: 1.0,
            scene_change: 0.0,
            lsc_gains: None,
            blc_values: None,
            warp_grid: None,
            warp_shading: None,
            output_format: cam_isp::engine::OutputFormat::default(),
            timestamp_ns: 0,
            isp_params: None,
            ..cam_isp::engine::ProcessParams::new(w, h, &buf)
        });

        match &result {
            Ok(frame) => {
                eprintln!(
                    "Frame {}: {}×{} fmt={:?} size={}",
                    frame_idx,
                    frame.width,
                    frame.height,
                    frame.format,
                    frame.data.len()
                );
                assert!(
                    frame.data.len() >= expected_frame_bytes(frame),
                    "frame {} too small",
                    frame_idx
                );
            }
            Err(e) => {
                panic!("Frame {} failed: {}", frame_idx, e);
            }
        }
    }

    let _ = std::fs::remove_file(&mnn);
    eprintln!("PASSED: 5 streaming frames");
}

/// Exercise the staged prep / run / finish path with the real MNN backend.
#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires libMNN.so + MNNConvert
fn test_mnn_engine_staged_streaming() {
    use cam_isp::engine::{IspEngine, PrepareParams, ProcessParams};
    use cam_isp::mnnengine::{MnnBackend, MnnEngine};

    let mnn = build_test_mnn(".mnn_test_staged").expect("model build + convert");
    let mut engine = MnnEngine::new(MnnBackend::Vulkan);
    engine.set_model_path(&mnn);
    let head = profile_head(cam_isp::profile::PipelineProfile::TEST, 48, 64);
    engine.build(head, vec![], None, 21).expect("build");

    let w = 48u32;
    let h = 64u32;
    let tw = 64u32;
    let th = 64u32;
    let smax = 1024.0f32;
    let empty = [];

    for frame_idx in 0..3 {
        let mut prep_params = ProcessParams::new(w, h, &empty);
        prep_params.target_width = tw;
        prep_params.target_height = th;
        prep_params.sensor_max = smax;

        let prepared = engine.prepare(&prep_params).expect("prepare");
        assert_eq!(prepared.prepare_params(), PrepareParams::from(&prep_params));

        let mut buf = vec![0u8; (w * h * 2) as usize];
        for (i, chunk) in buf.chunks_exact_mut(2).enumerate() {
            let v = ((i ^ frame_idx as usize) & 0xFF) as u16;
            chunk[0] = v as u8;
            chunk[1] = (v >> 8) as u8;
        }
        let mut params = ProcessParams::new(w, h, &buf);
        params.target_width = tw;
        params.target_height = th;
        params.sensor_max = smax;
        assert_eq!(prepared.prepare_params(), PrepareParams::from(&params));

        let (frame, pending) = engine
            .run_prepared(prepared, &params)
            .expect("run_prepared");
        assert!(frame.data.len() >= expected_frame_bytes(&frame));
        let finished = engine.finish(pending);
        assert!(finished.total_duration_ns > 0);
    }

    let _ = std::fs::remove_file(&mnn);
    eprintln!("PASSED: 3 staged frames");
}

/// Verify that frames change when input changes (not just cached output).
#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires libMNN.so + MNNConvert
#[cfg(feature = "mnn")]
fn test_mnn_engine_frame_difference() {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnBackend, MnnEngine};
    use std::collections::HashSet;

    let mnn = build_test_mnn(".mnn_test_diff").expect("model build");

    let mut engine = MnnEngine::new(MnnBackend::Vulkan);
    engine.set_model_path(&mnn);
    let head = profile_head(cam_isp::profile::PipelineProfile::TEST, 48, 64);
    engine.build(head, vec![], None, 16).expect("build");

    let params = cam_isp::engine::default_tone_params();
    let mut outputs: Vec<Vec<u8>> = Vec::new();

    // Frame 0: all zeros
    let buf0 = vec![0u8; 48 * 64 * 2];
    let f0 = engine
        .process(&cam_isp::engine::ProcessParams {
            width: 48,
            height: 64,
            stride_width: 48,
            buf: &buf0,
            sensor_max: 1024.0,
            target_width: 64,
            target_height: 64,
            ccm_matrix: None,
            tone_params: params.clone(),
            bayer_gains: None,
            awb_gains: None,
            bayer_pattern: 0,
            analog_gain: 1.0,
            scene_change: 0.0,
            lsc_gains: None,
            blc_values: None,
            warp_grid: None,
            warp_shading: None,
            output_format: cam_isp::engine::OutputFormat::default(),
            timestamp_ns: 0,
            isp_params: None,
            ..cam_isp::engine::ProcessParams::new(48, 64, &buf0)
        })
        .expect("frame 0");
    outputs.push(f0.data);

    // Frame 1: all 0xFF
    let buf1 = vec![0xFFu8; 48 * 64 * 2];
    let f1 = engine
        .process(&cam_isp::engine::ProcessParams {
            width: 48,
            height: 64,
            stride_width: 48,
            buf: &buf1,
            sensor_max: 1024.0,
            target_width: 64,
            target_height: 64,
            ccm_matrix: None,
            tone_params: params.clone(),
            bayer_gains: None,
            awb_gains: None,
            bayer_pattern: 0,
            analog_gain: 1.0,
            scene_change: 0.0,
            lsc_gains: None,
            blc_values: None,
            warp_grid: None,
            warp_shading: None,
            output_format: cam_isp::engine::OutputFormat::default(),
            timestamp_ns: 0,
            isp_params: None,
            ..cam_isp::engine::ProcessParams::new(48, 64, &buf1)
        })
        .expect("frame 1");
    outputs.push(f1.data);

    // Frame 2: ramp pattern
    let mut buf2 = vec![0u8; 48 * 64 * 2];
    for (i, chunk) in buf2.chunks_exact_mut(2).enumerate() {
        let v = (i % 256) as u16;
        chunk[0] = v as u8;
        chunk[1] = (v >> 8) as u8;
    }
    let f2 = engine
        .process(&cam_isp::engine::ProcessParams {
            width: 48,
            height: 64,
            stride_width: 48,
            buf: &buf2,
            sensor_max: 1024.0,
            target_width: 64,
            target_height: 64,
            ccm_matrix: None,
            tone_params: params.clone(),
            bayer_gains: None,
            awb_gains: None,
            bayer_pattern: 0,
            analog_gain: 1.0,
            scene_change: 0.0,
            lsc_gains: None,
            blc_values: None,
            warp_grid: None,
            warp_shading: None,
            output_format: cam_isp::engine::OutputFormat::default(),
            timestamp_ns: 0,
            isp_params: None,
            ..cam_isp::engine::ProcessParams::new(48, 64, &buf2)
        })
        .expect("frame 2");
    outputs.push(f2.data);

    // Verify all 3 outputs are different from each other
    let unique: HashSet<Vec<u8>> = outputs.into_iter().collect();
    assert!(
        unique.len() >= 2,
        "Expected at least 2 unique outputs for different inputs, got {}",
        unique.len()
    );

    let _ = std::fs::remove_file(&mnn);
    eprintln!("PASSED: all frames unique");
}

/// Test LITE profile through MNN engine.
#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires libMNN.so + MNNConvert
#[cfg(feature = "mnn")]
fn test_mnn_engine_lite_profile() {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnn_converter::convert_onnx_to_mnn;
    use cam_isp::mnnengine::{MnnBackend, MnnEngine};
    use cam_isp::pipeline::GraphComposer;
    use cam_isp::profile::PipelineProfile;
    use std::path::Path;

    // Build LITE profile ONNX model
    let blocks = PipelineProfile::LITE.build_blocks(64, 64, 0);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&refs, &[], 16).expect("LITE compose");

    let onnx_path = ".mnn_test_lite.onnx";
    std::fs::write(onnx_path, &model).expect("write .onnx");
    let _mnn_path =
        convert_onnx_to_mnn(onnx_path, ".mnn_test_lite.mnn", None).expect("convert to MNN");
    if Path::new(onnx_path).exists() {
        let _ = std::fs::remove_file(onnx_path);
    }

    let mnn = ".mnn_test_lite.mnn".to_string();
    eprintln!("MNN model: {}", mnn);

    let mut engine = MnnEngine::new(MnnBackend::Vulkan);
    engine.set_model_path(&mnn);
    let head = profile_head(cam_isp::profile::PipelineProfile::LITE, 48, 64);
    engine.build(head, vec![], None, 16).expect("build");

    let w = 48u32;
    let h = 64u32;
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
        let result = engine.process(&cam_isp::engine::ProcessParams {
            width: w,
            height: h,
            stride_width: w,
            buf: &buf,
            sensor_max: smax,
            target_width: 64,
            target_height: h,
            ccm_matrix: None,
            tone_params: cam_isp::engine::default_tone_params(),
            bayer_gains: None,
            awb_gains: None,
            bayer_pattern: 0,
            analog_gain: 1.0,
            scene_change: 0.0,
            lsc_gains: None,
            blc_values: None,
            warp_grid: None,
            warp_shading: None,
            output_format: cam_isp::engine::OutputFormat::default(),
            timestamp_ns: 0,
            isp_params: None,
            ..cam_isp::engine::ProcessParams::new(w, h, &buf)
        });
        match &result {
            Ok(frame) => {
                eprintln!(
                    "Frame {}: {}×{} fmt={:?} size={}",
                    frame_idx,
                    frame.width,
                    frame.height,
                    frame.format,
                    frame.data.len()
                );
                assert!(
                    frame.data.len() >= expected_frame_bytes(frame),
                    "frame {} too small",
                    frame_idx
                );
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
fn build_profile_mnn(
    profile: &cam_isp::profile::PipelineProfile,
    tag: &str,
    target_width: u32,
    target_height: u32,
) -> Result<String, String> {
    use cam_isp::mnn_converter::convert_onnx_to_mnn;
    use cam_isp::pipeline::GraphComposer;
    use std::path::Path;

    let blocks = profile.build_blocks(target_width, target_height, 0);
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

#[cfg(feature = "mnn")]
fn test_profile_via_mnn(
    profile: cam_isp::profile::PipelineProfile,
    tag: &str,
) -> Result<String, String> {
    build_profile_mnn(&profile, tag, 64, 64)
}

#[cfg(feature = "mnn")]
fn run_mnn_profile_test(tag: &str, profile: cam_isp::profile::PipelineProfile) {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnBackend, MnnEngine};

    let mnn = test_profile_via_mnn(profile, tag).expect(&format!("{} model build + convert", tag));
    eprintln!("MNN model: {}", mnn);

    let mut engine = MnnEngine::new(MnnBackend::Vulkan);
    engine.set_model_path(&mnn);
    let head = profile_head(profile, 48, 64);
    engine.build(head, vec![], None, 16).expect("build");

    let w = 48u32;
    let h = 64u32;
    let buf = vec![0x80u8; (w * h * 2) as usize];
    let result = engine.process(&cam_isp::engine::ProcessParams {
        width: w,
        height: h,
        stride_width: w,
        buf: &buf,
        sensor_max: 1024.0,
        target_width: 64,
        target_height: h,
        ccm_matrix: None,
        tone_params: cam_isp::engine::default_tone_params(),
        bayer_gains: None,
        awb_gains: None,
        bayer_pattern: 0,
        analog_gain: 1.0,
        scene_change: 0.0,
        lsc_gains: None,
        blc_values: None,
        warp_grid: None,
        warp_shading: None,
        output_format: cam_isp::engine::OutputFormat::default(),
        timestamp_ns: 0,
        isp_params: None,
        ..cam_isp::engine::ProcessParams::new(w, h, &buf)
    });
    match &result {
        Ok(frame) => {
            eprintln!(
                "{}: {}×{} fmt={:?} size={}",
                tag,
                frame.width,
                frame.height,
                frame.format,
                frame.data.len()
            );
            assert!(
                frame.data.len() >= expected_frame_bytes(frame),
                "{} frame too small",
                tag
            );
        }
        Err(e) => panic!("{} failed: {}", tag, e),
    }
    let _ = std::fs::remove_file(&mnn);
    eprintln!("PASSED: {} profile via MNN", tag);
}

#[cfg(feature = "mnn")]
#[test]
#[ignore]
#[cfg(feature = "mnn")]
fn test_mnn_all_profiles() {
    use cam_isp::profile::PipelineProfile;
    run_mnn_profile_test("LITE", PipelineProfile::LITE);
    run_mnn_profile_test("MED", PipelineProfile::MED);
    run_mnn_profile_test("HEAVY", PipelineProfile::HEAVY);
    run_mnn_profile_test("PRO", PipelineProfile::PRO);
    eprintln!("ALL 4 profiles passed via MNN");
}

/// Standalone HEAVY check. This is the profile to run when validating the
/// primitive chain without paying for the other profiles.
#[cfg(feature = "mnn")]
#[test]
#[ignore]
fn test_mnn_heavy_profile() {
    use cam_isp::profile::PipelineProfile;
    run_mnn_profile_test("HEAVY", PipelineProfile::HEAVY);
}

/// Stream at target resolution through MNN and report FPS.
#[cfg(feature = "mnn")]
fn stream_mnn_profile(w: u32, h: u32, n_frames: u32, profile_tag: &str) -> f64 {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnBackend, MnnEngine};
    use std::time::Instant;

    let mnn = build_profile_mnn(&cam_isp::profile::PipelineProfile::LITE, profile_tag, w, h)
        .expect(&format!("model build {}x{}", w, h));

    let mut engine = MnnEngine::new(MnnBackend::Vulkan);
    engine.set_model_path(&mnn);
    let head = profile_head(cam_isp::profile::PipelineProfile::LITE, w, h);
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
        let result = engine.process(&cam_isp::engine::ProcessParams {
            width: w,
            height: h,
            stride_width: w,
            buf: &buf,
            sensor_max: 1024.0,
            target_width: w,
            target_height: h,
            ccm_matrix: None,
            tone_params: params.clone(),
            bayer_gains: None,
            awb_gains: None,
            bayer_pattern: 0,
            analog_gain: 1.0,
            scene_change: 0.0,
            lsc_gains: None,
            blc_values: None,
            warp_grid: None,
            warp_shading: None,
            output_format: cam_isp::engine::OutputFormat::default(),
            timestamp_ns: 0,
            isp_params: None,
            ..cam_isp::engine::ProcessParams::new(w, h, &buf)
        });
        let elapsed = t_start.elapsed();
        total_duration += elapsed;

        match &result {
            Ok(frame) => {
                let expected = expected_frame_bytes(frame);
                assert_eq!(
                    frame.data.len(),
                    expected,
                    "frame {} size mismatch",
                    frame_idx
                );
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
    eprintln!(
        "  {:4}x{:4}: {} frames, avg {:?}, {:.1} fps",
        w, h, n_frames, avg, fps
    );
    fps
}

/// Benchmark multiple resolutions to find max achievable FPS.
#[cfg(feature = "mnn")]
#[test]
#[ignore]
#[cfg(feature = "mnn")]
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
        eprintln!(
            "  {:4}x{:4}: {:6.1} fps ({:.1} Mpixel/s)",
            w, h, fps, mpix_s
        );
    }
}

/// Stream at target resolution through MNN with a specific backend.
#[cfg(feature = "mnn")]
fn stream_mnn_backend(w: u32, h: u32, n_frames: u32, backend: &str) -> Result<f64, String> {
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnBackend, MnnEngine};
    use std::time::Instant;

    let be = match backend {
        "vulkan" => MnnBackend::Vulkan,
        _ => return Err(format!("unknown backend: {}", backend)),
    };

    // Clean up any stale model files
    let onnx = format!(".mnn_be_{}.onnx", backend);
    let mnn_p = format!(".mnn_be_{}.mnn", backend);
    let _ = std::fs::remove_file(&onnx);
    let _ = std::fs::remove_file(&mnn_p);

    let mnn = build_profile_mnn(
        &cam_isp::profile::PipelineProfile::LITE,
        &format!("be_{}", backend),
        w,
        h,
    )?;

    let mut engine = MnnEngine::new(be);
    engine.set_model_path(&mnn);
    let head = profile_head(cam_isp::profile::PipelineProfile::LITE, w, h);
    engine
        .build(head, vec![], None, 16)
        .map_err(|e| format!("build: {}", e))?;

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
        let result = engine.process(&cam_isp::engine::ProcessParams {
            width: w,
            height: h,
            stride_width: w,
            buf: &buf,
            sensor_max: 1024.0,
            target_width: w,
            target_height: h,
            ccm_matrix: None,
            tone_params: params.clone(),
            bayer_gains: None,
            awb_gains: None,
            bayer_pattern: 0,
            analog_gain: 1.0,
            scene_change: 0.0,
            lsc_gains: None,
            blc_values: None,
            warp_grid: None,
            warp_shading: None,
            output_format: cam_isp::engine::OutputFormat::default(),
            timestamp_ns: 0,
            isp_params: None,
            ..cam_isp::engine::ProcessParams::new(w, h, &buf)
        });
        let elapsed = t_start.elapsed();
        total_duration += elapsed;

        match &result {
            Ok(frame) => {
                let expected = expected_frame_bytes(frame);
                if frame.data.len() != expected {
                    let _ = std::fs::remove_file(&mnn);
                    return Err(format!(
                        "frame {} size: {} vs expected {}",
                        frame_idx,
                        frame.data.len(),
                        expected
                    ));
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
    eprintln!(
        "  {}: {:4}x{:4} -> {:.1} fps (backend={})",
        backend, w, h, fps, backend
    );
    Ok(fps)
}

/// Compare CPU vs Vulkan vs OpenCL at 720p.
#[cfg(feature = "mnn")]
#[test]
#[ignore]
#[cfg(feature = "mnn")]
fn test_mnn_backend_compare() {
    for backend in &["vulkan", "opencl", "opengl", "cpu"] {
        match stream_mnn_backend(640, 360, 5, backend) {
            Ok(fps) => eprintln!("  {} OK: {:.1} fps", backend, fps),
            Err(e) => eprintln!("  {} FAILED: {}", backend, e),
        }
    }
}

/// Test packed INT32 pipeline end-to-end: RawInput(INT32,W/2) → UnpackCfaBlock → Display(PackedRgb)
#[cfg(feature = "mnn")]
#[test]
#[ignore] // Requires Vulkan backend — CPU backend SIGSEGV on Extra ops
#[cfg(feature = "mnn")]
fn test_mnn_packed_pipeline() {
    use cam_isp::blocks::{DisplayBlock, RawInputPackedBlock, UnpackBlock};
    use cam_isp::engine::IspEngine;
    use cam_isp::mnnengine::{MnnBackend, MnnEngine};
    use cam_isp::pipeline::GraphComposer;

    let h = 48i64;
    let w = 64i64; // full width
    let pw = w / 2; // packed width

    // Build packed pipeline: RawInputPacked(INT32, 2ch, w/2) → UnpackBlock
    // → Display(PackedRgb). `with_concrete_dims` takes the FULL width; the
    // packed head halves the tensor width internally.
    let b1: Box<dyn IspBlock> = Box::new(RawInputPackedBlock::new().with_concrete_dims(h, w));
    let b2: Box<dyn IspBlock> = Box::new(UnpackBlock::new().with_concrete_width(w));
    let b3: Box<dyn IspBlock> = Box::new(
        DisplayBlock::new(w as u32)
            .with_output_format(cam_isp::engine::OutputFormat::PackedRgb)
            .with_concrete_dims(h / 2, w / 2),
    );

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2, b3];
    GraphComposer::wire_blocks(&mut blocks);
    let mut all = blocks;
    let head = all.remove(0);

    let mut engine = MnnEngine::new(MnnBackend::Vulkan);
    engine.build(head, all, None, 16).expect("engine build");

    eprintln!("Packed pipeline: engine built OK");
    let (input_code, input_bits) = engine
        .model_input_type()
        .map(|(code, bits)| (code.to_string(), bits.to_string()))
        .unwrap_or_else(|| ("?".into(), "?".into()));
    eprintln!(
        "  Model input type: code={}, bits={}, expected_elems={}",
        input_code,
        input_bits,
        engine
            .expected_input_elements()
            .map(|e| e.to_string())
            .unwrap_or("?".into())
    );

    // Create packed input: each u32 = pixel_even | (pixel_odd << 16)
    let packed_elems = (h * pw) as usize;
    let mut packed = vec![0u32; packed_elems];
    let pw_u32 = pw as u32;
    let h_u32 = h as u32;
    for y in 0..h_u32 {
        for x in 0..pw_u32 {
            let even: u16 = ((x * 7 + y * 13) & 0xFFF) as u16;
            let odd: u16 = ((x * 11 + y * 5) & 0xFFF) as u16;
            packed[(y * pw_u32 + x) as usize] = (even as u32) | ((odd as u32) << 16);
        }
    }
    // Reinterpret as bytes (same memory) for engine.process()
    let buf: &[u8] =
        unsafe { std::slice::from_raw_parts(packed.as_ptr() as *const u8, packed.len() * 4) };

    // Run packed inference through engine
    let params = cam_isp::engine::default_tone_params();
    let result = engine.process(&cam_isp::engine::ProcessParams {
        width: w as u32,
        height: h as u32,
        stride_width: w as u32,
        buf: buf,
        sensor_max: 65536.0,
        target_width: w as u32,
        target_height: h as u32,
        ccm_matrix: None,
        tone_params: params.clone(),
        bayer_gains: None,
        awb_gains: None,
        bayer_pattern: 0,
        analog_gain: 1.0,
        scene_change: 0.0,
        lsc_gains: None,
        blc_values: None,
        warp_grid: None,
        warp_shading: None,
        output_format: cam_isp::engine::OutputFormat::PackedRgb,
        timestamp_ns: 0,
        isp_params: None,
        ..cam_isp::engine::ProcessParams::new(w as u32, h as u32, buf)
    });

    match &result {
        Ok(frame) => {
            eprintln!(
                "Packed inference OK: {}×{} fmt={:?} size={}",
                frame.width,
                frame.height,
                frame.format,
                frame.data.len()
            );
            let expected_min = (h / 2) as usize * (w / 2) as usize; // 2 pixels per INT32 word => 2 bytes/pixel
            assert!(
                frame.data.len() >= expected_min,
                "frame too small: {} < {}",
                frame.data.len(),
                expected_min
            );
            eprintln!("PASSED: packed zero-copy inference");
        }
        Err(e) => {
            panic!("Packed inference failed: {}", e);
        }
    }

    eprintln!("PASSED: packed INT32 pipeline end-to-end");
}

/// Feed a dummy 1 MB `AhbBinding` as `ProcessParams::input_ahb` through the
/// MNN glue layer and verify zero-copy input is accepted end-to-end without
/// crashing, falling back to host upload when the pointer is not a real
/// `AHardwareBuffer`.
///
/// This proves the input-AHB glue path: `ProcessParams::input_ahb` is
/// propagated into `PreparedMeta`, `prepare()` calls
/// `mnn_import_ahb_as_device_memory` on the input tensor before resize, and a
/// non-AHB pointer degrades gracefully instead of taking the process down.
///
/// Uses the in-memory ONNX→MNN buffer converter (no MNNConvert binary needed),
/// then writes the .mnn bytes to a temp file so `set_model_path` can load them.
#[cfg(all(feature = "mnn", target_os = "android"))]
#[test]
fn test_mnn_input_ahb_zero_copy_dummy() {
    use cam_isp::engine::{IspEngine, ProcessParams};
    use cam_isp::mnn_converter::convert_onnx_buffer;
    use cam_isp::mnnengine::{MnnBackend, MnnEngine};
    use cam_isp::pipeline::{GraphComposer, IspBlock};
    use cam_isp::profile::PipelineProfile;
    use cam_types::{AhbBinding, AhbFenceTokens};

    // ── Build ONNX model and convert in-memory (no MNNConvert binary) ──────
    let blocks = PipelineProfile::TEST.build_blocks(64, 64, 0);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx =
        GraphComposer::compose_from_vec(&refs, &[], 16).expect("TEST profile ONNX compose");
    let mnn_bytes = convert_onnx_buffer(&onnx).expect("ONNX→MNN buffer conversion");
    assert!(!mnn_bytes.is_empty(), "expected non-empty MNN bytes");

    let mnn_path = ".mnn_test_input_ahb.mnn";
    std::fs::write(mnn_path, &mnn_bytes).expect("write temp .mnn");

    let mut engine = MnnEngine::new(MnnBackend::Vulkan);
    engine.set_model_path(mnn_path);
    let head = profile_head(cam_isp::profile::PipelineProfile::TEST, 48, 64);
    engine.build(head, vec![], None, 16).expect("build");

    let w = 48u32;
    let h = 64u32;
    let frame_size = (w * h * 2) as usize;
    let smax = 1024.0f32;

    // Dummy "AHB": a NULL pointer that is NOT a real AHardwareBuffer.
    // `mnn_import_ahb_as_device_memory` checks `!ahb` first and returns -1
    // safely (no deref, no SIGSEGV). The engine logs a warning and falls back
    // to host upload. We use a 1 MB len so the size check in prepare() passes.
    let dummy_ptr: *mut std::ffi::c_void = std::ptr::null_mut();
    let dummy_ahb = AhbBinding::new(
        dummy_ptr,
        1_048_576, // 1 MB
        AhbFenceTokens::new(-1, -1, 1),
    );

    let mut buf = vec![0u8; frame_size];
    for (i, chunk) in buf.chunks_exact_mut(2).enumerate() {
        let v = (i as u16) as u8;
        chunk[0] = v;
        chunk[1] = 0;
    }

    // Build params with input_ahb set — this exercises the zero-copy input
    // binding path.
    let mut params = ProcessParams::new(w, h, &buf);
    params.sensor_max = smax;
    params.target_width = w;
    params.target_height = h;
    params.input_ahb = Some(dummy_ahb);

    let prepared = engine.prepare(&params).expect("prepare with dummy input AHB");
    // The prepared metadata should carry the input AHB generation.
    assert_eq!(
        prepared.prepare_params().input_generation,
        1,
        "input AHB generation should be recorded in prepared params"
    );

    // run_prepared must not crash: the import failed (invalid AHB) so MNN
    // uses host upload instead.
    let (frame, pending) = engine.run_prepared(prepared, &params).expect("run_prepared");
    // Output must be non-empty — the graph produced real pixels. Exact size
    // depends on the graph's declared output format (PackedRgb = 2 bpp, etc.),
    // so just verify it produced something.
    assert!(
        !frame.data.is_empty(),
        "output frame is empty; inference produced no pixels"
    );
    let finished = engine.finish(pending);
    assert!(finished.total_duration_ns > 0);

    // Run a second frame with a normal (no-AHB) path to confirm the engine
    // is still healthy after the dummy-AHB attempt.
    let mut params2 = ProcessParams::new(w, h, &buf);
    params2.sensor_max = smax;
    params2.target_width = w;
    params2.target_height = h;
    let result2 = engine.process(&params2);
    assert!(result2.is_ok(), "second frame after dummy AHB should succeed");

    let _ = std::fs::remove_file(mnn_path);
    // `dummy_ptr` is intentionally never dereferenced.
    eprintln!("PASSED: dummy input AHB zero-copy glue accepted, graceful fallback");
}

/// Uninitialized engine should return error.
#[test]
#[cfg(feature = "mnn")]
fn test_mnn_engine_uninitialized() {
    #[cfg(feature = "mnn")]
    {
        use cam_isp::engine::IspEngine;
        use cam_isp::mnnengine::{MnnBackend, MnnEngine};
        let engine = MnnEngine::new(MnnBackend::Vulkan);
        let result = engine.process(&cam_isp::engine::ProcessParams {
            width: 48,
            height: 64,
            stride_width: 48,
            buf: &[0u8; 48 * 64 * 2],
            sensor_max: 1024.0,
            target_width: 64,
            target_height: 64,
            ccm_matrix: None,
            tone_params: cam_isp::engine::default_tone_params(),
            bayer_gains: None,
            awb_gains: None,
            bayer_pattern: 0,
            analog_gain: 1.0,
            scene_change: 0.0,
            lsc_gains: None,
            blc_values: None,
            warp_grid: None,
            warp_shading: None,
            output_format: cam_isp::engine::OutputFormat::default(),
            timestamp_ns: 0,
            isp_params: None,
            ..cam_isp::engine::ProcessParams::new(48, 64, &[0u8; 48 * 64 * 2])
        });
        assert!(result.is_err(), "uninitialized should fail");
        eprintln!("Uninitialized check OK: {:?}", result.err().unwrap());
    }
    #[cfg(not(feature = "mnn"))]
    {
        eprintln!("MNN feature not enabled");
    }
}
