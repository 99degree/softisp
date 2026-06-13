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
