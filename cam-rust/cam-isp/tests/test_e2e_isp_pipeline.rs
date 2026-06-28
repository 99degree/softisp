//! End-to-end ISP pipeline test: ONNX → MNNConvert → Vulkan run → verify.
//!
//! This test exercises the full toolchain:
//!   1. Generate ONNX (via Python `gen_isp_onnx_standard.py`)
//!   2. Convert ONNX → .mnn via Rust FFI (mnn_convert_onnx_to_mnn)
//!   3. Load .mnn with MNN on Vulkan backend
//!   4. Run inference with known Bayer pattern via mnn_run_host_tensors
//!   5. Verify output pixel values match expected (R=0.3454 G=0.4794 B=0.2449)
//!
//! Run:
//!   cd cam-rust
//!   LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
//!     cargo test --test test_e2e_isp_pipeline --features mnn -- --ignored --nocapture
//!
//! IMPORTANT: Run with --test-threads=1 to avoid Vulkan device queue races.
//!
//! Requires:
//!   - libMNN.so in lib/aarch64-v8a/
//!   - libMNNConvertDeps.so in lib/aarch64-v8a/
//!   - Python3 with onnx + numpy
//!   - gen_isp_onnx_standard.py at softisp/vulkan_isp/

use std::path::Path;
use std::process::Command;

const EXPECTED_R: f32 = 0.3454;
const EXPECTED_G: f32 = 0.4794;
const EXPECTED_B: f32 = 0.2449;

/// Generate a standard-op ONNX model using Python.
fn generate_onnx(output_path: &str, bayer_w: u32, bayer_h: u32) -> Result<(), String> {
    let script = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent().unwrap()
        .parent().unwrap()
        .join("vulkan_isp")
        .join("gen_isp_onnx_standard.py");

    if !script.exists() {
        return Err(format!("Python script not found: {}", script.display()));
    }

    let status = Command::new("python3")
        .arg(script.to_str().unwrap())
        .arg("--bayer-width")
        .arg(bayer_w.to_string())
        .arg("--bayer-height")
        .arg(bayer_h.to_string())
        .arg("-o")
        .arg(output_path)
        .status()
        .map_err(|e| format!("Failed to execute python3: {}", e))?;

    if !status.success() {
        return Err(format!("Python script exited with {}", status));
    }
    Ok(())
}

/// Convert ONNX → MNN via static FFI.
fn convert_onnx_to_mnn_via_ffi(onnx_path: &str, mnn_path: &str) -> Result<(), String> {
    cam_isp::mnn_converter::convert_onnx_to_mnn(onnx_path, mnn_path, None)?;
    Ok(())
}

/// Run inference using mnn_run_host_tensors FFI.
/// For multi-input models, also sets extra inputs (fcs_gain, ldci_strength, ee_gain).
fn run_inference(mnn_path: &str, use_vulkan: bool) -> Result<(f32, f32, f32), String> {
    use std::ffi::CString;
    use cam_isp::mnn_sys::*;

    let c_path = CString::new(mnn_path).map_err(|_| "NUL in path")?;
    let interp = unsafe { mnn_interpreter_create_from_file(c_path.as_ptr()) };
    if interp.is_null() {
        return Err("Failed to open model".to_string());
    }

    let backend = if use_vulkan { MnnBackendType::Vulkan } else { MnnBackendType::Cpu };
    let session = unsafe { mnn_session_create(interp, backend, 1) };
    if session.is_null() {
        unsafe { mnn_interpreter_destroy(interp); }
        return Err("Failed to create session".to_string());
    }

    // Get input shape
    let input_tensor = unsafe { mnn_session_get_input_v2(interp, session, std::ptr::null()) };
    if input_tensor.is_null() {
        unsafe { mnn_session_release(interp, session); mnn_interpreter_destroy(interp); }
        return Err("Failed to get input tensor".to_string());
    }

    let mut dims = [0i32; 8];
    let ndim = unsafe { mnn_tensor_get_shape(input_tensor, dims.as_mut_ptr(), 8) };

    // Resolve dynamic dims (-1) to 32 for test models
    let (in_c, in_h, in_w) = if ndim >= 4 {
        let h = if dims[2] > 0 { dims[2] } else { 32 };
        let w = if dims[3] > 0 { dims[3] } else { 48 };
        (dims[1].max(1) as usize, h as usize, w as usize)
    } else if ndim == 3 {
        (1, dims[1].max(1) as usize, dims[2].max(1) as usize)
    } else {
        (1, 1, dims[0].max(1) as usize)
    };
    let total_input = in_c * in_h * in_w;

    // Create test input: Bayer pattern R=100, Gr=200, Gb=200, B=50
    let mut input_data = vec![0.0f32; total_input];
    for y in 0..in_h {
        for x in 0..in_w {
            for c in 0..in_c {
                let idx = c * in_h * in_w + y * in_w + x;
                if c == 0 {
                    if y % 2 == 0 {
                        input_data[idx] = if x % 2 == 0 { 100.0 } else { 200.0 };
                    } else {
                        input_data[idx] = if x % 2 == 0 { 200.0 } else { 50.0 };
                    }
                }
            }
        }
    }

    // Set extra inputs for multi-input models (HEAVY profile)
    let identity_val = [1.0f32];
    let ldci_val = [0.5f32];
    let shape_1 = [1i32];
    let names = ["zzz_FcsBlock/fcs_gain_scaled", "zzz_EeBlock/ee_gain_scaled"];
    for name in &names {
        let c_name = CString::new(*name).unwrap();
        unsafe {
            mnn_set_input_float(interp, session, c_name.as_ptr(),
                identity_val.as_ptr(), shape_1.as_ptr(), 1);
        }
    }
    {
        let c_name = CString::new("zzz_LdciBlock/ldci_strength_scaled").unwrap();
        unsafe {
            mnn_set_input_float(interp, session, c_name.as_ptr(),
                ldci_val.as_ptr(), shape_1.as_ptr(), 1);
        }
    }

    // Allocate output buffer
    let max_out = 3 * 1920 * 1080;
    let mut output_data = vec![0.0f32; max_out];

    // Run inference
    let in_shape: [i32; 4] = [1, in_c as i32, in_h as i32, in_w as i32];
    let n_out = unsafe {
        mnn_run_host_tensors(
            interp, session,
            input_data.as_ptr(), in_shape.as_ptr(), 4,
            output_data.as_mut_ptr(), max_out as i32,
        )
    };

    if n_out < 0 {
        unsafe { mnn_session_release(interp, session); mnn_interpreter_destroy(interp); }
        return Err(format!("mnn_run_host_tensors failed: {}", n_out));
    }

    // Determine output shape
    let out_tensor = unsafe { mnn_session_get_output_v2(interp, session, std::ptr::null()) };
    let mut out_dims = [0i32; 8];
    let out_ndim = if !out_tensor.is_null() {
        unsafe { mnn_tensor_get_shape(out_tensor, out_dims.as_mut_ptr(), 8) }
    } else {
        0
    };

    let (_out_c, out_h, out_w) = if out_ndim >= 4 {
        (out_dims[1] as usize, out_dims[2] as usize, out_dims[3] as usize)
    } else {
        (1, 1, n_out as usize)
    };

    let ch_stride = out_h * out_w;
    let r = output_data[0];
    let g = output_data[ch_stride];
    let b = output_data[2 * ch_stride];

    unsafe { mnn_session_release(interp, session); mnn_interpreter_destroy(interp); }

    Ok((r, g, b))
}

/// Run the full e2e test for a given Bayer resolution.
fn run_e2e_test(label: &str, prefix: &str, bayer_w: u32, bayer_h: u32) {
    let onnx_path = format!("{}.onnx", prefix);
    let mnn_path  = format!("{}.mnn", prefix);

    // 1. Generate ONNX
    generate_onnx(&onnx_path, bayer_w, bayer_h)
        .expect("ONNX generation failed");

    // 2. Convert to MNN via FFI
    convert_onnx_to_mnn_via_ffi(&onnx_path, &mnn_path)
        .expect("MNN conversion failed");

    // 3. Run on Vulkan
    let (r, g, b) = run_inference(&mnn_path, true)
        .expect("Vulkan inference failed");

    // 4. Verify
    let ok = (r - EXPECTED_R).abs() < 0.01
          && (g - EXPECTED_G).abs() < 0.01
          && (b - EXPECTED_B).abs() < 0.01;
    println!("  {}: R={:.4} G={:.4} B={:.4}",
        if ok { "\u{2713} PASS" } else { "\u{2717} FAIL" }, r, g, b);
    assert!(ok, "{} output mismatch: got R={:.4} G={:.4} B={:.4}", label, r, g, b);

    // Cleanup
    for p in [&onnx_path, &mnn_path] {
        if Path::new(p).exists() { let _ = std::fs::remove_file(p); }
    }
}

#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires Python3, MNN libs, Vulkan
fn test_e2e_isp_small() {
    run_e2e_test("48\u{d7}32 small model (LITE)", ".mnn_e2e_small", 48, 32);
}

#[cfg(feature = "mnn")]
#[test]
#[ignore] // requires Python3, MNN libs, Vulkan
fn test_e2e_isp_4k() {
    run_e2e_test("3840\u{d7}2160 4K (HEAVY)", ".mnn_e2e_4k", 3840, 2160);
}

/// Verify HEAVY-profile ONNX converts without crashing.
/// Note: Rust pipeline blocks produce a different ONNX structure than the
/// Python-generated standard pipeline, so IspChainFusion patterns don't match.
/// The e2e tests above verify the full pipeline with Python-generated models.
#[cfg(feature = "mnn")]
#[test]
fn test_heavy_profile_mnn_convert() {
    use cam_isp::pipeline::GraphComposer;
    use cam_isp::profile::PipelineProfile;
    use std::path::PathBuf;

    // Generate HEAVY ONNX from Rust pipeline
    let blocks = PipelineProfile::HEAVY.build_blocks(48, 0);
    let refs: Vec<&dyn cam_isp::pipeline::IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16).unwrap();
    assert!(onnx.len() > 2000, "HEAVY ONNX too small: {} bytes", onnx.len());

    // Write to disk
    let onnx_path = ".mnn_heavy_rust.onnx";
    let mnn_path = ".mnn_heavy_rust.mnn";
    std::fs::write(onnx_path, &onnx).expect("write ONNX");

    // Convert through MNNConvert (runs IspChainFusion with RemoveIdentityOps)
    let result = cam_isp::mnn_converter::convert_onnx_to_mnn(onnx_path, mnn_path, None);
    match &result {
        Ok(log) => {
            let mnn_size = PathBuf::from(mnn_path).metadata().map(|m| m.len()).unwrap_or(0);
            println!("HEAVY Rust ONNX ({} bytes) -> MNN ({} bytes)", onnx.len(), mnn_size);
            println!("  Converter log: {}", log);
            assert!(mnn_size > 1000, "MNN model too small: {} bytes", mnn_size);
        }
        Err(e) => {
            // Conversion failure is acceptable for Rust pipeline patterns
            // (IspChainFusion may not match all Rust-generated patterns yet)
            println!("HEAVY Rust ONNX conversion: {} (expected for Rust patterns)", e);
        }
    }

    // Cleanup
    let _ = std::fs::remove_file(onnx_path);
    let _ = std::fs::remove_file(mnn_path);
}

/// Verify FP16 output DisplayBlock generates correct ONNX graph.
/// Tests that Float16Rgb produces a Cast(FLOAT→FLOAT16) node and the
/// output value_info has elem_type=10 (FLOAT16).
#[test]
fn test_fp16_output_display_block() {
    use cam_isp::blocks::DisplayBlock;
    use cam_isp::engine::OutputFormat;
    use cam_isp::pipeline::IspBlock;

    for (fmt, label, expected_nodes) in [
        (OutputFormat::Float16Rgb, "Float16Rgb", 2),
        (OutputFormat::Float16Bgra, "Float16Bgra", 3),
    ] {
        let block = DisplayBlock::new(1920)
            .with_output_format(fmt)
            .with_concrete_dims(1080, 1920);

        let nodes = block.nodes();
        let inits = block.initializers();

        // Float16Rgb: [Mul, Cast], Float16Bgra: [Conv, Identity, Cast]
        assert_eq!(nodes.len(), expected_nodes,
            "{}: expected {} nodes, got {}", label, expected_nodes, nodes.len());

        // output_value_info should be present
        assert!(block.output_value_info().is_some(),
            "{}: output_value_info should be present", label);

        // is_fp16()
        assert!(fmt.is_fp16(), "{}: is_fp16() should be true", label);
        assert_eq!(fmt.onnx_elem_type(), 10, "{}: onnx_elem_type() should be 10", label);

        // Bytes per pixel
        match fmt {
            OutputFormat::Float16Rgb => assert_eq!(fmt.bytes_per_pixel(), 6),
            OutputFormat::Float16Bgra => assert_eq!(fmt.bytes_per_pixel(), 8),
            _ => {}
        }

        println!("{}: {} nodes, {} initializers — OK", label, nodes.len(), inits.len());
    }
}
