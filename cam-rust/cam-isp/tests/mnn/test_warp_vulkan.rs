#![cfg(feature = "mnn")]
//! End-to-end test: GridSampler warp on Vulkan backend.
//!
//! Verifies that standard ONNX GridSampler runs natively on Vulkan
//! without any custom ISP op — proving the "generic ONNX + auto-detect"
//! architecture works for warp/stabilization.
//!
//! Run:
//!   cd cam-rust
//!   LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
//!     cargo test --test test_warp_vulkan --features mnn -- --ignored --nocapture --test-threads=1

use std::path::Path;
use std::process::Command;

/// Generate a GridSampler warp test model using Python.
fn generate_warp_onnx(output_path: &str, w: u32, h: u32) -> Result<(), String> {
    let script = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("vulkan_isp")
        .join("gen_warp_test.py");

    if !script.exists() {
        return Err(format!("Script not found: {}", script.display()));
    }

    let status = Command::new("python3")
        .arg(script.to_str().unwrap())
        .arg("--width")
        .arg(w.to_string())
        .arg("--height")
        .arg(h.to_string())
        .arg("-o")
        .arg(output_path)
        .status()
        .map_err(|e| format!("python3 failed: {}", e))?;

    if !status.success() {
        return Err(format!("Script exited {}", status));
    }
    Ok(())
}

/// Convert ONNX → MNN.
fn convert(onnx_path: &str, mnn_path: &str) -> Result<(), String> {
    cam_isp::mnn_converter::convert_onnx_to_mnn(onnx_path, mnn_path, None)?;
    Ok(())
}

/// Run inference using mnn_run_host_tensors and return output pixel at center.
fn run_warp(mnn_path: &str) -> Result<(f32, f32, f32), String> {
    use cam_isp::mnn_sys::*;
    use std::ffi::CString;

    let c_path = CString::new(mnn_path).map_err(|_| "NUL")?;
    let interp = unsafe { mnn_interpreter_create_from_file(c_path.as_ptr()) };
    if interp.is_null() {
        return Err("no model".into());
    }

    let session = unsafe { mnn_session_create(interp, MnnBackendType::Vulkan, 1) };
    if session.is_null() {
        unsafe {
            mnn_interpreter_destroy(interp);
        }
        return Err("no session".into());
    }

    let input = unsafe { mnn_session_get_input_v2(interp, session, std::ptr::null()) };
    if input.is_null() {
        unsafe {
            mnn_session_release(interp, session);
            mnn_interpreter_destroy(interp);
        }
        return Err("no input".into());
    }

    let mut dims = [0i32; 8];
    let ndim = unsafe { mnn_tensor_get_shape(input, dims.as_mut_ptr(), 8) };
    let (c, h, w) = if ndim >= 4 {
        (
            dims[1].max(1) as usize,
            dims[2].max(1) as usize,
            dims[3].max(1) as usize,
        )
    } else {
        unsafe {
            mnn_session_release(interp, session);
            mnn_interpreter_destroy(interp);
        }
        return Err(format!("unexpected ndim={}", ndim));
    };

    // Fill input: R=1.0, G=0.5, B=0.25
    let total = h * w;
    let mut input_data = vec![0.0f32; c * total];
    for i in 0..total {
        input_data[0 * total + i] = 1.0;
    }
    for i in 0..total {
        input_data[1 * total + i] = 0.5;
    }
    for i in 0..total {
        input_data[2 * total + i] = 0.25;
    }

    let max_out = c * total * 2;
    let mut output_data = vec![0.0f32; max_out];

    let in_shape: [i32; 4] = [1, c as i32, h as i32, w as i32];
    let n_out = unsafe {
        mnn_run_host_tensors(
            interp,
            session,
            input_data.as_ptr(),
            in_shape.as_ptr(),
            4,
            output_data.as_mut_ptr(),
            max_out as i32,
        )
    };

    if n_out < 0 {
        unsafe {
            mnn_session_release(interp, session);
            mnn_interpreter_destroy(interp);
        }
        return Err(format!("mnn_run_host_tensors failed: {}", n_out));
    }

    let out_bytes = (n_out as usize) * std::mem::size_of::<f32>();
    let out_floats = out_bytes / std::mem::size_of::<f32>();
    let out_total = out_floats / 3; // assume 3 channels

    // Sample center pixel
    let cx = w / 2;
    let cy = h / 2;
    let plane = w * h;
    let idx = cy * w + cx;

    let r = if out_total > idx {
        output_data[idx]
    } else {
        -1.0
    };
    let g = if out_total + plane > idx {
        output_data[plane + idx]
    } else {
        -1.0
    };
    let b = if out_total + 2 * plane > idx {
        output_data[2 * plane + idx]
    } else {
        -1.0
    };

    unsafe {
        mnn_session_release(interp, session);
        mnn_interpreter_destroy(interp);
    }
    Ok((r, g, b))
}

// ═══════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn test_warp_identity_16x16() {
    let onnx = format!("{}/.warp_test_16.onnx", env!("CARGO_MANIFEST_DIR"));
    let mnn = format!("{}/.warp_test_16.mnn", env!("CARGO_MANIFEST_DIR"));

    generate_warp_onnx(&onnx, 16, 16).expect("generate ONNX");
    convert(&onnx, &mnn).expect("convert ONNX→MNN");

    let (r, g, b) = run_warp(&mnn).expect("run inference");

    let eps = 0.01;
    assert!((r - 1.0).abs() < eps, "R: expected 1.0, got {}", r);
    assert!((g - 0.5).abs() < eps, "G: expected 0.5, got {}", g);
    assert!((b - 0.25).abs() < eps, "B: expected 0.25, got {}", b);

    println!("✓ PASS: identity warp 16×16 on Vulkan");
    println!("  R={:.4} G={:.4} B={:.4}", r, g, b);

    let _ = std::fs::remove_file(&onnx);
    let _ = std::fs::remove_file(&mnn);
}

#[test]
#[ignore]
fn test_warp_identity_64x64() {
    let onnx = format!("{}/.warp_test_64.onnx", env!("CARGO_MANIFEST_DIR"));
    let mnn = format!("{}/.warp_test_64.mnn", env!("CARGO_MANIFEST_DIR"));

    generate_warp_onnx(&onnx, 64, 64).expect("generate ONNX");
    convert(&onnx, &mnn).expect("convert ONNX→MNN");

    let (r, g, b) = run_warp(&mnn).expect("run inference");

    let eps = 0.01;
    assert!((r - 1.0).abs() < eps, "R: expected 1.0, got {}", r);
    assert!((g - 0.5).abs() < eps, "G: expected 0.5, got {}", g);
    assert!((b - 0.25).abs() < eps, "B: expected 0.25, got {}", b);

    println!("✓ PASS: identity warp 64×64 on Vulkan");
    println!("  R={:.4} G={:.4} B={:.4}", r, g, b);

    let _ = std::fs::remove_file(&onnx);
    let _ = std::fs::remove_file(&mnn);
}
