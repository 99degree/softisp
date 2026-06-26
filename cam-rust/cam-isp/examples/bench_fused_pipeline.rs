//! Benchmark 4K fused ISP pipeline (ONNX → MNN → Vulkan).
//!
//! Measures GPU-only inference time for the fused 3-dispatch pipeline:
//!   unpack_demosaic → fcs → display → ee_ldci
//!
//! Run:
//!   cd cam-rust
//!   LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
//!     cargo run --example bench_fused_pipeline -p cam-isp --features mnn

use std::time::Instant;

fn main() {
    let _ = env_logger::builder().is_test(false).filter_level(log::LevelFilter::Warn).try_init();

    // Test configurations: (label, bayer_w, bayer_h)
    let configs: [(&str, u32, u32); 3] = [
        ("HD  1280×720",   1280, 720),
        ("FHD 1920×1080",  1920, 1080),
        ("4K  3840×2160",  3840, 2160),
    ];

    for &(label, bw, bh) in &configs {
        let prefix = format!(".bench_{}x{}", bw, bh);
        let onnx_path = format!("{}.onnx", prefix);
        let mnn_path  = format!("{}.mnn", prefix);

        // 1. Generate ONNX
        let script = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent().unwrap().parent().unwrap()
            .join("vulkan_isp").join("gen_isp_onnx_standard.py");
        let status = std::process::Command::new("python3")
            .arg(script.to_str().unwrap())
            .arg("--bayer-width").arg(bw.to_string())
            .arg("--bayer-height").arg(bh.to_string())
            .arg("-o").arg(&onnx_path)
            .status()
            .expect("python3 failed");
        if !status.success() { eprintln!("  ONNX gen failed"); continue; }

        // 2. Convert to MNN
        if let Err(e) = cam_isp::mnn_converter::convert_onnx_to_mnn(&onnx_path, &mnn_path, None) {
            eprintln!("  MNN convert failed: {}", e);
            let _ = std::fs::remove_file(&onnx_path);
            continue;
        }

        // 3. Run on Vulkan and measure
        use std::ffi::CString;
        use cam_isp::mnn_sys::*;

        let c_path = CString::new(mnn_path.as_str()).unwrap();
        let interp = unsafe { mnn_interpreter_create_from_file(c_path.as_ptr()) };
        if interp.is_null() { eprintln!("  Failed to load model"); continue; }

        let session = unsafe { mnn_session_create(interp, MnnBackendType::Vulkan, 1) };
        if session.is_null() { eprintln!("  Failed to create session"); continue; }

        // Get input dimensions
        let in_tensor = unsafe { mnn_session_get_input_v2(interp, session, std::ptr::null()) };
        let mut dims = [0i32; 4];
        unsafe { mnn_tensor_get_shape(in_tensor, dims.as_mut_ptr(), 4); }
        let (in_c, in_h, in_w) = (dims[1] as usize, dims[2] as usize, dims[3] as usize);
        let total = in_c * in_h * in_w;

        // Output dimensions
        let out_tensor = unsafe { mnn_session_get_output_v2(interp, session, std::ptr::null()) };
        let mut out_dims = [0i32; 4];
        unsafe { mnn_tensor_get_shape(out_tensor, out_dims.as_mut_ptr(), 4); }
        let (out_c, out_h, out_w) = (out_dims[1] as usize, out_dims[2] as usize, out_dims[3] as usize);

        // Create test input: Bayer checkerboard
        let mut input = vec![0.0f32; total];
        for y in 0..in_h {
            for x in 0..in_w {
                let idx = y * in_w + x;
                input[idx] = if y % 2 == 0 {
                    if x % 2 == 0 { 100.0 } else { 200.0 }
                } else {
                    if x % 2 == 0 { 200.0 } else { 50.0 }
                };
            }
        }

        let max_out = out_c * out_h * out_w;
        let mut output = vec![0.0f32; max_out];
        let shape = [1, in_c as i32, in_h as i32, in_w as i32];

        // Warmup
        for _ in 0..5 {
            unsafe {
                mnn_run_host_tensors(interp, session, input.as_ptr(), shape.as_ptr(), 4,
                                     output.as_mut_ptr(), max_out as i32);
            }
        }

        // Benchmark GPU-only (the first call already set up pipelines)
        let n_iters = 100;
        let start = Instant::now();
        for _ in 0..n_iters {
            unsafe {
                mnn_run_host_tensors(interp, session, input.as_ptr(), shape.as_ptr(), 4,
                                     output.as_mut_ptr(), max_out as i32);
            }
        }
        let elapsed = start.elapsed().as_secs_f64() * 1000.0 / n_iters as f64;
        let fps = 1000.0 / elapsed;

        // Verify
        let ch_stride = out_h * out_w;
        let (r, g, b) = (output[0], output[ch_stride], output[2 * ch_stride]);
        let ok = (r - 0.345).abs() < 0.01 && (g - 0.479).abs() < 0.01 && (b - 0.245).abs() < 0.01;

        println!("{:14} → {:3}×{:3} | {:7.2} ms = {:5.1} FPS ({:3} iters) | {}",
            label, out_w, out_h, elapsed, fps, n_iters, if ok { "✓" } else { "✗" });

        unsafe { mnn_session_release(interp, session); mnn_interpreter_destroy(interp); }

        // Cleanup
        let _ = std::fs::remove_file(&onnx_path);
        let _ = std::fs::remove_file(&mnn_path);
    }
}
