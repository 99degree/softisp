#![cfg(feature = "mnn")]

#[test]
#[ignore]
fn test_mnn_per_block_profiling() {
    // This test now validates the full pipeline using session inference memory data
    // instead of running each block individually.
    use cam_isp::blocks::DisplayBlock;
    use cam_isp::mnn_converter::convert_onnx_to_mnn;
    use cam_isp::mnn_sys::{MnnBackendType, MnnInterpreterSafe};
    use cam_isp::pipeline::GraphComposer;
    use cam_isp::profile::PipelineProfile;
    use std::ffi::CStr;
    use std::os::raw::c_void;

    let w = 48u32;
    let h = 64u32;

    // Build the full LITE pipeline.
    let lite_blocks = PipelineProfile::LITE.build_blocks(64, 0);
    let mut pipeline: Vec<Box<dyn cam_isp::pipeline::IspBlock>> = lite_blocks;
    // Ensure a display block at the end.
    pipeline.push(Box::new(DisplayBlock::new(w)));
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut pipeline);
    let refs: Vec<&dyn cam_isp::pipeline::IspBlock> = pipeline.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");

    // Write ONNX and convert to MNN.
    let onnx_path = ".tmp_full_test.onnx";
    let mnn_path = ".tmp_full_test.mnn";
    std::fs::write(onnx_path, &model).expect("write onnx");
    convert_onnx_to_mnn(onnx_path, mnn_path, None).expect("convert to mnn");
    let _ = std::fs::remove_file(onnx_path);

    // Load interpreter and create a session.
    let interp = MnnInterpreterSafe::from_file(mnn_path).expect("load MNN model");
    let sess = interp
        .create_session(MnnBackendType::Vulkan, 4)
        .expect("create session");

    // Synthetic raw input buffer (INT16 packed).
    let frame_size = (w * h * 2) as usize;
    let mut buf = vec![0u8; frame_size];
    for y in 0..h {
        for x in 0..w {
            let off = (y * w + x) as usize * 2;
            let val = ((x ^ y) & 0xFF) as u16;
            buf[off] = val as u8;
            buf[off + 1] = (val >> 8) as u8;
        }
    }

    // Input shape for the model.
    let shape = vec![1, 1, h as i32, w as i32];
    if let Some(t) = interp.get_first_input(&sess) {
        let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), &shape);
    }
    let _ = sess.resize();

    // Allocate output buffer.
    let max_out = (w * h * 4) as usize;
    let mut out_bytes = vec![0u8; max_out * 4];
    let out_ptr = out_bytes.as_mut_ptr() as *mut f32;

    // Run inference.
    unsafe {
        let _ = cam_isp::mnn_sys::mnn_run_with_output(
            interp.as_ptr(),
            sess.as_ptr(),
            std::ptr::null(),
            buf.as_ptr() as *const c_void,
            0,
            32,
            shape.as_ptr(),
            shape.len() as i32,
            CStr::from_bytes_with_nul_unchecked(b"DisplayBlock/frame\0").as_ptr(),
            out_ptr,
            max_out as i32,
        );
    }

    // Retrieve numeric session info (MEMORY, FLOPS, THREAD_NUMBER).
    unsafe {
        let mut mem: f32 = 0.0;
        let ret_mem = cam_isp::mnn_sys::mnn_get_model_info(
            interp.as_ptr(),
            sess.as_ptr(),
            cam_isp::mnn_sys::MnnModelInfo::MEMORY as i32,
            &mut mem as *mut _ as *mut c_void,
        );
        if ret_mem == 0 {
            println!("[perf] MNN MEMORY: {}", mem);
        }
        let mut flops: f32 = 0.0;
        let ret_flops = cam_isp::mnn_sys::mnn_get_model_info(
            interp.as_ptr(),
            sess.as_ptr(),
            cam_isp::mnn_sys::MnnModelInfo::FLOPS as i32,
            &mut flops as *mut _ as *mut c_void,
        );
        if ret_flops == 0 {
            println!("[perf] MNN FLOPS: {}", flops);
        }
        let mut threads: i32 = 0;
        let ret_threads = cam_isp::mnn_sys::mnn_get_model_info(
            interp.as_ptr(),
            sess.as_ptr(),
            cam_isp::mnn_sys::MnnModelInfo::THREAD_NUMBER as i32,
            &mut threads as *mut _ as *mut c_void,
        );
        if ret_threads == 0 {
            println!("[perf] MNN THREAD_NUMBER: {}", threads);
        }
    }

    // Clean up temporary files.
    let _ = std::fs::remove_file(mnn_path);
}
