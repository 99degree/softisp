/// Regression test: Conv packing with coefficients [65536, 256, 1]
/// Input: R,G,B (f32), Conv(1x1) with weights [65536,256,1]
/// Expected: R*65536 + G*256 + B (single f32)
use cam_isp::onnx::proto::Proto;
use cam_isp::mnn_sys::{MnnInterpreterSafe, MnnBackendType};

fn build_conv_pack_model() -> Vec<u8> {
    let inp = Proto::value_info("input",
        &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3)],
        1); // FLOAT
    let out = Proto::value_info("output",
        &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(1)],
        1); // FLOAT
    // Conv weight: out_ch=1, in_ch=3, kernel=1x1
    let conv_w = Proto::tensor_proto_float(
        &"pack_weights".to_string(),
        &[1, 3, 1, 1],
        &[65536.0f32, 256.0, 1.0]
    );
    let nodes = vec![
        Proto::node("Conv", &["input", "pack_weights", "bias"],
            &["output"],
            &[
                Proto::attribute_ints("kernel_shape", &[1, 1]),
                Proto::attribute_ints("strides", &[1, 1]),
                Proto::attribute_ints("pads", &[0, 0]),
            ]
        )
    ];
    let graph = Proto::graph("conv_pack", &nodes, &[inp], &[out], &[conv_w], &[]);
    Proto::model(11, &Proto::opset("ai.onnx", 11), "conv_pack", &graph)
}

fn main() {
    println!("=== Test: Conv packing with [65536, 256, 1] ===");
    let onnx = build_conv_pack_model();
    let onnx_path = "test_conv.onnx";
    let mnn_path = "test_conv.mnn";
    std::fs::write(onnx_path, &onnx).unwrap();

    if let Err(e) = cam_isp::mnn_converter::convert_onnx_to_mnn(onnx_path, mnn_path, None) {
        println!("Convert error: {}", e);
        return;
    }

    let mnn = std::fs::read(mnn_path).unwrap();
    let interp = MnnInterpreterSafe::from_buffer(&mnn).expect("interp");
    let sess = interp.create_session(MnnBackendType::Cpu, 1).expect("sess");

    // Input: R,G,B values
    let inp_values = [25.0f32, 51.0, 76.0];
    let expected = 25.0 * 65536.0 + 51.0 * 256.0 + 76.0;

    let mut inp_bytes = Vec::with_capacity(12);
    for &f in &inp_values { inp_bytes.extend_from_slice(&f.to_le_bytes()); }
    let inp_shape = [1, 3];

    let mut out_buf = vec![0u8; 4];
    let n = unsafe {
        cam_isp::mnn_sys::mnn_run_true_zero_copy(
            interp.as_ptr(), sess.as_ptr(),
            inp_bytes.as_ptr() as *const std::ffi::c_void,
            2, 32,
            inp_shape.as_ptr(), inp_shape.len() as i32,
            out_buf.as_mut_ptr() as *mut f32,
            1
        )
    };

    let out_f32 = unsafe { std::slice::from_raw_parts(out_buf.as_ptr() as *const f32, n as usize) };

    println!("Input RGB: {:?}", inp_values);
    println!("Expected: {} = 0x{:x}", expected, expected as u32);
    println!("Output:   {} = 0x{:x}", out_f32[0], out_f32[0] as u32);
    println!("Diff: {}", (out_f32[0] - expected).abs());

    if (out_f32[0] - expected).abs() < 0.5 {
        println!("=> PASS");
    } else {
        println!("=> FAIL");
    }
}
