use cam_isp::mnn_sys::{MnnBackendType, MnnInterpreterSafe};
/// Regression test: Verify Cast(INT32) preserves byte values
/// Input: [0.1, 0.2, 0.3] -> Mul(255) -> Cast(INT32)
/// Check that lower 8 bits of each INT32 output = expected 8-bit value
use cam_isp::onnx::proto::Proto;

fn build_cast_model_1d() -> Vec<u8> {
    let inp = Proto::value_info(
        "input",
        &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3)],
        1,
    ); // FLOAT
    let out = Proto::value_info(
        "output",
        &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3)],
        6,
    ); // INT32
    let scale = Proto::tensor_proto_float_scalar("scale", 255.0);
    let nodes = vec![
        Proto::node("Mul", &["input", "scale"], &["mul_out"], &[]),
        Proto::node(
            "Cast",
            &["mul_out"],
            &["output"],
            &[Proto::attribute_int("to", 6)],
        ),
    ];
    let graph = Proto::graph("cast_bytes", &nodes, &[inp], &[out], &[scale], &[]);
    Proto::model(11, &Proto::opset("ai.onnx", 11), "cast_bytes", &graph)
}

fn main() {
    println!("=== Test: Cast(INT32) preserves byte values ===");
    let onnx = build_cast_model_1d();
    let onnx_path = "test_cast_1d.onnx";
    let mnn_path = "test_cast_1d.mnn";
    std::fs::write(onnx_path, &onnx).unwrap();

    if let Err(e) = cam_isp::mnn_converter::convert_onnx_to_mnn(onnx_path, mnn_path, None) {
        println!("Convert error: {}", e);
        return;
    }

    let mnn = std::fs::read(mnn_path).unwrap();
    let interp = MnnInterpreterSafe::from_buffer(&mnn).expect("interp");
    let sess = interp.create_session(MnnBackendType::Cpu, 1).expect("sess");

    let inp_values = [0.1f32, 0.2, 0.3];
    let mut inp_bytes = Vec::with_capacity(12);
    for &f in &inp_values {
        inp_bytes.extend_from_slice(&f.to_le_bytes());
    }
    let inp_shape = [1, 3];

    let mut out_buf = vec![0u8; 12];
    let n = unsafe {
        cam_isp::mnn_sys::mnn_run_true_zero_copy(
            interp.as_ptr(),
            sess.as_ptr(),
            inp_bytes.as_ptr() as *const std::ffi::c_void,
            2,
            32,
            inp_shape.as_ptr(),
            inp_shape.len() as i32,
            out_buf.as_mut_ptr() as *mut f32,
            3,
        )
    };

    assert_eq!(n, 0, "mnn_run_true_zero_copy failed (returned {})", n);

    // Output was written directly into out_buf by zero-copy (3 INT32 values).
    let out_i32 = unsafe { std::slice::from_raw_parts(out_buf.as_ptr() as *const i32, 3) };
    let exp = [
        (inp_values[0] * 255.0).trunc(),
        (inp_values[1] * 255.0).trunc(),
        (inp_values[2] * 255.0).trunc(),
    ];

    println!("Input: {:?}", inp_values);
    println!("Expected f32: {:?}", exp);
    println!("Output INT32: {:?}", &out_i32[..3]);
    println!(
        "Lower 8 bits: [{}, {}, {}]",
        out_i32[0] & 0xff,
        out_i32[1] & 0xff,
        out_i32[2] & 0xff
    );
    println!(
        "Expected u8:   [{}, {}, {}]",
        exp[0] as u8, exp[1] as u8, exp[2] as u8
    );

    if (out_i32[0] & 0xff) == exp[0] as u8 as i32
        && (out_i32[1] & 0xff) == exp[1] as u8 as i32
        && (out_i32[2] & 0xff) == exp[2] as u8 as i32
    {
        println!("=> PASS");
    } else {
        println!("=> FAIL");
    }
}
