/// Minimal test: Cast(INT32/UINT32) → MNN → check output type.
use cam_isp::mnn_sys::{MnnInterpreterSafe, MnnBackendType, mnn_run_true_zero_copy};

fn build_cast_model(to_type: i64, elem_type: i32) -> Vec<u8> {
    use cam_isp::onnx::proto::Proto;
    let inp = Proto::value_info("input", &[
        Proto::tensor_dim_value(1),
        Proto::tensor_dim_value(3),
        Proto::tensor_dim_value(4),
        Proto::tensor_dim_value(4),
    ], 1);

    let out = Proto::value_info("output", &[
        Proto::tensor_dim_value(1),
        Proto::tensor_dim_value(3),
        Proto::tensor_dim_value(4),
        Proto::tensor_dim_value(4),
    ], elem_type);

    let cast_node = Proto::node("Cast", &["input"], &["output"], &[
        Proto::attribute_int("to", to_type),
    ]);

    let graph = Proto::graph("test_cast", &[cast_node], &[inp], &[out], &[], &[]);
    Proto::model(11, &Proto::opset("ai.onnx", 11), "cast_test", &graph)
}

fn test_cast(to_type: i64, elem_type: i32, label: &str) {
    println!("\n=== Cast(to={}, elem_type={}) {}", to_type, elem_type, label);
    let onnx = build_cast_model(to_type, elem_type);
    let onnx_path = "ct.onnx";
    let mnn_path = "ct.mnn";
    std::fs::write(onnx_path, &onnx).unwrap();
    println!("ONNX: {} bytes", onnx.len());

    match cam_isp::mnn_converter::convert_onnx_to_mnn(onnx_path, mnn_path, None) {
        Ok(_) => println!("Convert OK"),
        Err(e) => { println!("Convert FAILED: {}", e); return; }
    }

    let mnn = std::fs::read(mnn_path).unwrap();
    let interp = MnnInterpreterSafe::from_buffer(&mnn).expect("interp");
    let sess = interp.create_session(MnnBackendType::Cpu, 1).expect("session");
    println!("Session OK, backend=CPU");

    // Input [1.0, 2.0, ..., 48.0]
    let inp: Vec<f32> = (1..=48).map(|i| i as f32).collect();
    let inp_shape = [1i32, 3, 4, 4];
    let mut out_f32 = vec![0.0f32; 48 + 16];
    let n = unsafe {
        mnn_run_true_zero_copy(
            interp.as_ptr(),
            sess.as_ptr(),
            inp.as_ptr() as *const std::ffi::c_void,
            2,     // halide_type_float = 2
            32,    // bits = 32
            inp_shape.as_ptr(),
            inp_shape.len() as i32,
            out_f32.as_mut_ptr(),
            out_f32.len() as i32,
        )
    };
    println!("Elements returned: {}", n);

    // Check output - reinterpret as bytes
    let out_bytes = unsafe {
        std::slice::from_raw_parts(out_f32.as_ptr() as *const u8, n as usize * 4)
    };

    print!("Bytes[0..16]: ");
    for i in 0..16.min(out_bytes.len()) { print!(" {:02x}", out_bytes[i]); }
    println!();

    // Check as i32
    let i32s = unsafe { std::slice::from_raw_parts(out_f32.as_ptr() as *const i32, n as usize) };
    println!("As i32[0..8]: {:?}", &i32s[..8.min(n as usize)]);
    println!("As f32[0..8]: {:?}", &out_f32[..8.min(n as usize)]);

    // Get data type from output tensor
    if let Some(t) = interp.get_first_output(&sess) {
        let dt = t.data_type();
        let shape = t.shape();
        println!("Shape: {:?}, dtype={}", shape, dt);
        if dt == 5 { println!("→ INT32 ✅"); }
        else if dt == 6 { println!("→ UINT32 ✅"); }
        else if dt == 4 { println!("→ FLOAT32 (Cast removed by MNN) ❌"); }
        else { println!("→ Unknown dtype={}", dt); }
    }
}

fn main() {
    test_cast(6, 6, "INT32");    // ONNX type 6
    test_cast(12, 12, "UINT32"); // ONNX type 12
}
