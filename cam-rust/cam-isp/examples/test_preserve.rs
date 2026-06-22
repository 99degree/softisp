//! Smoke test for preserveInputType on a minimal INT16 ONNX model.

fn main() {
    use cam_isp::mnn_converter::{convert_onnx_to_mnn, MnnConvertOptions};
    use cam_isp::mnn_sys::{mnn_get_model_input_type, MnnBackendType, MnnInterpreterSafe};
    use cam_isp::onnx::proto::Proto;

    let input_name = "input";
    let output_name = "output";
    let shape = vec![
        Proto::tensor_dim_value(1),
        Proto::tensor_dim_value(1),
        Proto::tensor_dim_value(4),
        Proto::tensor_dim_value(4),
    ];

    let node = Proto::node("Identity", &[input_name], &[output_name], &[]);
    let input = Proto::value_info(input_name, &shape, 5); // INT16
    let output = Proto::value_info(output_name, &shape, 1); // FLOAT
    let graph = Proto::graph("preserve_input_type", &[node], &[input], &[output], &[], &[]);
    let opset = Proto::opset("", 21);
    let model = Proto::model(11, &opset, "test_preserve", &graph);

    std::fs::write("_test_int16.onnx", &model).unwrap();
    println!("ONNX model written ({} bytes)", model.len());

    convert_onnx_to_mnn("_test_int16.onnx", "_test_int16_default.mnn", None).unwrap();

    let opts = MnnConvertOptions { preserve_input_type: true, ..Default::default() };
    convert_onnx_to_mnn("_test_int16.onnx", "_test_int16_preserved.mnn", Some(&opts)).unwrap();

    for (label, path) in [
        ("default (no flag)", "_test_int16_default.mnn"),
        ("preserved (flag)", "_test_int16_preserved.mnn"),
    ] {
        if let Some(interp) = MnnInterpreterSafe::from_file(path) {
            if let Some(sess) = interp.create_session(MnnBackendType::Cpu, 1) {
                let mut code = 0i32;
                let mut bits = 0i32;
                unsafe {
                    mnn_get_model_input_type(interp.as_ptr(), sess.as_ptr(), &mut code, &mut bits);
                }
                println!("  {}: code={}, bits={}", label, code, bits);
            }
        }
    }

    std::fs::remove_file("_test_int16.onnx").ok();
    std::fs::remove_file("_test_int16_default.mnn").ok();
    std::fs::remove_file("_test_int16_preserved.mnn").ok();
}
