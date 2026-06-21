//! Test preserveInputType flag by creating a simple ONNX model
//! with INT16 input and converting with/without the flag.
fn main() {
    use cam_isp::onnx::proto::Proto;
    use cam_isp::mnn_converter::{convert_onnx_to_mnn, MnnConvertOptions};
    use cam_isp::mnn_sys::{MnnInterpreterSafe, MnnBackendType};

    // Manually build ONNX: Input(INT16) → Identity → Output
    let input_name = "input";
    let id_out = "id_out";
    let output_name = "output";

    let mut graph = Vec::new();
    // GraphProto fields
    graph.extend(Proto::message(1, || {  // node[]
        Proto::message(1, || {  // NodeProto
            Proto::string(1, "Identity")   // op_type (field 1 for ONNX proto)
            + Proto::strings(2, &[input_name])  // inputs
            + Proto::strings(4, &[id_out])      // outputs (field 4 for MNN compat)
            + Proto::string(7, "identity")      // name
        })
    }));
    graph.extend(Proto::message(2, || {  // initializer[] (empty)
        vec![]
    }));
    graph.extend(Proto::message(4, || {  // input[]
        Proto::message(1, || {  // ValueInfoProto
            Proto::string(1, input_name)  // name
            + Proto::message(2, || {  // type
                Proto::message(1, || {  // TensorTypeProto
                    Proto::int32(1, 5)  // elem_type = INT16
                    + Proto::message(2, || { // shape
                        Proto::message(1, || { Proto::int64(1, 1) })  // dim[1]
                        + Proto::message(1, || { Proto::int64(1, 1) })
                        + Proto::message(1, || { Proto::int64(1, 4) })
                        + Proto::message(1, || { Proto::int64(1, 4) })
                    })
                })
            })
        })
    }));
    graph.extend(Proto::message(5, || {  // output[]
        Proto::message(1, || {  // ValueInfoProto
            Proto::string(1, output_name)
            + Proto::message(2, || {
                Proto::message(1, || {
                    Proto::int32(1, 1)  // FLOAT
                    + Proto::message(2, || {
                        Proto::message(1, || { Proto::int64(1, 1) })
                        + Proto::message(1, || { Proto::int64(1, 1) })
                        + Proto::message(1, || { Proto::int64(1, 4) })
                        + Proto::message(1, || { Proto::int64(1, 4) })
                    })
                })
            })
        })
    }));
    // opset
    graph.extend(Proto::message(8, || {
        Proto::message(1, || { Proto::int32(1, 21) })
    }));

    // Wrap in ModelProto
    let mut model = Vec::new();
    model.extend(Proto::int64(1, 0x10000000000001u64 as i64));  // ir_version
    model.extend(Proto::message(2, || {  // opset_import
        Proto::message(1, || { Proto::string(1, "") + Proto::int64(2, 21) })
    }));
    model.extend(Proto::string(3, "test"));  // producer_name
    model.extend(Proto::message(7, || graph));  // graph

    std::fs::write("_test_int16.onnx", &model).unwrap();
    println!("ONNX model written ({} bytes)", model.len());

    // Convert without preserveInputType
    convert_onnx_to_mnn("_test_int16.onnx", "_test_int16_default.mnn", None).unwrap();

    // Convert with preserveInputType
    let opts = MnnConvertOptions { preserve_input_type: true, ..Default::default() };
    convert_onnx_to_mnn("_test_int16.onnx", "_test_int16_preserved.mnn", Some(&opts)).unwrap();

    // Check input types
    for (label, path) in [
        ("default (no flag)", "_test_int16_default.mnn"),
        ("preserved (flag)", "_test_int16_preserved.mnn"),
    ] {
        if let Some(interp) = MnnInterpreterSafe::from_file(path) {
            if let Some(sess) = interp.create_session(MnnBackendType::Cpu, 1) {
                if let Some((code, bits)) = interp.get_input_type(&sess) {
                    println!("  {}: code={}, bits={}", label, code, bits);
                }
            }
        }
    }

    std::fs::remove_file("_test_int16.onnx").ok();
    std::fs::remove_file("_test_int16_default.mnn").ok();
    std::fs::remove_file("_test_int16_preserved.mnn").ok();
}
