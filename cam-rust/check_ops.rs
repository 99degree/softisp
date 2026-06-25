fn main() {
    use cam_isp::profile::PipelineProfile;
    use cam_isp::pipeline::GraphComposer;
    use cam_isp::mnn_converter::convert_onnx_to_mnn;
    use cam_isp::mnn_sys::{MnnBackendType, MnnInterpreterSafe};
    
    let w = 48u32;
    let h = 64u32;
    let blocks = PipelineProfile::HEAVY.build_blocks(h, 0, 0);
    let mut pipeline: Vec<Box<dyn cam_isp::pipeline::IspBlock>> = blocks;
    pipeline.push(Box::new(cam_isp::blocks::DisplayBlock::new(w)));
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut pipeline);
    let refs: Vec<&dyn cam_isp::pipeline::IspBlock> = pipeline.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");
    
    let onnx = ".tmp_check_ops.onnx".to_string();
    let mnn = ".tmp_check_ops.mnn".to_string();
    std::fs::write(&onnx, &model).unwrap();
    convert_onnx_to_mnn(&onnx, &mnn, None).unwrap();
    
    println!("Input shape: [1,1,{},{}]", h, w/2);
    println!("HEAVY pipeline ops (from ONNX graph):");
    
    let interp = MnnInterpreterSafe::from_file(&mnn).unwrap();
    if let Some(info) = interp.get_model_info() {
        println!("Model: {} ops total", info.op_count);
        for op in &info.ops {
            println!("  {}: {}", op.op_type, op.name);
        }
    } else {
        println!("(get_model_info not available, listing ONNX nodes instead)");
        // Fallback: list nodes from ONNX protobuf
        let proto = prost::Message::decode(&model[..]);
        println!("  {:?}", proto);
    }
    
    let _ = std::fs::remove_file(&onnx);
    let _ = std::fs::remove_file(&mnn);
}
