use cam_isp::engine::OutputFormat;
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;
use cam_isp::engine::ProcessParams;

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    let w = 1920u32;
    let h = 1080u32;
    let profile = PipelineProfile::HEAVY;
    let blocks = profile.build_blocks(w, 0);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

    // Write ONNX for MNN inspection
    std::fs::write("/data/data/com.termux/files/home/softisp/cam-rust/heavy_debug.onnx", &onnx).unwrap();
    println!("Wrote heavy_debug.onnx ({} bytes)", onnx.len());

    // Convert to MNN
    let on = "heavy_debug.onnx";
    let mn = "heavy_debug.mnn";
    let opts = cam_isp::mnn_converter::MnnConvertOptions::default();
    cam_isp::mnn_converter::convert_onnx_to_mnn(on, mn, Some(&opts)).unwrap();
    println!("Converted to heavy_debug.mnn ({} bytes)", std::fs::metadata(mn).unwrap().len());

    // Load with MNN and inspect
    let interp = cam_isp::mnn::MnnInterpreterSafe::from_file(mn).unwrap();
    let sess = interp.create_session(cam_isp::mnn::MnnBackendType::Vulkan, 2).unwrap();

    // Check output tensors
    if let Some(first_out) = interp.get_first_output(&sess) {
        let shape = first_out.shape();
        println!("First output shape: {:?}", shape);
        let name = first_out.name().unwrap_or("unknown".to_string());
        println!("First output name: {}", name);
    } else {
        println!("No first output!");
    }

    // Check specific output
    if let Some(t) = interp.get_output(&sess, "DisplayBlock/frame") {
        let shape = t.shape();
        println!("DisplayBlock/frame shape: {:?}", shape);
    } else {
        println!("DisplayBlock/frame not found!");
    }
}
