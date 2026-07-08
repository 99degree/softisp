use cam_isp::pipeline::IspBlock;
use cam_isp::engine::{ProcessParams, OutputFormat};
use cam_isp::profile::PipelineProfile;

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    
    let formats = [
        ("PackedRgb", OutputFormat::PackedRgb),
        ("Argb", OutputFormat::Argb),
        ("FloatRgb", OutputFormat::FloatRgb),
    ];
    
    let profiles = [
        ("LITE", PipelineProfile::LITE),
        ("HEAVY", PipelineProfile::HEAVY),
    ];
    
    let sizes = [
        ("HD", 1280, 720),
        ("FHD", 1920, 1080),
        ("4K", 3840, 2160),
    ];
    
    for (prof_name, mut profile) in profiles {
        for (fmt_name, fmt) in formats {
            profile.output_format = fmt;
            let mut blocks = profile.build_blocks(sizes[1].1, 0); // FHD
            cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
            let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
            let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
            let mut block_vec = blocks;
            let head = block_vec.remove(0);
            let aux = block_vec;
            let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
            engine.build(head, aux, None, 21).unwrap();
            
            for (sz_name, w, h) in sizes {
                let raw: Vec<u8> = vec![128u8; (w * h * 2) as usize];
                let params = ProcessParams::new(w, h, &raw);
                let start = std::time::Instant::now();
                let mut ok = false;
                for _ in 0..5 {
                    if let Ok(f) = engine.process(&params) {
                        if f.data.len() > 0 {
                            ok = true;
                        }
                    }
                }
                let elapsed = start.elapsed().as_millis() / 5;
                let fps = if elapsed > 0 { 1000 / elapsed } else { 0 };
                println!("{} {} {}: {}ms/frame ({} FPS) data={}", 
                    prof_name, fmt_name, sz_name, elapsed, fps, 
                    if ok { "OK" } else { "FAIL" });
            }
        }
    }
}
