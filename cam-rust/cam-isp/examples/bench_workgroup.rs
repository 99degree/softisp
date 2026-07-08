use cam_isp::profile::PipelineProfile;
use cam_isp::engine::{OutputFormat, ProcessParams};

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    
    let profiles = [
        ("LITE", PipelineProfile::LITE),
        ("MED", PipelineProfile::MED),
        ("HEAVY", PipelineProfile::HEAVY),
    ];
    
    for (prof_name, mut profile) in profiles {
        profile.output_format = OutputFormat::FloatRgb;
        
        for &(w, h, label) in &[(1280, 720, "HD"), (1920, 1080, "FHD"), (3840, 2160, "4K")] {
            let mut blocks = profile.build_blocks(w, 0);
            cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
            let block_refs: Vec<&dyn cam_isp::pipeline::IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
            let _onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
            let mut block_vec = blocks;
            let head = block_vec.remove(0);
            let aux = block_vec;
            let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
            engine.build(head, aux, None, 21).unwrap();
            
            let raw: Vec<u8> = vec![128; (w as usize * h as usize * 2) as usize];
            let params = ProcessParams::new(w, h, &raw);
            
            // Warmup
            for _ in 0..20 {
                let _ = engine.process(&params);
            }
            
            // Timed runs
            let iters = if w * h > 2_000_000 { 50 } else { 200 };
            let start = std::time::Instant::now();
            for _ in 0..iters {
                let _ = engine.process(&params);
            }
            let elapsed_us = start.elapsed().as_micros() / iters as u128;
            
            println!("{:<7} {:<4} {:>5}us ({:>5} FPS) [{}x{}]", 
                prof_name, label, elapsed_us, 1_000_000 / elapsed_us.max(1), w, h);
        }
    }
}
