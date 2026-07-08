use cam_isp::profile::PipelineProfile;
use cam_isp::engine::{OutputFormat, ProcessParams};

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    
    let profiles = [
        ("LITE", PipelineProfile::LITE),
        ("HEAVY", PipelineProfile::HEAVY),
    ];
    
    for (prof_name, mut profile) in profiles {
        profile.output_format = OutputFormat::FloatRgb;
        
        // Build at HD
        let mut blocks = profile.build_blocks(1280, 0);
        cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
        let block_refs: Vec<&dyn cam_isp::pipeline::IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let _onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
        let mut block_vec = blocks;
        let head = block_vec.remove(0);
        let aux = block_vec;
        let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
        engine.build(head, aux, None, 21).unwrap();
        
        let raw: Vec<u8> = vec![128; (1280 * 720 * 2) as usize];
        let params = ProcessParams::new(1280, 720, &raw);
        
        // Warmup
        for _ in 0..10 {
            let _ = engine.process(&params);
        }
        
        // Timed runs
        let start = std::time::Instant::now();
        for _ in 0..100 {
            let _ = engine.process(&params);
        }
        let elapsed_us = start.elapsed().as_micros() / 100;
        
        println!("{:<7} HD: {:>4}us/frame ({:>4} FPS)", prof_name, elapsed_us, 1_000_000 / elapsed_us.max(1));
    }
}
