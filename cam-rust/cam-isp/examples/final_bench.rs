use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::profile::PipelineProfile;

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    let sizes = [("HD", 1280, 720), ("FHD", 1920, 1080), ("4K", 3840, 2160)];

    let profiles = [
        ("LITE", PipelineProfile::LITE),
        ("MED", PipelineProfile::MED),
        ("HEAVY", PipelineProfile::HEAVY),
        ("UNIFIED", PipelineProfile::UNIFIED),
    ];

    for (prof_name, mut profile) in profiles {
        profile.output_format = OutputFormat::FloatRgb;
        println!("\n=== {} Profile ===", prof_name);

        for (size_name, w, h) in sizes {
            // Benchmark
            println!("  Building {} {} pipeline...", prof_name, size_name);
            let mut blocks = profile.build_blocks(w, 0);
            cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);

            let block_refs: Vec<&dyn cam_isp::pipeline::IspBlock> =
                blocks.iter().map(|b| b.as_ref()).collect();
            let onnx =
                cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

            let mut block_vec = blocks;
            let head = block_vec.remove(0);
            let aux = block_vec;

            let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
            engine.build(head, aux, None, 21).unwrap();

            // Test data
            let raw: Vec<u8> = vec![128; (w * h * 2) as usize];
            let params = ProcessParams::new(w, h, &raw);

            // Warmup
            for _ in 0..2 {
                let _ = engine.process(&params);
            }

            // Benchmark
            let start = std::time::Instant::now();
            let mut ok = false;
            for _ in 0..5 {
                if let Ok(frame) = engine.process(&params) {
                    if !frame.data.is_empty() {
                        ok = true;
                    }
                }
            }

            let elapsed_ms = start.elapsed().as_millis() / 5;
            let fps = if elapsed_ms > 0 { 1000 / elapsed_ms } else { 0 };

            println!(
                "  {:<4} {:>4}ms/frame {:>3} FPS data={}",
                size_name,
                elapsed_ms,
                fps,
                if ok { "✅" } else { "❌" }
            );
        }
    }
}
