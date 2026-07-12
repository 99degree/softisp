use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    let profile = PipelineProfile::HEAVY;
    let blocks = profile.build_blocks(1920, 0);
    println!("HEAVY blocks ({}):", blocks.len());
    for b in &blocks {
        println!(
            "  id={:<20} frame_tensor={:?} graph_output={:?} output_tensors={:?}",
            b.id(),
            b.frame_tensor(),
            b.graph_output_name(),
            b.output_tensors()
        );
    }
}
