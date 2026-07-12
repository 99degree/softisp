use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

fn main() {
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    let mut profile = PipelineProfile::LITE;
    profile.output_format = OutputFormat::Argb;
    let mut blocks = profile.build_blocks(1920, 0);
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let _onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    engine.build(head, aux, None, 21).unwrap();

    let raw: Vec<u8> = vec![128u8; (1920 * 1080 * 2) as usize];
    let params = ProcessParams::new(1920, 1080, &raw);
    let frame = engine.process(&params).unwrap();

    println!(
        "Output: {}×{} data={} bytes",
        frame.width,
        frame.height,
        frame.data.len()
    );

    // Check ARGB packing: each pixel should be 4 bytes (ARGB)
    // But we're using INT32 packing, so each pixel is 4 bytes
    let expected = 1920 * 1080 * 4; // ARGB8888: 4 bytes per pixel
    println!("Expected: {} bytes (ARGB8888)", expected);

    // Sample first few pixels
    for i in 0..5 {
        let offset = i * 4;
        if offset + 4 <= frame.data.len() {
            let a = frame.data[offset];
            let r = frame.data[offset + 1];
            let g = frame.data[offset + 2];
            let b = frame.data[offset + 3];
            println!("Pixel {}: A={} R={} G={} B={}", i, a, r, g, b);
        }
    }
}
