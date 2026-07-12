use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;

fn main() {
    // Enable CPU fallback and MNN/Vulkan backend
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    // Test UNIFIED profile with 4K Bayer → FHD ARGB8888
    let input_dimensions = [("4K", 3840, 2160)];
    let output_dimensions = [("FHD", 1920, 1080)];

    // Use UNIFIED profile with ARGB8888 output format
    let mut profile = PipelineProfile::UNIFIED;
    profile.output_format = OutputFormat::Argb;

    // Build pipeline for 4K → FHD downscale
    let mut blocks = profile.build_blocks(1920, 0); // FHD as reference, auto-scaled
    GraphComposer::wire_blocks(&mut blocks);

    let block_refs: Vec<&dyn cam_isp::pipeline::IspBlock> =
        blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;

    // Create MNN/Vulkan engine
    let mut engine = match cam_isp::engine::select_engine_by_name("mnn_vulkan") {
        Some(e) => e,
        None => {
            eprintln!(
                "Failed to select Vulkan engine: {}, falling back",
                "no engine"
            );
            cam_isp::engine::select_engine_by_name("mnn_cpu").unwrap()
        }
    };

    engine.build(head, aux, None, 21).unwrap();

    for (input_name, input_w, input_h) in input_dimensions {
        for (output_name, output_w, output_h) in output_dimensions {
            println!(
                "
=== Testing {} {} → {} {} with {} profile ===",
                input_name, input_w, output_name, output_h, profile.label
            );

            // Create synthetic 4K Bayer test pattern
            let raw: Vec<u8> = vec![128; (input_w * input_h * 2) as usize];

            // Process with auto downscaling
            let mut params = ProcessParams::new(input_w, input_h, &raw);
            params.target_width = output_w;
            params.target_height = output_h;

            let start = std::time::Instant::now();
            match engine.process(&params) {
                Ok(frame) => {
                    let elapsed_ms = start.elapsed().as_millis();
                    println!(
                        "✅ Success: {}ms ({} FPS)",
                        elapsed_ms,
                        1000 / elapsed_ms.max(1)
                    );
                    println!("   Raw input:  {}×{} (Bayer)", input_w, input_h);
                    println!(
                        "   Output:     {}×{} {:?} ({} bytes)",
                        frame.width,
                        frame.height,
                        frame.format,
                        frame.data.len()
                    );

                    // Verify output:
                    // ARGB8888 packed as 4B/pixel (1920×1080×4 = 8,294,400 bytes)
                    let expected_bytes = output_w * output_h * 4;
                    if frame.data.len() == expected_bytes as usize {
                        println!(
                            "✅ Size correct: {} = {}×{}×4",
                            frame.data.len(),
                            output_w,
                            output_h
                        );

                        // Sample first 4 pixels
                        let pixels = &frame.data[..16];
                        for i in 0..4 {
                            let a = pixels[i * 4 + 0];
                            let r = pixels[i * 4 + 1];
                            let g = pixels[i * 4 + 2];
                            let b = pixels[i * 4 + 3];
                            println!("   Pixel {}: A={:3} R={:3} G={:3} B={:3}", i, a, r, g, b);
                        }
                    } else {
                        println!(
                            "❌ Size mismatch: got {} bytes, expected {} bytes ({}×{})",
                            frame.data.len(),
                            expected_bytes,
                            output_w,
                            output_h
                        );
                    }
                }
                Err(err) => {
                    println!("❌ Error: {:?}", err);
                }
            }
        }
    }
}
