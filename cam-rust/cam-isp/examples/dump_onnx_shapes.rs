use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

fn main() {
    let mut blocks = PipelineProfile::HEAVY.build_blocks(1920, 0);
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut blocks);
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();
    
    // Find DisplayBlock/frame in the ONNX data
    let data = &onnx;
    let target = b"DisplayBlock/frame";
    let mut pos = 0;
    while let Some(idx) = data[pos..].windows(target.len()).position(|w| w == target) {
        let abs = pos + idx;
        println!("Found 'DisplayBlock/frame' at offset {}", abs);
        // Print surrounding bytes for analysis
        let start = abs.saturating_sub(5);
        let end = (abs + target.len() + 50).min(data.len());
        let hex: Vec<String> = data[start..end].iter().map(|b| format!("{:02x}", b)).collect();
        println!("  hex: {}", hex.join(" "));
        pos = abs + 1;
    }
    
    // Also look for all occurrences
    for name in &["DisplayBlock/frame", "UnpackCfaBlock/frame", "DemosaicCcmBlock/frame"] {
        let mut pos = 0;
        while let Some(idx) = data[pos..].windows(name.len()).position(|w| w == name.as_bytes()) {
            let abs = pos + idx;
            // Check if this is inside a graph.output field (tag 0x5a = field 11, length-delimited)
            // Look backwards for the field tag
            let context_start = abs.saturating_sub(20);
            let context = &data[context_start..abs+name.len()+30];
            let hex: Vec<String> = context.iter().map(|b| format!("{:02x}", b)).collect();
            println!("{} at {}: ...{}...", name, abs, hex.join(" "));
            pos = abs + 1;
        }
    }
}
