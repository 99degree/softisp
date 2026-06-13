//! Quick script to generate HEAVY profile ONNX model and test MNN conversion.
use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;
use cam_isp::pipeline::IspBlock;

fn main() {
    let blocks = PipelineProfile::HEAVY.build_blocks(8, 2);
    println!("HEAVY profile: {} blocks", blocks.len());
    for b in &blocks {
        println!("  {}", b.id());
    }
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let model = GraphComposer::compose_from_vec(&block_refs, &[], 16).unwrap();
    println!("ONNX model: {} bytes", model.len());
    std::fs::write("heavy_profile.onnx", &model).unwrap();
    println!("Written to heavy_profile.onnx");
    
    // Try MNN conversion
    use std::process::Command;
    let out = Command::new("MNNConvert")
        .args(&["-f", "ONNX", "--modelFile", "heavy_profile.onnx", "--MNNModel", "heavy_profile.mnn", "--bizCode", "test"])
        .output();
    match out {
        Ok(o) => {
            print!("{}", String::from_utf8_lossy(&o.stdout));
            eprint!("{}", String::from_utf8_lossy(&o.stderr));
            if o.status.success() {
                println!("MNNConvert: SUCCESS");
                let meta = std::fs::metadata("heavy_profile.mnn").unwrap();
                println!("MNN model: {} bytes", meta.len());
            } else {
                println!("MNNConvert: FAILED (exit code: {:?})", o.status.code());
            }
        }
        Err(e) => println!("MNNConvert: ERROR: {}", e),
    }
}
