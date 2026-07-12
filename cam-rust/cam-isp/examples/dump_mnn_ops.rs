use cam_isp::engine::OutputFormat;
use cam_isp::mnn_converter::convert_onnx_to_mnn;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;

fn main() {
    let profiles = [
        ("LITE", PipelineProfile::LITE),
        ("HEAVY", PipelineProfile::HEAVY),
    ];

    let base = "/data/data/com.termux/files/home/softisp/cam-rust";

    for (prof_name, mut profile) in profiles {
        profile.output_format = OutputFormat::FloatRgb;
        let mut blocks = profile.build_blocks(1280, 0);
        GraphComposer::wire_blocks(&mut blocks);
        let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let onnx = GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

        let onnx_path = format!("{}/{}_1280.onnx", base, prof_name);
        let mnn_path = format!("{}/{}_1280.mnn", base, prof_name);
        std::fs::write(&onnx_path, &onnx).unwrap();

        let result = convert_onnx_to_mnn(&onnx_path, &mnn_path, None);

        match result {
            Ok(msg) => {
                eprintln!("=== {} Profile ===", prof_name);
                eprintln!("  ONNX: {} bytes", onnx.len());
                eprintln!("  Convert: {}", msg);
            }
            Err(e) => {
                eprintln!("=== {} Profile ===", prof_name);
                eprintln!("  Convert FAILED: {}", e);
            }
        }
    }
}
