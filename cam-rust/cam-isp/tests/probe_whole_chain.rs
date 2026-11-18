//! What does the whole-pipeline profile actually build?
//!
//! Run:
//!   cargo test -p cam-isp --test probe_whole_chain --features mnn -- --nocapture

use cam_isp::pipeline::IspBlock;
use cam_isp::profile::{FuseStages, PipelineProfile};

#[test]
fn dump_whole_chain() {
    cam_isp::init();

    let mut whole = PipelineProfile::MED;
    whole.fuse_stages = FuseStages {
        whole_pipeline: true,
        ..FuseStages::NONE
    };
    whole.opset_mode = 2;

    let blocks = whole.build_blocks(2592, 1940, 2);
    eprintln!("\nWHOLE chain: {} blocks", blocks.len());
    for (i, b) in blocks.iter().enumerate() {
        eprintln!(
            "  [{i}] {:<20} ops={:?} inputs={:?} outs={:?}",
            b.id(),
            b.custom_op_types(),
            b.input_tensors(),
            b.output_tensors()
        );
    }

    // The node's own inputs, in order — this is the op's binding contract.
    for b in blocks.iter().skip(1) {
        for n in b.nodes() {
            let s = String::from_utf8_lossy(&n);
            if s.contains("isp.pipeline") {
                eprintln!("\nnode text (truncated):\n{}", &s[..s.len().min(400)]);
            }
        }
    }
}
