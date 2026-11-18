//! What the FUSED chain is actually made of, and what could still merge.
//!
//! `FuseStages::ALL` fuses seven stages today. This prints the chain that
//! survives, so the next merge is chosen from what is *in the chain* rather than
//! from what happens to have a shader.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_fusion_map --features mnn -- --nocapture

use cam_isp::pipeline::{BlockOpsetMode, IspBlock};
use cam_isp::profile::{FuseStages, PipelineProfile};

fn dump(label: &str, mut p: PipelineProfile) {
    let blocks = p.build_blocks(1296, 970, 2);
    eprintln!("\n{label}: {} blocks\n", blocks.len());
    eprintln!("{:<3} {:<26} {:<9} {}", "#", "block", "emits", "notes");
    for (i, b) in blocks.iter().enumerate() {
        let ops = b.custom_op_types().unwrap_or(&[]);
        let emits = if b.is_noop() {
            "noop".to_string()
        } else if ops.is_empty() {
            "primitive".to_string()
        } else if b.opset_mode() == BlockOpsetMode::MnnIspMin {
            ops.join(",")
        } else {
            format!("{} (held primitive)", ops.join(","))
        };
        eprintln!("{i:<3} {:<26} {:<9}", b.id(), emits);
    }

    // Adjacent pairs that both still emit primitive nodes are the candidates:
    // a fused op is worth writing when it removes a full-frame round trip.
    let prim: Vec<&str> = blocks
        .iter()
        .filter(|b| !b.is_noop() && b.custom_op_types().is_none())
        .map(|b| b.id())
        .collect();
    eprintln!("\nstill primitive ({}): {}", prim.len(), prim.join(", "));
}

#[test]
fn fusion_map() {
    cam_isp::init();
    dump("MED", PipelineProfile::MED);

    let mut fused = PipelineProfile::FUSED;
    fused.fuse_stages = FuseStages::ALL;
    dump("FUSED (all verified stages)", fused);
}
