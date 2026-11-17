//! How many of the fused pipeline's graph nodes do no work at all?
//!
//! `PipelineProfile::build_blocks` ends with
//! `GraphComposer::wire_blocks_with_identities`, which inserts a
//! `BridgeIdentityBlock` between *every* adjacent pair of blocks. Those emit a
//! real ONNX node:
//!
//!     Transpose(x, perm = [0,1,2,3])
//!
//! Semantically a no-op — and on the GPU a full-frame read + write. The builder
//! also emits `IdentityBlock`s for stages a profile has turned off (`lsc`,
//! `bilateral`, `vignetting`, `colorspace`, …), and those are no-ops too.
//!
//! This counts both, so the cost of the no-ops is a number rather than an
//! assumption.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_noop_nodes --features mnn -- --nocapture

use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::{FuseStages, PipelineProfile};

/// Count occurrences of an ONNX node type in a serialized graph.
fn count_op(graph: &[u8], op_type: &str) -> usize {
    // NodeProto field 4 is `op_type`, a length-delimited string.
    let mut needle = vec![(4u8 << 3) | 2];
    let n = op_type.len();
    if n < 128 {
        needle.push(n as u8);
    } else {
        needle.push(((n & 0x7F) | 0x80) as u8);
        needle.push((n >> 7) as u8);
    }
    needle.extend_from_slice(op_type.as_bytes());
    graph
        .windows(needle.len())
        .filter(|w| *w == needle.as_slice())
        .count()
}

fn describe(label: &str, profile: PipelineProfile) {
    let blocks = profile.build_blocks(2592, 1940, 2);
    let total_blocks = blocks.len();
    let refs: Vec<&dyn cam_isp::pipeline::IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let aux: Vec<Box<dyn cam_isp::pipeline::IspBlock>> = profile.build_aux_blocks(1940, 2592);
    let aux_refs: Vec<&dyn cam_isp::pipeline::IspBlock> = aux.iter().map(|b| b.as_ref()).collect();

    let Ok(graph) = GraphComposer::compose_from_vec(&refs, &aux_refs, 21) else {
        eprintln!("{label}: compose failed");
        return;
    };

    let transpose = count_op(&graph, "Transpose");
    let identity = count_op(&graph, "Identity");
    let conv = count_op(&graph, "Conv");
    let total = transpose + identity + conv;

    eprintln!(
        "{label:<12} blocks={total_blocks:<3} transpose={transpose:<3} identity={identity:<3} \
         conv={conv:<3}  no-ops={} of {} = {:.0}%",
        transpose + identity,
        total,
        100.0 * (transpose + identity) as f64 / total.max(1) as f64
    );
}

#[test]
fn noop_node_share() {
    cam_isp::init();
    eprintln!();

    describe("MED", PipelineProfile::MED);

    let mut fused = PipelineProfile::FUSED;
    fused.fuse_stages = FuseStages::ALL;
    describe("FUSED", fused);
}
