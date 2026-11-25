//! Count the ONNX op types in the fused 4K graph, and which of them the Vulkan
//! backend can actually run.
//!
//! The answer decides whether the statistics chain is GPU work or a round trip:
//! any op without a Vulkan creator runs on the CPU backup backend, and MNN then
//! copies the tensor GPU→host→GPU around it.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_op_inventory --features mnn -- --nocapture

use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;
use std::collections::BTreeMap;

fn count_ops(graph: &[u8]) -> BTreeMap<String, usize> {
    // NodeProto field 4 is `op_type`, a length-delimited string. Scan for the
    // length-delimited encoding that follows the field tag.
    let mut counts = BTreeMap::new();
    for op in [
        "AveragePool",
        "Conv",
        "ReduceMean",
        "ReduceMin",
        "ReduceMax",
        "ReduceSum",
        "Greater",
        "Less",
        "Size",
        "Concat",
        "Sub",
        "Mul",
        "Add",
        "Div",
        "Max",
        "Min",
        "Clip",
    ] {
        let mut needle = vec![(4u8 << 3) | 2, op.len() as u8];
        // Longest name here is 11 bytes, so the single-byte length is enough.
        needle[1] = op.len() as u8;
        needle.extend_from_slice(op.as_bytes());
        let n = graph
            .windows(needle.len())
            .filter(|w| *w == needle.as_slice())
            .count();
        if n > 0 {
            counts.insert(op.to_string(), n);
        }
    }
    counts
}

#[test]
fn op_inventory() {
    cam_isp::init();

    for (label, p) in [
        ("FUSED (stats full)", PipelineProfile::FUSED),
        ("MED", PipelineProfile::MED),
    ] {
        let mut blocks = p.build_blocks(3840, 2160, 2);
        let head = blocks.remove(0);
        let aux = p.build_aux_blocks(2160, 3840);
        let refs: Vec<&dyn cam_isp::pipeline::IspBlock> = std::iter::once(head.as_ref())
            .chain(blocks.iter().map(|b| b.as_ref()))
            .chain(aux.iter().map(|b| b.as_ref()))
            .collect();

        let graph = match GraphComposer::compose_from_vec(&refs, &[], 21) {
            Ok(g) => g,
            Err(e) => {
                eprintln!("{label}: compose failed: {e}");
                continue;
            }
        };
        let counts = count_ops(&graph);
        let total: usize = counts.values().sum();
        eprintln!("\n{label}: {total} counted nodes");
        for (op, n) in &counts {
            eprintln!("  {op:<12} {n}");
        }
    }
}
