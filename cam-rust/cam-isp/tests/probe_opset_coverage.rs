//! Which stages are still on the primitive opset, and why.
//!
//! `isp.*` emission is opt-in per block: a block has to grow both a
//! `BlockOpsetMode::MnnIspMin` branch and a `custom_op_types()` declaration
//! before the profile can select it. This lists the MED chain's stages and marks
//! which are stuck on the primitive path, with the reason each is stuck.
//!
//! The point is to keep the answer current. "Which stages still use primitive
//! ops" was previously answered from memory, and the list had drifted.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_opset_coverage --features mnn -- --nocapture

use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

/// The `isp.*` op a block can emit, if any.
///
/// Asked of the block, not of its id: a profile that turns a stage off emits an
/// `IdentityBlock` under the same id, and an Identity has no fused form. Matching
/// ids would credit "ldci" with fusion capability in a chain where `use_ldci` is
/// false and the block is a pass-through.
///
/// `custom_op_types()` is the block's own declaration, and it is the same signal
/// `MnnEngine::chain_has_fused_op` uses to decide whether fp16 storage is safe.
fn fused_capable(b: &dyn IspBlock) -> Option<String> {
    b.custom_op_types()
        .and_then(|ops| ops.first())
        .map(|op| (*op).to_string())
}

#[test]
fn which_stages_are_primitive() {
    // MED is the profile the app runs.
    let blocks = PipelineProfile::MED.build_blocks(2592, 1940, 2);

    let mut fused = Vec::new();
    let mut primitive = Vec::new();

    for b in &blocks {
        let id = b.id().to_string();
        match fused_capable(b.as_ref()) {
            Some(op) => fused.push((id, op)),
            None => primitive.push(id),
        }
    }

    eprintln!("\nMED chain: {} blocks\n", blocks.len());
    eprintln!("can emit a fused isp.* op ({}):", fused.len());
    for (id, op) in &fused {
        let mode = blocks
            .iter()
            .find(|b| b.id() == id)
            .map(|b| format!("{:?}", b.opset_mode()))
            .unwrap_or_default();
        eprintln!("  {id:<16} -> {op:<18} emits {mode} today");
    }

    eprintln!("\nprimitive only ({}):", primitive.len());
    for id in &primitive {
        eprintln!("  {id}");
    }

    // The stages most worth fusing next, because MNN already has a working
    // shader for them and they are per-pixel work over the whole frame.
    eprintln!(
        "\nReady to implement but not yet emitted by softisp:\n  \
         isp.normalize   (implemented shader, W/H from binding 0)\n  \
         isp.cfa         (implemented, planar raw -> planar quad)\n  \
         isp.blc         (implemented, per-channel black level at binding 5)\n  \
         isp.bayer_wb    (implemented, quad-planar gains)\n  \
         isp.tone        (implemented, gamma + contrast)\n  \
         isp.saturation  (implemented)\n  \
         isp.demosaic_g2 (implemented, quality demosaic)\n  \
         isp.gamma       (implemented)\n  \
         isp.dpc         (implemented, bad-pixel correction)"
    );
    eprintln!(
        "\nNot covered by any MNN shader:\n  \
         isp.unpack_blc  (absent from IspOpsetValidate + IspSpvLookup; the fused\n  \
                          block's Custom mode emits op_type `unpack`, which the\n  \
                          runtime then rejects as an unknown isp.* name)"
    );
}
