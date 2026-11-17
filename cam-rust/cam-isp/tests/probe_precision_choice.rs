//! Does an unfused chain keep fp16 storage?
//!
//! The engine drops to fp32 whenever the chain contains a fused `isp.*` op —
//! their shaders declare `float[]` buffers and fp16 would hand them halves. The
//! test that decided "contains a fused op" used to be `opset_mode()`, which
//! defaults to `Custom` for most blocks, so *every* chain looked fused and the
//! app silently lost fp16. Four fp32 sessions at 2592x1940 do not fit, and the
//! device process died mid-frame after a multi-second stall in session creation.
//!
//! This checks the decision itself (`chain_has_fused_op`) rather than the
//! resulting tensor precision: building two engines to read their precision back
//! is what the first version of this test did, and it reliably crashed inside
//! MNN's pipeline construction on this device.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_precision_choice --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::blocks::*;
use cam_isp::mnnengine::chain_has_fused_op;
use cam_isp::pipeline::IspBlock;

/// A chain shaped like the app's, with one fusion-capable stage controllable.
fn chain(display_mode: i32) -> Vec<Box<dyn IspBlock>> {
    vec![
        Box::new(
            RawInputPackedBlock::new()
                .with_concrete_width(256)
                .with_concrete_height(192),
        ),
        Box::new(UnpackBlock::new().with_concrete_width(256)),
        Box::new(NormalizeBlock::new()),
        Box::new(CfaBlockPacked::new()),
        Box::new(DemosaicBlock::new(2)),
        Box::new(CcmBlock::with_instance("ccm")),
        Box::new(
            DisplayBlock::new(256)
                .with_output_format(cam_isp::engine::OutputFormat::FloatRgb)
                .with_opset_mode(display_mode),
        ),
    ]
}

#[test]
#[ignore]
fn precision_follows_the_chain() {
    cam_isp::init();

    // What the app builds today: every fusion-capable stage on its primitive
    // form. `opset_mode()` alone would say "fused" for most of these blocks,
    // which is the bug this function exists to avoid.
    let prim = chain(0);
    let prim_fused = chain_has_fused_op(&prim);
    eprintln!("primitive chain: has_fused={prim_fused}");
    assert!(
        !prim_fused,
        "an all-primitive chain would be forced to fp32 storage, and four fp32 \
         sessions at 2592x1940 do not fit on this device"
    );

    // A block that really does emit `isp.display`.
    let fused = chain(2);
    let fused_fused = chain_has_fused_op(&fused);
    eprintln!("fused chain: has_fused={fused_fused}");
    assert!(
        fused_fused,
        "a chain containing `isp.display` must be detected — its shader declares \
         float buffers and fp16 storage would hand it halves"
    );

    eprintln!("\nprecision decision follows the chain");
}
