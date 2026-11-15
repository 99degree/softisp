//! Does the post-CCM downscale survive ONNX→MNN conversion and session build?
//!
//! The block emits a `Resize` node whose `scales` tensor is a runtime graph
//! input, not a baked initializer. MNN's `InterpComputer` reads that tensor's
//! host data at shape-inference time — which is before the engine ever writes
//! a value — so this checks whether the session can be created at all.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_post_ccm_downscale --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

#[test]
#[ignore]
fn post_ccm_downscale_builds_a_session() {
    cam_isp::init();

    let (w, h) = (2592u32, 1940u32);

    for (label, ds) in [("off", (0u32, 0u32)), ("1920x1436", (1920u32, 1436u32))] {
        let mut profile = cam_isp::profile::PipelineProfile::MED;
        profile.post_ccm_downscale = ds;

        let mut blocks = profile.build_blocks(w, h, 2);
        let head = blocks.remove(0);
        let aux = profile.build_aux_blocks(h as i64, w as i64);
        blocks.extend(aux);

        let mut engine =
            cam_isp::engine::select_engine_by_name("mnn_vulkan").expect("mnn_vulkan engine");

        match engine.build(head, blocks, None, 21) {
            Ok(()) => eprintln!("{label:<12} build OK"),
            Err(e) => eprintln!("{label:<12} build FAILED: {e}"),
        }
    }
}
