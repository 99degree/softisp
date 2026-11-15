//! Probe what MNN's converter actually does to a declared INT16 graph input.
//!
//! The pipeline can build a native 16-bit head (`RawInput16Block` +
//! `UnpackCfaBlock::NativeInt16`), which declares `[1,1,H,W]` INT16 — one
//! 16-bit sample per Bayer pixel, no packed tensor, no host-side lane split.
//!
//! Whether that survives conversion decides how much the engine has to move.
//! If the converter keeps INT16 the graph input is 2 bytes per pixel; if it
//! widens to FLOAT32 it is 4. The engine adapts either way (`RawFloat`), but
//! the difference at 5 Mpx is 19 MB per session per frame.
//!
//! Run:
//!   cd cam-rust
//!   cargo test -p cam-isp --test probe_input16_head --features mnn \
//!     -- --nocapture --ignored

#![cfg(feature = "mnn")]

// `select_engine_by_name` hands back a `Box<dyn IspEngine>`, so `build` and
// `as_any` resolve through the trait object's vtable — the trait itself does
// not need to be imported here.

#[test]
#[ignore]
fn probe_input16_head_conversion() {
    cam_isp::init();

    let (w, h) = (1920u32, 1080u32);

    for (label, force16) in [
        ("packed i32 head (current)", false),
        ("native i16 head", true),
    ] {
        let mut profile = cam_isp::profile::PipelineProfile::MED;
        if force16 {
            profile = profile.with_force_input16();
        }

        let mut blocks = profile.build_blocks(w, h, 0);
        let head = blocks.remove(0);
        let aux = profile.build_aux_blocks(h as i64, w as i64);
        blocks.extend(aux);

        let mut engine =
            cam_isp::engine::select_engine_by_name("mnn_vulkan").expect("mnn_vulkan engine");

        if let Err(e) = engine.build(head, blocks, None, 21) {
            eprintln!("{label:<28} BUILD FAILED: {e}");
            continue;
        }

        let typed = engine
            .as_any()
            .downcast_ref::<cam_isp::mnnengine::MnnEngine>()
            .and_then(|e| e.model_input_type());

        match typed {
            Some((code, bits)) => {
                let kind = match code {
                    0 => "int",
                    1 => "uint",
                    2 => "float",
                    _ => "other",
                };
                let bytes = if bits == 16 { 2 } else { 4 };
                eprintln!(
                    "{label:<28} graph input: {kind}{bits} -> {bytes} B/pixel ({} MB at {w}x{h})",
                    (w as usize * h as usize * bytes) / 1_048_576
                );
            }
            None => eprintln!("{label:<28} graph input type unknown"),
        }
    }
}
