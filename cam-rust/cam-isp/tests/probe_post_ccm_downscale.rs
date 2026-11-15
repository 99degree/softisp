//! Does the post-CCM downscale build *and* run?
//!
//! `post_ccm_downscale` is expressed in **RGB** coordinates: demosaic halves
//! both edges, so a 2592x1940 Bayer frame reaches the tail as 1296x970 RGB.
//! This drives the MED profile at that sensor size and rotates through the
//! whole session pool, which is what the live preview does.
//!
//! History: the block used to declare its Resize `scales` as a *runtime* graph
//! input. MNN reads that tensor's contents during `resizeSession`
//! (`InterpComputer` → `scalePtr[i]`), so `createSession` got a null host
//! pointer and failed. `scales` is an initializer now.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_post_ccm_downscale --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{OutputFormat, ProcessParams};

fn run_case(ds: (u32, u32)) -> Result<(u32, u32, usize), String> {
    let (w, h) = (2592u32, 1940u32);

    let mut profile = cam_isp::profile::PipelineProfile::MED;
    profile.post_ccm_downscale = ds;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").expect("mnn_vulkan");
    engine
        .build(head, blocks, None, 21)
        .map_err(|e| e.to_string())?;

    let mut buf = vec![0u8; (w * h * 2) as usize];
    for (i, chunk) in buf.chunks_exact_mut(2).enumerate() {
        let v = ((i % 251) as u16) * 13;
        chunk[0] = (v & 0xFF) as u8;
        chunk[1] = (v >> 8) as u8;
    }

    let params = ProcessParams {
        width: w,
        height: h,
        stride_width: w,
        buf: &buf,
        sensor_max: 1023.0,
        target_width: w,
        target_height: h,
        ccm_matrix: None,
        tone_params: cam_isp::engine::default_tone_params(),
        bayer_gains: None,
        awb_gains: None,
        bayer_pattern: 2,
        analog_gain: 1.0,
        scene_change: 0.0,
        lsc_gains: None,
        blc_values: None,
        warp_grid: None,
        warp_shading: None,
        output_format: OutputFormat::Bgra,
        timestamp_ns: 0,
        isp_params: None,
    };

    let mut last = (0u32, 0u32, 0usize);
    for frame in 0..8 {
        match engine.process(&params) {
            Ok(f) => last = (f.width, f.height, f.data.len()),
            Err(e) => return Err(format!("frame {frame}: {e}")),
        }
    }
    Ok(last)
}

#[test]
#[ignore]
fn post_ccm_downscale_inference() {
    cam_isp::init();

    // Dims are RGB. 2592x1940 Bayer -> 1296x970 RGB, so:
    //   off / (1296,970)  no resample inserted
    //   (648,485)         exact half of the RGB frame
    //   (800,599)         arbitrary ratio (exercises the floor arithmetic)
    for (label, ds) in [
        ("off", (0u32, 0u32)),
        ("(1296,970) no change", (1296u32, 970u32)),
        ("(648,485) exact half", (648u32, 485u32)),
        ("(800,599) arbitrary", (800u32, 599u32)),
    ] {
        match run_case(ds) {
            Ok((w, h, bytes)) => {
                eprintln!("{label:<22} 8 frames OK, output {w}x{h} ({bytes} bytes)")
            }
            Err(e) => eprintln!("{label:<22} FAILED: {e}"),
        }
    }
}
