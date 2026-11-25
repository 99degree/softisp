//! Does lens shading actually correct a vignetted frame?
//!
//! `use_lsc` used to insert a 4x4 identity `Conv` — a placeholder that multiplied
//! the Bayer quad by the identity matrix and changed nothing. It is now a
//! `VignettingBlock` with a 4-plane gain map applied to the pre-demosaic quad,
//! which is where lens falloff physically happens.
//!
//! The scene here is deliberately dark at the corners and bright in the centre.
//! A working correction must (a) run without error and (b) lift the corners
//! relative to the centre.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_lsc_post_cfa --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::profile::PipelineProfile;

/// Mean of a planar f32 frame's four corner patches vs a centre patch.
fn corner_vs_centre(data: &[u8], w: usize, h: usize) -> (f64, f64) {
    let vals: Vec<f32> = data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    let plane = vals.len() / 3;
    if plane < w * h {
        return (0.0, 0.0);
    }
    let patch = 16usize;
    let at = |x: usize, y: usize| -> f64 {
        let i = y * w + x;
        vals[i] as f64 + vals[plane + i] as f64 + vals[2 * plane + i] as f64
    };
    let mut corner = 0.0;
    for (ox, oy) in [
        (0, 0),
        (w - patch, 0),
        (0, h - patch),
        (w - patch, h - patch),
    ] {
        for dy in 0..patch {
            for dx in 0..patch {
                corner += at(ox + dx, oy + dy);
            }
        }
    }
    corner /= (patch * patch * 4) as f64;

    let mut centre = 0.0;
    for dy in 0..patch {
        for dx in 0..patch {
            centre += at(w / 2 + dx, h / 2 + dy);
        }
    }
    centre /= (patch * patch) as f64;
    (corner, centre)
}

fn render(lsc: bool) -> Option<Vec<u8>> {
    let (w, h) = (512u32, 384u32);

    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;
    profile.use_lsc = lsc;
    profile.lsc_strength = 0.6;
    // Pin a zero CCM so the comparison measures lens shading, not colour.
    profile.use_ee = false;
    profile.use_saturation = false;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    engine.build(head, blocks, None, 21).ok()?;

    // Vignetted scene: cos^4-ish falloff from the centre, peaks at the middle.
    let mut buf = vec![0u8; (w as usize) * (h as usize) * 2];
    let (cx, cy) = (w as f32 / 2.0, h as f32 / 2.0);
    let max_r = (cx * cx + cy * cy).sqrt();
    for y in 0..h as usize {
        for x in 0..w as usize {
            let dx = x as f32 - cx;
            let dy = y as f32 - cy;
            let r = (dx * dx + dy * dy).sqrt() / max_r;
            // Bright centre, ~40% at the corners.
            let level = (900.0 * (1.0 - 0.6 * r * r)) as u16;
            let i = (y * w as usize + x) * 2;
            buf[i] = (level & 0xFF) as u8;
            buf[i + 1] = (level >> 8) as u8;
        }
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
        output_format: OutputFormat::FloatRgb,
        timestamp_ns: 0,
        isp_params: None,
        input_buffer: None,
        output_buffer: None,
        input_ahb: None,
        output_ahb: None,
    };

    let f = engine.process(&params).ok()?;
    Some(f.data)
}

#[test]
#[ignore]
fn lsc_lifts_the_corners() {
    cam_isp::init();

    let (w, h) = ((512 / 2) as usize, (384 / 2) as usize);

    let off = render(false).expect("LSC-off chain failed");
    let on = render(true).expect("LSC-on chain failed");

    let (off_corner, off_centre) = corner_vs_centre(&off, w, h);
    let (on_corner, on_centre) = corner_vs_centre(&on, w, h);

    let off_ratio = off_corner / off_centre.max(1e-9);
    let on_ratio = on_corner / on_centre.max(1e-9);

    eprintln!("LSC off: corner {off_corner:.4}  centre {off_centre:.4}  ratio {off_ratio:.3}");
    eprintln!("LSC on : corner {on_corner:.4}  centre {on_centre:.4}  ratio {on_ratio:.3}");

    assert!(
        off_centre > 0.05,
        "the scene rendered black, nothing to compare"
    );
    assert!(
        off_ratio < 0.85,
        "the test scene is not vignetted (corner/centre {off_ratio:.3}); \
         the correction would have nothing to do"
    );
    assert!(
        on_ratio > off_ratio,
        "LSC did not lift the corners: ratio {off_ratio:.3} -> {on_ratio:.3}"
    );
    eprintln!("corner/centre improved {off_ratio:.3} -> {on_ratio:.3}");
}
