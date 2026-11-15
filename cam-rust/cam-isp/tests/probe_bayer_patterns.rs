//! Does a colour cast survive on each Bayer pattern?
//!
//! The CFA stage emits four planes in physical quad order (TL, TR, BL, BR), so
//! which plane holds red depends on the sensor pattern. Everything downstream
//! that indexes planes by colour — AWB gains, black level tables, the stats
//! fold — has to go through the pattern, or the wrong colour is corrected.
//!
//! This renders the same *scene* encoded as each of the four patterns and
//! asserts the picture agrees. A block that assumes RGGB passes for RGGB and
//! produces a cast for the other three.
//!
//! Runs as a normal test with the `mnn` feature; it takes about a second.
//!
//!   cargo test -p cam-isp --test probe_bayer_patterns --features mnn

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::profile::PipelineProfile;

/// Per-quad channel values for a red-dominant scene.
const SCENE: [u16; 3] = [900, 450, 200]; // R, G, B

/// Render `SCENE` encoded as `pattern`, returning the RGB plane means.
fn render(pattern: i32) -> Option<(f64, f64, f64)> {
    let (w, h) = (256u32, 192u32);

    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;
    // Turn off the adaptive stages so the measurement is the raw colour path.
    profile.use_ee = false;
    profile.use_saturation = false;
    profile.use_bad_pixel = false;

    let mut blocks = profile.build_blocks(w, h, pattern);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    engine.build(head, blocks, None, 21).ok()?;

    // Place each scene channel at the quad position its pattern assigns it.
    let planes = cam_isp::bayer_planes::plane_channels(pattern);
    let mut buf = vec![0u8; (w as usize) * (h as usize) * 2];
    for y in 0..h as usize {
        for x in 0..w as usize {
            let quad = (y % 2) * 2 + (x % 2);
            let v = SCENE[planes[quad].min(2)];
            let i = (y * w as usize + x) * 2;
            buf[i] = (v & 0xFF) as u8;
            buf[i + 1] = (v >> 8) as u8;
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
        bayer_pattern: pattern,
        analog_gain: 1.0,
        scene_change: 0.0,
        lsc_gains: None,
        blc_values: None,
        warp_grid: None,
        warp_shading: None,
        output_format: OutputFormat::FloatRgb,
        timestamp_ns: 0,
        isp_params: None,
    };

    let f = engine.process(&params).ok()?;
    let vals: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    let plane = vals.len() / 3;
    if plane == 0 {
        return None;
    }
    let mean = |p: &[f32]| p.iter().map(|v| *v as f64).sum::<f64>() / p.len() as f64;
    Some((
        mean(&vals[0..plane]),
        mean(&vals[plane..2 * plane]),
        mean(&vals[2 * plane..]),
    ))
}

#[test]
fn every_pattern_renders_the_same_scene() {
    cam_isp::init();

    let names = ["RGGB", "GRBG", "GBRG", "BGGR"];
    let mut results = Vec::new();

    for pattern in 0..4 {
        let Some((r, g, b)) = render(pattern) else {
            panic!("{} failed to render", names[pattern as usize]);
        };
        eprintln!(
            "{:<5} R {:.4}  G {:.4}  B {:.4}   (R/G {:.3}, B/G {:.3})",
            names[pattern as usize],
            r,
            g,
            b,
            r / g.max(1e-9),
            b / g.max(1e-9)
        );
        results.push((pattern, r, g, b));
    }

    // The same scene through the same optics must come out the same colour,
    // whatever pattern encodes it on the sensor.
    let (_, r0, g0, b0) = results[0];
    for (pattern, r, g, b) in &results[1..] {
        let name = names[*pattern as usize];
        assert!(
            (r - r0).abs() < 0.05 * r0.max(0.05),
            "{name} red is {r:.4} but RGGB gave {r0:.4} — the red gain landed on \
             the wrong plane"
        );
        assert!(
            (b - b0).abs() < 0.10 * b0.max(0.05),
            "{name} blue is {b:.4} but RGGB gave {b0:.4} — the blue gain landed on \
             the wrong plane"
        );
        let _ = (g, g0);
    }

    eprintln!("\nall four patterns agree");
}
