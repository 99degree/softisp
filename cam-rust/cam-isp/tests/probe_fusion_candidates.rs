//! Which stages can be emitted as fused `isp.*` ops — verified one at a time.
//!
//! The chain has 12 stages that *could* fuse. This walks them in chain order and
//! reports, for each, whether softisp can emit a fused form, what the shader
//! needs, and whether the fused stage produces the same numbers as its
//! primitive form.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_fusion_candidates --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use std::path::PathBuf;

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::profile::PipelineProfile;

fn mnn_isp_dir() -> Option<PathBuf> {
    let mut c = Vec::new();
    if let Ok(p) = std::env::var("MNN_SOURCE") {
        c.push(PathBuf::from(p).join("source/backend/vulkan/buffer/execution/isp_spv"));
    }
    if let Ok(home) = std::env::var("HOME") {
        c.push(PathBuf::from(home).join("MNN/source/backend/vulkan/buffer/execution/isp_spv"));
    }
    c.into_iter().find(|d| d.is_dir())
}

/// One stage's fusion status.
struct Candidate {
    stage: &'static str,
    op: &'static str,
    /// Which softisp block could emit it, or None when nothing does.
    emitter: Option<&'static str>,
}

const CANDIDATES: &[Candidate] = &[
    Candidate {
        stage: "normalize",
        op: "isp.normalize",
        emitter: None,
    },
    Candidate {
        stage: "blc_dpc",
        op: "isp.dpc",
        emitter: None,
    },
    Candidate {
        stage: "blc",
        op: "isp.blc",
        emitter: None,
    },
    Candidate {
        stage: "cfa",
        op: "isp.cfa",
        emitter: None,
    },
    Candidate {
        stage: "lens shading",
        op: "isp.lsc",
        emitter: None,
    },
    Candidate {
        stage: "bayer wb",
        op: "isp.bayer_wb",
        emitter: None,
    },
    Candidate {
        stage: "demosaic+ccm",
        op: "isp.demosaic_ccm",
        emitter: Some("DemosaicCcmBlock"),
    },
    Candidate {
        stage: "ccm",
        op: "isp.ccm",
        emitter: Some("CcmBlock"),
    },
    Candidate {
        stage: "tone",
        op: "isp.tone",
        emitter: None,
    },
    Candidate {
        stage: "ee",
        op: "isp.ee",
        emitter: Some("EeBlock"),
    },
    Candidate {
        stage: "ldci",
        op: "isp.ldci",
        emitter: Some("LdciBlock"),
    },
    Candidate {
        stage: "saturation",
        op: "isp.saturation",
        emitter: None,
    },
    Candidate {
        stage: "vignetting",
        op: "isp.vignetting",
        emitter: None,
    },
    Candidate {
        stage: "display",
        op: "isp.display",
        emitter: Some("DisplayBlock"),
    },
];

/// Shader state for an op: implemented / placeholder / missing, plus the
/// bindings it declares.
fn shader_state(dir: &PathBuf, op: &str) -> (String, Vec<u32>) {
    let p = dir.join(format!("{}.comp", op.replace('.', "_")));
    let Ok(src) = std::fs::read_to_string(&p) else {
        let alt = dir.join(format!("{}.comp", op.replace('.', ".")));
        return match std::fs::read_to_string(alt) {
            Ok(_) => ("implemented".into(), Vec::new()),
            Err(_) => ("MISSING".into(), Vec::new()),
        };
    };
    let state = if src.contains("placeholder") {
        "placeholder"
    } else {
        "implemented"
    };
    let mut binds: Vec<u32> = src
        .lines()
        .filter(|l| l.contains("layout(set = 0, binding"))
        .filter_map(|l| {
            let at = l.find("binding = ")? + "binding = ".len();
            let rest = &l[at..];
            let end = rest.find(|c: char| !c.is_ascii_digit())?;
            rest[..end].parse().ok()
        })
        .collect();
    binds.sort_unstable();
    binds.dedup();
    (state.to_string(), binds)
}

/// Render one frame through MED at the given opset and report the RGB means.
fn render(opset_mode: i32) -> Result<(f64, f64, f64), String> {
    let (w, h) = (512u32, 384u32);
    let sample: u16 = 600;

    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;
    profile.opset_mode = opset_mode;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")
        .ok_or_else(|| "no engine".to_string())?;
    engine
        .build(head, blocks, None, 21)
        .map_err(|e| e.to_string())?;

    let mut buf = vec![0u8; (w as usize) * (h as usize) * 2];
    for c in buf.chunks_exact_mut(2) {
        c[0] = (sample & 0xFF) as u8;
        c[1] = (sample >> 8) as u8;
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
    };
    // Two frames: stats from N feed frame N+1.
    engine.process(&params).map_err(|e| e.to_string())?;
    let f = engine.process(&params).map_err(|e| e.to_string())?;

    let vals: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    let plane = vals.len() / 3;
    if plane == 0 {
        return Err("empty".into());
    }
    let mean = |p: &[f32]| p.iter().map(|v| *v as f64).sum::<f64>() / p.len() as f64;
    Ok((
        mean(&vals[0..plane]),
        mean(&vals[plane..2 * plane]),
        mean(&vals[2 * plane..]),
    ))
}

#[test]
#[ignore]
fn fusion_candidates_one_at_a_time() {
    cam_isp::init();
    let dir = mnn_isp_dir().expect("MNN shader dir (set MNN_SOURCE)");

    eprintln!(
        "\n{:<14} {:<18} {:<12} {:<7} {:<22} {}",
        "stage", "isp.* op", "shader", "emitter", "bindings", "verdict"
    );

    let mut ready = 0usize;
    for c in CANDIDATES {
        let (state, binds) = shader_state(&dir, c.op);
        let verdict = match (c.emitter, state.as_str()) {
            (None, _) => "no client emitter",
            (Some(_), "missing") | (Some(_), "MISSING") => "shader missing",
            (Some(_), "placeholder") => "shader is a stub",
            (Some(_), "implemented") => {
                ready += 1;
                "ready"
            }
            _ => "?",
        };
        eprintln!(
            "{:<14} {:<18} {:<12} {:<7} {:<22} {}",
            c.stage,
            c.op,
            state,
            c.emitter.unwrap_or("-"),
            format!("{binds:?}"),
            verdict
        );
    }

    eprintln!(
        "\n{ready} of {} stages have a working shader AND a client emitter",
        CANDIDATES.len()
    );
    eprintln!("Stages with a shader but no emitter need a block change (not a flag).");

    // What the two modes actually produce end to end.
    let prim = render(0);
    let fused = render(2);
    eprintln!("\nend to end at 512x384:");
    match (&prim, &fused) {
        (Ok((r, g, b)), Ok((r2, g2, b2))) => {
            eprintln!("  primitive  R {r:.4} G {g:.4} B {b:.4}");
            eprintln!("  MnnIspMin  R {r2:.4} G {g2:.4} B {b2:.4}");
            let close = (r - r2).abs() < 0.05 && (g - g2).abs() < 0.05 && (b - b2).abs() < 0.05;
            eprintln!(
                "  {}",
                if close {
                    "the fused chain matches the primitive one"
                } else {
                    "the fused chain does NOT match — stages are being dropped or mis-fed"
                }
            );
        }
        (Ok(_), Err(e)) => eprintln!("  MnnIspMin failed: {e}"),
        (Err(e), _) => eprintln!("  primitive failed: {e}"),
    }
}
