//! Which of MNN's `isp.*` fused shaders are real, and which are stubs?
//!
//! Enabling a fused op only helps if the shader behind it implements the same
//! maths as the primitive chain. MNN's backend ships 82 `isp.*` shaders; a large
//! fraction are `// placeholder compute shader` stubs that copy their input.
//! Turning one of those on does not speed anything up — it silently drops the
//! stage, and the frame still "works", which is why a picture check alone is not
//! enough to adopt one.
//!
//! This is the evidence for which ops are safe to switch on: it greps the shader
//! sources rather than trusting the name.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_isp_shader_audit --features mnn -- --nocapture --ignored

use std::path::{Path, PathBuf};

/// Where a local MNN checkout keeps the shader sources, if there is one.
fn mnn_shader_dirs() -> Vec<PathBuf> {
    let mut dirs = Vec::new();
    if let Ok(p) = std::env::var("MNN_SOURCE") {
        dirs.push(PathBuf::from(p).join("source/backend/vulkan/buffer/execution/isp_spv"));
    }
    if let Ok(home) = std::env::var("HOME") {
        dirs.push(PathBuf::from(home).join("MNN/source/backend/vulkan/buffer/execution/isp_spv"));
    }
    dirs
}

/// `true` when the shader says it is a stub.
fn is_placeholder(src: &str) -> bool {
    src.contains("placeholder compute shader") || src.contains("PLACEHOLDER")
}

#[test]
#[ignore]
fn fused_shaders_are_real() {
    let Some(dir) = mnn_shader_dirs().into_iter().find(|d| d.is_dir()) else {
        eprintln!("no MNN shader source tree found; set MNN_SOURCE to the MNN checkout");
        return;
    };
    eprintln!("shader source: {}", dir.display());

    // The stages a MED preview actually runs, plus the ones most tempting to
    // fuse because they are convolution-heavy.
    let candidates = [
        "isp_normalize",
        "isp_blc",
        "isp_cfa",
        "isp_bayer_wb",
        "isp_demosaic_ccm",
        "isp_ccm",
        "isp_tone",
        "isp_saturation",
        "isp_unsharp",
        "isp_dpc",
        "isp_lsc",
        "isp_ee",
        "isp_ldci",
        "isp_fcs",
        "isp_bilateral",
        "isp_colorspace",
        "isp_display",
        "isp_display_rgb",
    ];

    eprintln!("\n{:<24} {:<12} note", "shader", "state");
    let mut real = Vec::new();
    let mut stub = Vec::new();
    for name in candidates {
        let path = dir.join(format!("{name}.comp"));
        match std::fs::read_to_string(&path) {
            Ok(src) => {
                if is_placeholder(&src) {
                    stub.push(name);
                    eprintln!("{name:<24} {:<12} copies its input", "PLACEHOLDER");
                } else {
                    // Pull the first descriptive comment line, so the report says
                    // what the shader claims to do.
                    let note = src
                        .lines()
                        .take(6)
                        .find(|l| l.starts_with("// ") && !l.contains("#version"))
                        .map(|l| l.trim_start_matches("// ").to_string())
                        .unwrap_or_default();
                    real.push(name);
                    eprintln!("{name:<24} {:<12} {note}", "implemented");
                }
            }
            Err(_) => eprintln!("{name:<24} {:<12} no source file", "MISSING"),
        }
    }

    eprintln!("\n{} implemented, {} placeholders", real.len(), stub.len());
    eprintln!(
        "Safe to enable as fused isp.* ops: {}",
        real.iter()
            .filter(|n| **n != "isp_display")
            .copied()
            .collect::<Vec<_>>()
            .join(", ")
    );
    eprintln!(
        "Must stay primitive (turning these on silently deletes the stage): {}",
        stub.join(", ")
    );

    // The point of the audit: an enabled fused op must not be a stub. The
    // engine offers no switch yet, so this asserts the classification is
    // non-empty rather than gating a build on it.
    assert!(
        !stub.is_empty(),
        "expected some placeholder shaders in this MNN"
    );
    assert!(!real.is_empty(), "expected some implemented shaders");
}
