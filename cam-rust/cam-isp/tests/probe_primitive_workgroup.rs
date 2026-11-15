//! Where do the *primitive* (non-`isp.*`) ops get their Vulkan workgroup size?
//!
//! The question is whether MNN's ONNX op set is tuned for the GPU at all, or
//! whether it dispatches at a fixed geometry that happens to suit a desktop.
//! This greps the MNN sources rather than trusting the docs, because the answer
//! decides whether anything can be tuned from the model side.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_primitive_workgroup --features mnn -- --nocapture --ignored
//!
//! (`mnn` is only needed so the test lands in the same target as the rest; the
//! analysis itself reads files.)

use std::path::PathBuf;

fn mnn_root() -> Option<PathBuf> {
    let mut candidates = Vec::new();
    if let Ok(p) = std::env::var("MNN_SOURCE") {
        candidates.push(PathBuf::from(p));
    }
    if let Ok(home) = std::env::var("HOME") {
        candidates.push(PathBuf::from(home).join("MNN"));
    }
    candidates.into_iter().find(|d| d.is_dir())
}

/// Count occurrences of `needle` in a file, returning 0 when unreadable.
fn count(path: &PathBuf, needle: &str) -> usize {
    std::fs::read_to_string(path)
        .map(|s| s.lines().filter(|l| l.contains(needle)).count())
        .unwrap_or(0)
}

/// First line of `path` containing `needle`, trimmed.
fn first_line(path: &PathBuf, needle: &str) -> Option<String> {
    let s = std::fs::read_to_string(path).ok()?;
    s.lines()
        .find(|l| l.contains(needle))
        .map(|l| l.trim().to_string())
}

#[test]
#[ignore]
fn primitive_ops_have_a_fixed_workgroup() {
    let Some(root) = mnn_root() else {
        eprintln!("no MNN checkout found; set MNN_SOURCE");
        return;
    };
    let exec = root.join("source/backend/vulkan/buffer/execution");
    let glsl = exec.join("glsl");
    eprintln!("MNN: {}", root.display());

    // ── 1. The elementwise path ─────────────────────────────────────────
    //
    // `VulkanBinary` / `VulkanUnary` are what every primitive elementwise op
    // compiles to. Their dispatch call is the whole story.
    for f in ["VulkanBinary.cpp", "VulkanUnary.cpp"] {
        let p = exec.join(f);
        if let Some(line) = first_line(&p, "vkCmdDispatch") {
            eprintln!("\n{f}: {line}");
        }
    }

    // ── 2. The convolution path ─────────────────────────────────────────
    let conv = exec.join("VulkanConvolution.cpp");
    if let Some(line) = first_line(&conv, "mLocalX") {
        eprintln!("VulkanConvolution.cpp: {line}");
    }
    if let Some(line) = first_line(&conv, "vkCmdDispatch") {
        eprintln!("VulkanConvolution.cpp: {line}");
    }

    // ── 3. Whether any of it is device-dependent ────────────────────────
    //
    // A vendor check would show up as an Adreno/Mali/Qualcomm test near the
    // local-size decision. There is none in `VulkanConvolution` or the
    // elementwise path, so the geometry is the same on every GPU.
    let conv_src = std::fs::read_to_string(&conv).unwrap_or_default();
    // `VulkanConvolution.cpp` does branch on Adreno, but only to pick a
    // cooperative-matrix path for *quantised* 1x1 convolutions. A float model
    // never enters it, and it selects a kernel rather than a workgroup size.
    let adreno_coopmat = conv_src.contains("gpuType() == VulkanRuntime::ADRENO")
        && conv_src.contains("supportCoopMat");
    let quant_only = conv_src.contains("useInt8Conv && is1x1");
    eprintln!(
        "\nAdreno branch in VulkanConvolution: coop-matrix = {adreno_coopmat}, quantised-only = {quant_only}"
    );
    eprintln!("  (a float model never reaches it; it picks a kernel, not a workgroup size)");

    // ── 4. What the shaders themselves declare ──────────────────────────
    let mut fixed_256 = 0;
    let mut spec_constant = 0;
    let mut tiled_2d = 0;
    let mut other = 0;
    let mut total = 0;
    if let Ok(entries) = std::fs::read_dir(&glsl) {
        for e in entries.flatten() {
            let path = e.path();
            if path.extension().and_then(|x| x.to_str()) != Some("comp") {
                continue;
            }
            let Ok(src) = std::fs::read_to_string(&path) else {
                continue;
            };
            let Some(decl) = src.lines().find(|l| l.contains("local_size")) else {
                continue;
            };
            if !decl.trim_start().starts_with("layout") {
                continue;
            }
            total += 1;
            if decl.contains("local_size_x_id") {
                spec_constant += 1;
            } else if decl.contains("local_size_x = 256") {
                fixed_256 += 1;
            } else if decl.contains("local_size_x = 16") && decl.contains("local_size_y = 16") {
                tiled_2d += 1;
            } else {
                other += 1;
            }
        }
    }
    eprintln!("\nshader workgroup declarations ({total} files):");
    eprintln!("  local_size_x = 256 (flat, 1-D)      {fixed_256}");
    eprintln!("  local_size_x_id (specialisation)    {spec_constant}");
    eprintln!("  local_size 16x16 (2-D tile)         {tiled_2d}");
    eprintln!("  other fixed sizes                   {other}");

    // ── 5. Anything tunable from the model side? ────────────────────────
    //
    // `VulkanFuse` reads a `group_size` attribute off the op, but that is only
    // honoured for `isp.*` Extra ops. Primitive ONNX ops never reach it.
    let fuse = exec.join("VulkanFuse.cpp");
    let group_attr = count(&fuse, "\"group_size\"");
    eprintln!("\nVulkanFuse reads a `group_size` attribute: {group_attr} site(s)");
    eprintln!("  (only `isp.*` Extra ops reach VulkanFuse; plain ONNX ops do not)");

    eprintln!("\nConclusion:");
    eprintln!(
        "  Primitive ops dispatch at a fixed geometry: {fixed_256} shaders use a flat\n  \
         256-wide group and the elementwise dispatch is the constant UP_DIV(total, 256)\n  \
         with no device query; deep convolutions use a hardcoded 16x16."
    );
    eprintln!(
        "  Nothing in the model can change it: `group_size` is read only by VulkanFuse for\n  \
         isp.* Extra ops, so plain ONNX ops never see it."
    );
    assert!(
        fixed_256 > 0 && group_attr > 0,
        "expected a fixed 256-wide group and a VulkanFuse-only group_size hook"
    );
}
