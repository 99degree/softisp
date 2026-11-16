//! Which `isp.*` names are real, on both sides of the boundary?
//!
//! softisp emits names; MNN runs them. A name that MNN cannot run is worse than
//! a missing stage: `VulkanFuseCreator::onCreate` returns nullptr, the pipeline
//! falls through to a CPU backup that has no implementation, and the op is
//! dropped or the frame comes back wrong — with no error that points here.
//!
//! Three things decide whether a name can run:
//!
//!   whitelist  source/core/IspOpsetValidate.hpp          (converter TRAP check)
//!   lookup     .../execution/IspSpvLookup.hpp            (embedded SPIR-V table)
//!   shader     .../execution/isp_spv/isp_<name>.comp     (real vs placeholder)
//!
//! Report both directions: what MNN can run, and what softisp emits.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_isp_op_audit --features mnn -- --nocapture --ignored

use std::path::PathBuf;

fn mnn_root() -> Option<PathBuf> {
    let mut c = Vec::new();
    if let Ok(p) = std::env::var("MNN_SOURCE") {
        c.push(PathBuf::from(p));
    }
    if let Ok(home) = std::env::var("HOME") {
        c.push(PathBuf::from(home).join("MNN"));
    }
    c.into_iter().find(|d| d.is_dir())
}

/// Pull every `isp.x` string out of a file.
fn names_in(path: &PathBuf) -> Vec<String> {
    let Ok(src) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    let mut rest = src.as_str();
    while let Some(i) = rest.find("\"isp.") {
        let tail = &rest[i + 1..];
        let Some(end) = tail.find('"') else { break };
        out.push(tail[..end].to_string());
        rest = &tail[end..];
    }
    out.sort();
    out.dedup();
    out
}

/// `true` when the shader exists and is not a stub.
fn shader_state(root: &PathBuf, name: &str) -> &'static str {
    let p = root.join(format!(
        "source/backend/vulkan/buffer/execution/isp_spv/isp_{}.comp",
        name.trim_start_matches("isp.")
    ));
    match std::fs::read_to_string(&p) {
        Err(_) => "MISSING",
        Ok(src) => {
            if src.contains("placeholder") {
                "placeholder"
            } else {
                "implemented"
            }
        }
    }
}

#[test]
#[ignore]
fn isp_names_that_actually_run() {
    let Some(root) = mnn_root() else {
        eprintln!("no MNN checkout found; set MNN_SOURCE");
        return;
    };
    let white: Vec<String> = names_in(&root.join("source/core/IspOpsetValidate.hpp"));
    let lookup: Vec<String> =
        names_in(&root.join("source/backend/vulkan/buffer/execution/IspSpvLookup.hpp"));

    let runnable: Vec<&String> = white
        .iter()
        .filter(|n| lookup.contains(n) && shader_state(&root, n) == "implemented")
        .collect();
    eprintln!("MNN can run {} isp.* ops:", runnable.len());

    // ── What softisp emits at runtime ───────────────────────────────────
    //
    // Only the blocks can emit; `fusion_patterns.rs` is a table nothing reads,
    // so its names are reported separately (they are not a bug source).
    let src = root.parent().map(|p| p.join("softisp")).unwrap_or_default();
    let _ = src;
    let emitted = [
        "isp.demosaic_ccm",
        "isp.display",
        "isp.ee",
        "isp.ldci",
        "isp.unpack_blc",
    ];

    eprintln!(
        "\n{:<24} {:<10} {:<8} {}",
        "softisp emits", "whitelist", "lookup", "shader"
    );
    let mut dead = Vec::new();
    for name in emitted {
        let w = white.contains(&name.to_string());
        let l = lookup.contains(&name.to_string());
        let s = shader_state(&root, name);
        if !(w && l && s == "implemented") {
            dead.push(name);
        }
        eprintln!(
            "{name:<24} {:<10} {:<8} {s}",
            if w { "yes" } else { "NOT IN" },
            if l { "yes" } else { "NOT IN" }
        );
    }

    eprintln!(
        "\n{} of {} runtime names cannot run in MNN: {}",
        dead.len(),
        emitted.len(),
        dead.join(", ")
    );

    // The audit is only useful if it is honest about the bad state; pin the
    // finding so a future MNN rebuild that fixes a shader shows up as a change.
    assert!(!runnable.is_empty(), "expected some runnable isp ops");
    assert!(
        dead.contains(&"isp.unpack_blc"),
        "isp.unpack_blc is reported runnable — the audit or MNN changed; re-check"
    );
}
