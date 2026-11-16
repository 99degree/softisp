//! Verify the whole contract chain for an `isp.*` op: ONNX node → MNN Extra →
//! VulkanFuse → shader.
//!
//! Each link has its own failure mode, and none of them reports an error that
//! points at the link:
//!
//!   softisp emits attributes the node's shader never reads (silently ignored);
//!   the converter keeps attributes it does not understand (harmless);
//!   VulkanFuse reads only its own allow-list, so anything else is decoration;
//!   the shader reads buffers whose contents nothing writes (zeros).
//!
//! This walks a real graph and compares the three lists that have to agree:
//! what the block emits, what VulkanFuse consumes, and what the shader declares.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_isp_contract --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use std::path::PathBuf;

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

/// Attribute keys `VulkanFuse` reads, extracted from its source.
fn fuse_attributes(root: &PathBuf) -> Vec<String> {
    // `root` is .../execution/isp_spv, so VulkanFuse.cpp is one level up.
    let src = std::fs::read_to_string(root.join("../VulkanFuse.cpp")).unwrap_or_default();
    let mut out: Vec<String> = Vec::new();
    for line in src.lines() {
        if let Some(rest) = line.split("attr->key()->str() == \"").nth(1) {
            if let Some(end) = rest.find('"') {
                out.push(rest[..end].to_string());
            }
        }
    }
    out.sort();
    out.dedup();
    out
}

/// Binding declarations in a shader source: (name, binding).
fn shader_bindings(root: &PathBuf, op: &str) -> Vec<(String, u32)> {
    let path = root.join(format!("isp_{}.comp", op.trim_start_matches("isp.")));
    let Ok(src) = std::fs::read_to_string(&path) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for line in src.lines() {
        let Some(bpos) = line.find("binding = ") else {
            continue;
        };
        let rest = &line[bpos + "binding = ".len()..];
        let Some(comma) = rest.find(')') else {
            continue;
        };
        let Ok(binding) = rest[..comma]
            .trim_end_matches(&[',', ' '][..])
            .parse::<u32>()
        else {
            continue;
        };
        // The buffer's declared name follows the storage class keyword.
        let name = line
            .rsplit("buffer ")
            .next()
            .and_then(|s| s.split_whitespace().next())
            .unwrap_or("?")
            .to_string();
        out.push((name, binding));
    }
    out.sort_by_key(|(_, b)| *b);
    out
}

#[test]
#[ignore]
fn isp_contract_chain() {
    let Some(root) = mnn_isp_dir() else {
        eprintln!("no MNN shader tree found; set MNN_SOURCE");
        return;
    };

    let fuse = fuse_attributes(&root);
    eprintln!("VulkanFuse reads {} attribute keys:", fuse.len());
    eprintln!("  {}\n", fuse.join(", "));

    // The attributes each fusion-capable block emits. Kept as data rather than
    // parsed so the test says what it is checking.
    let emitted: [(&str, &[&str]); 3] = [
        ("ee", &["kernel_shape", "pads", "strides", "group"]),
        ("ldci", &["kernel_shape", "strides", "pads"]),
        (
            "display",
            &[
                "rotate_mode",
                "output_format",
                "pack_two_pixels",
                "in_h",
                "in_w",
            ],
        ),
    ];

    eprintln!(
        "{:<16} {:>9} {:>9}  emitted-but-ignored",
        "op", "emitted", "consumed"
    );
    let mut ignored_total = 0;
    for (op, attrs) in emitted {
        let consumed = attrs
            .iter()
            .filter(|a| fuse.iter().any(|f| f == *a))
            .count();
        let ignored: Vec<&str> = attrs
            .iter()
            .copied()
            .filter(|a| !fuse.iter().any(|f| f == *a))
            .collect();
        ignored_total += ignored.len();
        eprintln!(
            "isp.{op:<12} {:>9} {:>9}  {}",
            attrs.len(),
            consumed,
            if ignored.is_empty() {
                "-".to_string()
            } else {
                ignored.join(", ")
            }
        );
    }

    eprintln!("\n{ignored_total} emitted attributes are discarded before the shader sees them.");
    eprintln!(
        "That is not a bug in itself — VulkanFuse is positional, and the shader\n\
         gets its coefficients from buffers, not attributes. It matters because a\n\
         block that emits only attributes (EE's kernel, LDCI's strength, the display\n\
         gamma) is sending data to a shader that cannot hear it."
    );

    // Show, per op, which buffers the shader expects and whether anything fills
    // them. The list of fillers is the engine's `set_extra_inputs` writes.
    eprintln!("\nshader-declared buffers per op:");
    for op in ["ee", "ldci", "demosaic_ccm", "display", "normalize"] {
        let b = shader_bindings(&root, op);
        eprintln!("  isp.{op:<16} {:?}", b);
    }
}
