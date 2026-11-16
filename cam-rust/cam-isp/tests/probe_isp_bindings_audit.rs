//! Where does each shipped `isp.*` shader actually declare its tensors?
//!
//! This reads the compiled `.spv`, not the `.comp` source: the bytecode is what
//! the driver sees, and a stale `.spv` next to an edited `.comp` is a real
//! failure mode. For every shader it reports the descriptor bindings, so the
//! client-side map can be checked against reality instead of against comments.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_isp_bindings_audit --features mnn -- --nocapture --ignored

use std::path::PathBuf;
use std::process::Command;

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

/// Bindings declared by a compiled SPIR-V module.
fn spirv_bindings(path: &PathBuf) -> Vec<u32> {
    let Ok(out) = Command::new("spirv-dis").arg(path).output() else {
        return Vec::new();
    };
    let text = String::from_utf8_lossy(&out.stdout);
    let mut b: Vec<u32> = text
        .lines()
        .filter(|l| l.contains("OpDecorate") && l.contains("Binding"))
        .filter_map(|l| {
            l.split_whitespace()
                .last()
                .and_then(|v| v.parse::<u32>().ok())
        })
        .collect();
    b.sort_unstable();
    b.dedup();
    b
}

/// Bindings declared by the GLSL source, for the same cross-check.
fn comp_bindings(path: &PathBuf) -> Vec<u32> {
    let Ok(src) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let mut b: Vec<u32> = src
        .lines()
        .filter(|l| l.contains("layout(set = 0, binding"))
        .filter_map(|l| {
            let at = l.find("binding = ")? + "binding = ".len();
            let rest = &l[at..];
            let end = rest.find(|c: char| !c.is_ascii_digit())?;
            rest[..end].parse().ok()
        })
        .collect();
    b.sort_unstable();
    b.dedup();
    b
}

#[test]
#[ignore]
fn shader_bindings_match_their_sources() {
    let Some(dir) = mnn_isp_dir() else {
        eprintln!("no MNN shader tree found; set MNN_SOURCE");
        return;
    };

    let mut stale = Vec::new();
    let mut checked = 0;
    let mut entries: Vec<_> = std::fs::read_dir(&dir)
        .expect("read shader dir")
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|x| x.to_str()) == Some("spv"))
        .collect();
    entries.sort();

    for spv in &entries {
        let comp = spv.with_extension("comp");
        let from_spv = spirv_bindings(spv);
        let from_comp = comp_bindings(&comp);
        if from_spv.is_empty() {
            continue;
        }
        checked += 1;
        if !from_comp.is_empty() && from_spv != from_comp {
            stale.push((
                spv.file_stem().unwrap().to_string_lossy().to_string(),
                from_comp,
                from_spv,
            ));
        }
    }

    eprintln!("checked {checked} shaders");
    if stale.is_empty() {
        eprintln!("\nevery .spv matches its .comp bindings");
    } else {
        eprintln!(
            "\n{} shader(s) where the compiled SPIR-V disagrees with the source:",
            stale.len()
        );
        for (name, comp, spv) in &stale {
            eprintln!("  {name:<32} comp={comp:?}  spv={spv:?}");
        }
    }

    // Report the frame slot each shader uses; the client-side binding map has to
    // agree with the majority, and the exceptions are what break.
    let mut three_four = 0;
    let mut other = Vec::new();
    for spv in &entries {
        let b = spirv_bindings(spv);
        if b.is_empty() {
            continue;
        }
        if b.contains(&3) && b.contains(&4) {
            three_four += 1;
        } else {
            other.push((spv.file_stem().unwrap().to_string_lossy().to_string(), b));
        }
    }
    eprintln!("\n{three_four} shader(s) use frame at 3/4 (the current convention)");
    eprintln!("{} use a different map:", other.len());
    for (name, b) in &other {
        eprintln!("  {name:<32} {b:?}");
    }

    assert!(
        stale.is_empty(),
        "{} shader(s) have a .spv that disagrees with their .comp — rebuild the SPIR-V \
         before trusting either file",
        stale.len()
    );
}
