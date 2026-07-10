//! Generate API documentation markdown for runtime parameters (extra_inputs).
//!
//! Iterates all ISP blocks in different configurations and documents their
//! runtime-feedable input tensors, shapes, default values, and descriptions.
//!
//! Run: cargo run --release -p cam-isp --example gen_api_docs

use std::fs;
use std::path::Path;

use cam_isp::blocks::*;
use cam_isp::engine::OutputFormat;
use cam_isp::pipeline::IspBlock;

/// A documented runtime parameter.
struct DocParam {
    block_id: &'static str,
    block_desc: String,
    config: String,
    tensor_name: String,
    elem_type: i64,
    shape: Vec<i64>,
    default_val: String,
    description: String,
}

fn elem_type_str(t: i64) -> &'static str {
    match t {
        1 => "FLOAT",
        6 => "INT32",
        10 => "FLOAT16",
        _ => "UNKNOWN",
    }
}

fn shape_str(s: &[i64]) -> String {
    if s.is_empty() {
        "scalar".into()
    } else {
        format!("[{}]", s.iter().map(|d| d.to_string()).collect::<Vec<_>>().join(","))
    }
}

fn collect_params() -> Vec<DocParam> {
    let mut params = Vec::new();

    // ── DemosaicCcmBlock ────
    {
        let mut b = DemosaicCcmBlock::new(0);
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            let desc = if name.contains("/w") {
                "CCM convolution weights: shape [3,4,1,1] for 3×4 color correction matrix"
            } else {
                "CCM bias: shape [3] for per-channel offset"
            };
            params.push(DocParam {
                block_id: "demosaic_ccm",
                block_desc: "Fused demosaic + CCM".into(),
                config: "rggb".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: "4×3×1×1 weights, 3×1 bias".into(),
                description: desc.into(),
            });
        }
    }

    // ── BayerWbBlock ────
    {
        let mut b = BayerWbBlock::new();
        b.set_gains(1.0, 1.0, 1.0, 1.5); // non-identity to expose extra_inputs
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            params.push(DocParam {
                block_id: "bayer_wb",
                block_desc: "Bayer white balance — per-channel gain".into(),
                config: "rggb".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: "[1.0, 1.0, 1.0, 1.0]".into(),
                description: "RGGB channel gains for Bayer white balance".into(),
            });
        }
    }

    // ── ToneBlock ────
    {
        let b = ToneBlock::new();
        for (name, etype, shape) in b.extra_inputs() {
            let (desc, default) = if name.contains("contrast") {
                ("Contrast factor (1.0 = identity)", "1.0")
            } else if name.contains("brightness") {
                ("Brightness offset (-1..1)", "0.0")
            } else {
                ("Gamma reciprocal (1/γ)", "1.0")
            };
            params.push(DocParam {
                block_id: "tone",
                block_desc: "Tone mapping — contrast/brightness/gamma S-curve".into(),
                config: "default".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: default.into(),
                description: desc.into(),
            });
        }
    }

    // ── SaturationBlock ────
    {
        let mut b = SaturationBlock::new_default();
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            params.push(DocParam {
                block_id: "saturation",
                block_desc: "Saturation control — per-channel RGB factor".into(),
                config: "default".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: "[1.0, 1.0, 1.0]".into(),
                description: "Per-channel saturation scale [R,G,B]. 1.0 = identity".into(),
            });
        }
    }

    // ── SharpenBlock ────
    {
        let mut b = SharpenBlock::new(0.5);
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            params.push(DocParam {
                block_id: "sharpen",
                block_desc: "Unsharp mask sharpening".into(),
                config: "strength=0.5".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: "0.5".into(),
                description: "Sharpening strength. 0=off, 0.5=moderate, 1.0=strong".into(),
            });
        }
    }

    // ── LdciBlock ────
    {
        let mut b = LdciBlock::new();
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            params.push(DocParam {
                block_id: "ldci",
                block_desc: "Local contrast enhancement (adaptive tone mapping)".into(),
                config: "default".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: "1.0".into(),
                description: "Local contrast strength. 1.0 = default".into(),
            });
        }
    }

    // ── FcsBlock ────
    {
        let mut b = FcsBlock::new();
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            let (desc, default) = if name.contains("gain") {
                ("Per-channel gain [R,G,B]. 1.0 = identity", "[1.0, 1.0, 1.0]")
            } else {
                ("Per-channel bias [R,G,B]. 0.0 = identity", "[0.0, 0.0, 0.0]")
            };
            params.push(DocParam {
                block_id: "fcs",
                block_desc: "Film contrast stretch — Mul+Add per-channel".into(),
                config: "default".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: default.into(),
                description: desc.into(),
            });
        }
    }

    // ── NormalizeBlock ────
    {
        let mut b = NormalizeBlock::new();
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            params.push(DocParam {
                block_id: "normalize",
                block_desc: "INT32→FLOAT + Div by sensor max".into(),
                config: "default".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: "65535.0".into(),
                description: "Sensor max value (65535 for 16-bit)".into(),
            });
        }
    }

    // ── GammaBlock (active) ────
    {
        let mut b = GammaBlock::new(2.2);
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            let (desc, default) = if name.contains("inv_gamma") {
                ("Inverse gamma: 1/γ for gamma=2.2 → ~0.4545", format!("{:.4}", 1.0/2.2))
            } else if name.contains("min") {
                ("Clamp minimum (0.0) — Max(input, min)", "0.0".into())
            } else if name.contains("max") {
                ("Clamp maximum (1.0) — Min(input, max)", "1.0".into())
            } else if name.contains("lift") {
                ("Shadow lift offset (only when shadow_lift > 0)", "0.0".into())
            } else {
                ("Shadow lift norm factor (only when shadow_lift > 0)", "1.0".into())
            };
            params.push(DocParam {
                block_id: "gamma",
                block_desc: "Gamma correction — pow(x, 1/γ) + clamp + optional shadow lift".into(),
                config: "gamma=2.2".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: default,
                description: desc.into(),
            });
        }
    }

    // ── GammaBlock with shadow lift ────
    {
        let mut b = GammaBlock::new(2.2).with_shadow_lift(0.05);
        b.set_input_source("prev/frame");
        for (name, _etype, _shape) in b.extra_inputs() {
            if !name.contains("lift") && !name.contains("norm") { continue; }
        }
        for (name, etype, shape) in b.extra_inputs() {
            if !name.contains("lift") && !name.contains("norm") { continue; }
            let (desc, default) = if name.contains("lift") {
                ("Shadow lift offset (~0.01-0.10). Conditional: only when shadow_lift > 0.", "0.05")
            } else {
                ("Shadow lift re-normalize factor. Conditional: only when shadow_lift > 0.", "1.0")
            };
            params.push(DocParam {
                block_id: "gamma",
                block_desc: "Gamma correction (with shadow lift)".into(),
                config: "gamma=2.2, shadow_lift=0.05".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: default.into(),
                description: desc.into(),
            });
        }
    }

    // ── AutoContrastBlock (active) ────
    {
        let mut b = AutoContrastBlock::new(1.5);
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            let (desc, default) = if name.contains("lift") {
                ("Shadow lift offset. Conditional: only when shadow_lift > 0.01.", "0.0")
            } else if name.contains("half") {
                ("0.5 constant for center/uncenter S-curve.", "0.5")
            } else if name.contains("contrast_w") {
                ("Contrast weight factor. >1 = more contrast.", "1.5")
            } else if name.contains("zero") {
                ("Clip lower bound (0.0). Conditional.", "0.0")
            } else {
                ("Clip upper bound (1.0). Conditional.", "1.0")
            };
            params.push(DocParam {
                block_id: "auto_contrast",
                block_desc: "Adaptive contrast S-curve — center/stretch/uncenter".into(),
                config: "contrast=1.5".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: default.into(),
                description: desc.into(),
            });
        }
    }

    // ── AutoContrastBlock with highlight compress ────
    {
        let mut b = AutoContrastBlock::new(1.5).with_highlight_compress(0.1);
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            if !name.contains("zero") && !name.contains("one") { continue; }
            let default = if name.contains("zero") { "0.0" } else { "1.0" };
            let desc = if name.contains("zero") {
                "Clip lower bound. Conditional: only when highlight_compress > 0.01."
            } else {
                "Clip upper bound. Conditional: only when highlight_compress > 0.01."
            };
            params.push(DocParam {
                block_id: "auto_contrast",
                block_desc: "Adaptive contrast (with highlight compress)".into(),
                config: "contrast=1.5, highlight_compress=0.1".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: default.into(),
                description: desc.into(),
            });
        }
    }

    // ── DisplayBlock (FloatRgb — has extra_inputs) ────
    {
        let mut b = DisplayBlock::new(1920).with_output_format(OutputFormat::FloatRgb);
        b.set_input_source("prev/frame");
        for (name, etype, shape) in b.extra_inputs() {
            let (desc, default) = if name.contains("scale") {
                ("Scale factor. Used in Mul for FloatRgb path.".to_string(), "1.0".to_string())
            } else if name.contains("gamma_exp") {
                ("Gamma exponent for sRGB (1/2.4 ≈ 0.4167). Used in Pow.".to_string(), format!("{:.4}", 1.0/2.4))
            } else if name.contains("zero") {
                ("Clip lower bound (0.0) after gamma.".to_string(), "0.0".to_string())
            } else {
                ("Clip upper bound (1.0) after gamma.".to_string(), "1.0".to_string())
            };
            params.push(DocParam {
                block_id: "display",
                block_desc: "Display format conversion (FloatRgb)".into(),
                config: "FloatRgb".into(),
                tensor_name: name,
                elem_type: etype,
                shape,
                default_val: default,
                description: desc.into(),
            });
        }
    }

    // ── DisplayBlock (ARGB — no runtime params) ────
    {
        let b = DisplayBlock::new(1920).with_output_format(OutputFormat::Argb);
        if b.extra_inputs().is_empty() {
            params.push(DocParam {
                block_id: "display",
                block_desc: "Display format conversion (ARGB8888)".into(),
                config: "Argb".into(),
                tensor_name: "(none)".into(),
                elem_type: 0,
                shape: vec![],
                default_val: "-".into(),
                description: "ARGB uses fixed Conv weights — no runtime params.".into(),
            });
        }
    }

    // ── DisplayBlock (PackedRgb — no runtime params) ────
    {
        let b = DisplayBlock::new(1920).with_output_format(OutputFormat::PackedRgb);
        if b.extra_inputs().is_empty() {
            params.push(DocParam {
                block_id: "display",
                block_desc: "Display format conversion (PackedRgb)".into(),
                config: "PackedRgb".into(),
                tensor_name: "(none)".into(),
                elem_type: 0,
                shape: vec![],
                default_val: "-".into(),
                description: "PackedRgb uses fixed weights — no runtime params.".into(),
            });
        }
    }

    params
}

fn write_markdown(params: &[DocParam], path: &Path) {
    let mut md = String::new();
    md.push_str("# Runtime Parameters (extra_inputs) API\n\n");
    md.push_str("This document lists all runtime-feedable input tensors exposed by ISP blocks\n");
    md.push_str("via the `extra_inputs()` trait method. These tensors default to initializer values\n");
    md.push_str("but can be overridden per-frame through the MNN engine's `set_extra_inputs()`.\n\n");
    md.push_str("> **Note**: Conditional tensors are only present in the ONNX graph when the\n");
    md.push_str("> corresponding block parameter is active. Feeding a non-existent tensor is harmless\n");
    md.push_str("> — the engine skips it via `Option::None`.\n\n");

    // Group by block
    let mut block_ids: Vec<&str> = params.iter().map(|p| p.block_id).collect();
    block_ids.sort();
    block_ids.dedup();

    for &bid in &block_ids {
        let block_params: Vec<&DocParam> = params.iter().filter(|p| p.block_id == bid).collect();
        let first = block_params[0];
        md.push_str(&format!("## `{}` — {}\n\n", bid, first.block_desc));

        md.push_str("| Tensor Suffix | Config | Shape | Type | Default | Description |\n");
        md.push_str("|---|---|---|---|---|---|\n");
        for p in &block_params {
            // Extract just the suffix after namespace/
            let suffix = p.tensor_name.rsplit('/').next().unwrap_or(&p.tensor_name);
            md.push_str(&format!(
                "| `{}` | {} | {} | {} | {} | {} |\n",
                suffix,
                p.config,
                shape_str(&p.shape),
                elem_type_str(p.elem_type),
                p.default_val,
                p.description,
            ));
        }
        md.push('\n');
    }

    md.push_str("## Notes\n\n");
    md.push_str("- **Feed safety**: All extra_input tensors are also present as ONNX initializers.\n");
    md.push_str("  If the engine does not feed a value, the initializer default is used.\n");
    md.push_str("- **Conditional tensors**: Documented per-config above. Only created when the\n");
    md.push_str("  block's parameter exceeds its activation threshold.\n");
    md.push_str("- **Shape convention**: `scalar` = rank-0 tensor; `[N]` = 1D; `[H,W]` = 2D\n");
    md.push_str("- **Data type**: All runtime params are FLOAT (32-bit).\n");
    md.push_str("- **Tensor naming**: `{Namespace}/{suffix}`. Namespace = block's `tensor_ns()`.\n");

    fs::write(path, md).expect("Failed to write API docs");
    println!("Wrote {}", path.display());
}

fn main() {
    let params = collect_params();
    let out_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent().unwrap()
        .parent().unwrap()
        .join("docs")
        .join("api")
        .join("RUNTIME_PARAMS.md");
    write_markdown(&params, &out_path);
    let mut ids: Vec<&str> = params.iter().map(|p| p.block_id).collect();
    ids.sort();
    ids.dedup();
    println!("Generated {} parameters across {} blocks", params.len(), ids.len());
}
