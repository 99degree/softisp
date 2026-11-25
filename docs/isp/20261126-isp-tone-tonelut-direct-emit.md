# Direct emit for `isp.tone` and `isp.tonelut` (2026-11-26)

## Symptom / design concern

`ToneBlock` emits four primitive ONNX nodes (Mul → Mul → Add → Clip) and
`ToneLutBlock` emits thirty (y0 seed, six 4-node LUT segments, contrast,
brightness, clip, gamma). Both are pure per-pixel work and both have a natural
fused Vulkan form, but neither block can emit one today:

- `probe_fusion_candidates.rs` records `tone / isp.tone / emitter: None`.
- `fusion_patterns.rs` lists `["tone"] => Some("isp.tone")` but no block emits
  the op, so the pattern never fires.
- `isp.tonelut` is absent from the MNN fork's `IspOpsetValidate.hpp` and
  `IspSpvLookup.hpp`; the `isp_spv/` tree has only `isp_tone.comp` and
  `isp_tone_stats.comp`.

The requirement is direct emit: one fused `isp.*` node per block, with all
scalar derivations done on the CPU and fed as tensor inputs. Vulkan shaders live
in the MNN fork (`libMNN_Vulkan.so`); softisp emits the ops and relies on MNN's
embedded SPIR-V lookup — no shader files are shipped by softisp.

## Root cause / contract analysis

### How fused ops are emitted today

Blocks that support fusion implement `BlockOpsetMode` (`Primitive` / `Custom` /
`MnnIspMin`), store a `forced_mode`, and switch `nodes()` on the mode:
`MnnIspMin` and `Custom` emit `Proto::node_with_domain("isp.<op>", "isp", ...)`
with an `elementwise=1` attribute; `Primitive` emits standard ONNX nodes.
`FuseStages` (in `profile.rs`) gates which stages are allowed, and
`profile_builder.rs` calls `with_opset_mode()` on the blocks when the profile's
`opset_mode == 2` (MnnIspMin) and the stage flag is true. `set_extra_inputs()`
in `mnnengine.rs` writes the per-frame coefficient tensors by name.

### VulkanFuse binding map

`VulkanFuse.cpp` in the MNN fork (`99degree/MNN`) uses a universal map for
`isp.*` Extra ops:

| Binding | Contents |
|---|---|
| 0 | params SSBO — `width, height` hot-swapped by `elementwise=1` |
| 3 | input frame (`InBuf`, planar RGB float) |
| 4 | output frame (`OutBuf`) |
| 5.. | extra inputs in declaration order |

All shaders are embedded in `libMNN_Vulkan.so` via `IspSpvLookup.hpp`; the
`spirv` attribute on an Extra op is only a fallback for ops absent from the
embedded map. softisp does not use that fallback.

### Gamma semantics (from the earlier audit)

- `isp.gamma` expects `1/γ` directly.
- `isp.display*` expect display γ and compute `1/γ` internally.
- Gamma is owned by the tone stage, not by `isp.display*` (which already
  applies `pow(v, 1/γ)` during byte packing) and not by a separate `GammaBlock`.
  `ToneBlock` computes `inv_gamma = 1/γ` on the CPU and feeds
  `ToneBlock/inv_gamma`; `ToneLutBlock` does the same with
  `ToneLutBlock/inv_gamma`. Both primitive and fused forms apply the exponent.
  `DisplayBlock/gamma` is fed `1.0` so the display encode is an identity.

### CPU-derived slopes

`ToneLutBlock` no longer computes segment differences/slopes in the graph.
`tone_curve_slopes()` derives the six `s_i` on the CPU and
`set_extra_inputs()` writes them to `ToneLutBlock/s0..s5`. The MNN shader must
consume those same slopes; no slope-derivation node may be reintroduced.

## Fix applied

1. **Rust direct emit**:
   - `ToneBlock` gains `BlockOpsetMode` support; fused form emits one
     `isp.tone` node with `[frame, exposure, contrast, brightness, inv_gamma]`
     as graph inputs and `elementwise=1`. It now owns the gamma encode.
   - `ToneLutBlock` gains `BlockOpsetMode` support; fused form emits one
     `isp.tonelut` node with `[frame, y0, s0..s5, contrast, brightness,
     inv_gamma]` as graph inputs and `elementwise=1`.
   - `FuseStages` gains `tone` and `tonelut` flags; `profile_builder.rs` and
     `PipelineBuilder::tone_with_modes()` select the fused forms.
   - `set_extra_inputs()` writes the same tensors the primitive form uses, so
     both paths share one controller contract.

2. **MNN fork contract** (`docs/mnn/ISP_TONE_TONELUT_FORK_PATCH.md`): documents
   the shader source and registration changes required in the fork so its
   embedded SPIR-V matches the emitted tensor contract.

## Follow-up resolved

`isp.display*` audit is complete
([`20261126-isp-display-absorption-audit.md`](20261126-isp-display-absorption-audit.md)):
all display shaders apply only gamma + format packing — no BCS, rotation or
tone. Gamma is now owned by the tone stage and the display encode is neutral.

## Verification

- `cargo fmt --all -- --check`
- per-crate clippy loop with `-D warnings`
- `cargo test -p cam-isp --lib --features rectifier`
- `cargo check -p cam-isp --features "rectifier mnn"`
