# Gamma reciprocal audit — `inv_gamma` vs `gamma_recip` (2026-11-26)

## Question

Are `gamma_invert` / `inv_gamma` and `gamma_recip` the same value? If so, the
duplicate naming should be removed and the reciprocal derived once on the CPU.

## Verified answer

They are meant to be the same exponent (`1/γ` for display encoding), **but the
code currently uses the two names for different values**:

| Site | Value held | Consumer semantics |
|---|---|---|
| `cam_isp::isp_params::ToneParams::gamma` | γ (~2.2) | canonical source |
| `IspController::tone_gamma` | γ (clamped 2.0–2.6) | canonical source |
| `cam_types::ToneParams::gamma_recip` (from `controller_tone.rs:120`) | `1/γ` (~0.4545) | consumers treat it as **γ** |
| `cam_types::ToneParams::gamma_recip` (from `engine.rs:147`) | `2.2` (=γ) | consumers treat it as **γ** |
| `cam_types::ToneParams::gamma_recip` (from `mnnengine.rs:1906`) | `isp.tone.gamma` (=γ) | consumers treat it as **γ** |
| `Gamma/inv_gamma` | `1/γ` | `log → mul(inv_gamma) → exp` |
| `ToneLutBlock/inv_gamma` | `1/γ` | `Pow(x, inv_gamma)` |
| `isp_pipeline/gamma` uniform | `tone.gamma_recip` | shader computes `1/uniform` |

So `inv_gamma` is consistently `1/γ`, while `gamma_recip` is a legacy name for
**γ** in every consumer (`apply_tone` inverts once more; the Vulkan display
shader inverts it before `pow`). Only the two producers
`IspController::get_tone_params()` and `IpaContext::process_frame()` store the
reciprocal there, which makes controller-driven frames apply `pow(x, 2.2)`
(dark) instead of `pow(x, 1/2.2)`.

`ToneBlock/gamma_recip` written at `mnnengine.rs:1198` does not exist —
`ToneBlock` exposes only `contrast`, `brightness`, `exposure`, `zero`, `one`.
The write is dropped by the session tensor filter and is dead code.

## Fix

1. Rename `cam_types::ToneParams::gamma_recip` → `gamma` so the field name
   matches the value its consumers require (display gamma, ~2.2).
2. Fix the two producers to pass γ, not `1/γ`.
3. Remove the dead `ToneBlock/gamma_recip` write.
4. Keep `inv_gamma` as the single derived name for `1/γ`, computed on the CPU
   in `MnnEngine::set_extra_inputs()` (`Gamma/inv_gamma`,
   `ToneLutBlock/inv_gamma`) — no scalar reciprocal node in the graph.
5. Update the stale docs that still list `ToneBlock/gamma_recip`.

## Verification

- `cargo fmt --all -- --check`
- `cargo clippy -p cam-types -p cam-isp -p cam-ipa --lib --features cam-isp/rectifier -- -D warnings`
- `cargo test -p cam-types --lib`, `cargo test -p cam-isp --lib --features rectifier`

## Follow-up: MNN-fork shader contracts (same day)

The MNN fork used by this project is `99degree/MNN` (its `isp_*` shaders are not
in upstream `alibaba/MNN`). Every shader that touches a gamma-like coefficient
was checked at source. They do **not** share one convention:

| Shader | Param it reads | Exponent applied | softisp feed | Result |
|---|---|---|---|---|
| `isp.gamma` | binding 5 `params[0]` = `inv_gamma` | `params[0]` | `Gamma/inv_gamma` = `1/γ` | correct |
| `isp.display` | binding 5 `GammaBuf` = display γ | `1/gamma` | `DisplayBlock/gamma` = 2.4 (γ) | correct |
| `isp.display_uint32_bgra` (production packed path) | binding 0 `Uniforms[2]` = display γ | `1/u.gamma` | `VulkanFuse.cpp` copies graph input[1] (`DisplayBlock/gamma`) into `Uniforms[2]` | correct |
| `isp.algo_gamma` | binding 0 `u.gamma` = display γ | `1/u.gamma` | not emitted by any softisp block | latent |
| `isp.tone` | binding 0 `u.gamma` (baked `const`) | **`u.gamma` directly** (`exp(log(v)·u.gamma)`) | not emitted by any softisp block | latent trap |
| `isp.pipeline` | binding 9 `GamBuf` = `TONE_CURVE/GAMMA_RECIP` | **`gammaRecip` directly** | `isp_pipeline/gamma` | corrected below |

Two follow-up findings:

1. **`isp.pipeline` regression from the rename.** The shader's binding 9 is
   documented and used as `1/display_gamma`, and `PipelineAllBlock`'s built-in
   default is `1.0/2.2`. After renaming `cam_types::ToneParams::gamma_recip` to
   `gamma`, `set_extra_inputs()` briefly wrote γ there, which makes
   `pow(v, 2.2)` and darkens WHOLE-profile frames. It now writes `1/γ`, with a
   regression test pinning the block default to `1/2.2`.

2. **`isp.tone` is not a drop-in tone stage.** Its `u.gamma` is applied as the
   exponent itself, unlike `isp.algo_gamma`, which takes `1/u.gamma`. It is only
   referenced by the unused `fusion_patterns.rs` table and by probe tests — no
   `IspBlock` emits it — so nothing in the running pipeline is affected. If it
   is ever wired up, the CPU must feed `1/γ` (the `inv_gamma` value), not the
   display γ, or the tone stage will darken the frame. This belongs in the MNN
   fork if the field is renamed there; softisp cannot fix that shader here.

## Follow-up: missing `isp.tonelut`

`isp.tonelut` is absent from both MNN-fork contract points:
`IspOpsetValidate.hpp::kIspOpsetNames` and `IspSpvLookup.hpp::g_ispSpvMap` (the
`isp_spv/` tree has only `isp_tone.comp` and `isp_tone_stats.comp`). Therefore
`ToneLutBlock` has **no fused Vulkan shader** — it runs as primitive ONNX/MNN
elementwise ops. Its slopes can only arrive as graph inputs, which is exactly
the CPU-derived `ToneLutBlock/s0..s5` design from
[`20261126-tonelut-parallel-only.md`](20261126-tonelut-parallel-only.md). There
is no shader-side slope uniform to update until such an op is added to the
MNN fork.
