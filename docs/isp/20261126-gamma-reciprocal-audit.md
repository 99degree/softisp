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
