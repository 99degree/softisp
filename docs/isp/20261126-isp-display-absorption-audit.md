# `isp.display*` absorption audit (2026-11-26)

## Symptom / question

Follow-up from the `isp.tone` / `isp.tonelut` direct-emit work: do the
`isp.display*` Vulkan shaders absorb work that softisp also emits as standalone
ISP blocks? If they do, a fused pipeline applies that stage twice.

Initial suspicion came from a stale comment in `DisplayBlock` claiming the fused
op handles "rotation + format conversion + gamma + BCS in one dispatch".

## Method

Read every `isp_display*.comp` in the MNN fork at `~/MNN`, then trace the
softisp side: `DisplayBlock` emission, `profile_builder` block order, and
`MnnEngine::set_extra_inputs()` tensor writes. No code was changed until the
contract was mapped.

## Finding 1 — all `isp.display*` shaders apply gamma

Every display variant applies `pow(v, 1/gamma)`; the only differences are where
`gamma` comes from and whether the encode is conditional:

| Op type | Gamma source | Encode | Absorbs |
|---|---|---|---|
| `isp.display` | graph input → binding 5 `GammaBuf` | unconditional `pow(v,1/g)` | gamma |
| `isp.display_rgb` | binding 0 `Uniforms[2]` | unconditional | gamma |
| `isp.display_uint32_argb` | binding 0 `Uniforms[2]` | unconditional | gamma + byte pack |
| `isp.display_uint32_bgra` (production `PackedRgb`) | binding 0 `Uniforms[2]` | unconditional | gamma + byte pack |
| `isp.display_uint32_rgba` | binding 0 `Uniforms[2]` | unconditional | gamma + byte pack |
| `isp.display_uint8_argb` | binding 0 `Uniforms[2]` | unconditional | gamma + byte pack |
| `isp.display_uint8_bgra` | binding 0 `Uniforms[2]` | unconditional | gamma + byte pack |
| `isp.display_uint8_rgba` | binding 0 `Uniforms[2]` | **conditional**: skip if `gamma<=0` or `gamma==1` | gamma + `fullScale` + byte pack |
| `isp.display_rgba_f16` | binding 0 `Uniforms[2]` | conditional (`gamma==1` skips) | gamma + `fullScale` + f16 pack |

`VulkanFuse.cpp` copies graph input[1] (`DisplayBlock/gamma`) into
`Uniforms[2]` for every `isp.display_*` variant, so a value of `1.0` makes the
encode an identity (`pow(v,1) == v`). `isp.display` reads binding 5 directly,
where `1.0` is likewise an identity.

## Finding 2 — no BCS, no rotation, no tone

None of the shaders read exposure, brightness, contrast, saturation, lift, or
rotation attributes. The word "saturation" appears only in a copy-pasted
comment in `isp_display_uint32_argb.comp`. `DisplayBlock`'s own doc comment
("gamma + BCS in one dispatch") is wrong and must be corrected. Tone work
(`isp.tone` / `isp.tonelut`) is **not** absorbed — those stages stay separate.

`fullScale` (`Uniforms[3]`, used only by the uint8-rgba / f16 packers) is not
patched by `VulkanFuse.cpp`; an unfed binding-0 leaves it 0 and the shaders
guard it to 1.0 (assume already-normalized input). Tracked as a TODO below.

Rotation was therefore never applied on the fused production path even though
`DisplayBlock` declared a swapped output shape. The fused emitter now writes the
shader into an intermediate tensor and applies `Transpose` / `Slice` nodes to
reach `DisplayBlock/frame`; see
`docs/isp/20261126-default-on-ahb-rotation-report.md`.

## Finding 3 — softisp applies gamma twice when a standalone `GammaBlock` is present

`profile_builder.rs` inserts `GammaBlock::new(2.2)` whenever `use_gamma` is true
(HEAVY and PRO profiles both set it). `GammaBlock` computes
`pow(x, 1/2.2) ≈ x^0.4545`.

Separately:

- **Fused display** (`opset_mode=2`, `fuse_stages.display`): `DisplayBlock`
  emits one `isp.display_*` node with `DisplayBlock/gamma` as graph input.
  `set_extra_inputs()` writes `2.4` unconditionally → shader applies
  `pow(v, 1/2.4) ≈ v^0.4167`. Combined exponent ≈ 0.189 — badly washed out.
- **Primitive `FloatRgb` identity path**: `DisplayBlock::nodes()` emits
  `Pow(x, gamma_exp=1/2.4)` + `Clip`, again on top of `GammaBlock`.

Primitive `PackedRgb` (the profile default) does **not** apply gamma in
`DisplayBlock` — so the default path is correct. The double-apply appears only
when the display stage is fused (or when `FloatRgb` output is selected).

## Fix

Gamma belongs to the **tone stage**, not to display: `ToneBlock` now applies
`pow(clip(...), 1/γ)` in both its primitive graph (`Pow` node) and its fused
`isp.tone` shader (`ToneBlock/inv_gamma` at binding 8). `ToneLutBlock` already
owned gamma in both forms. `profile_builder.rs` therefore emits an
`IdentityBlock` for the `gamma` stage instead of `GammaBlock`, and
`MnnEngine::set_extra_inputs()` feeds `DisplayBlock/gamma = 1.0` and
`DisplayBlock/gamma_exp = 1.0` so every `isp.display*` encode is an identity.
The profile's `use_gamma` flag only seeds the graph-time default; the runtime
`ToneParams::gamma` always drives `ToneBlock/inv_gamma` (CPU-derived `1/γ`),
keeping one canonical source for the exponent.

This matches the contract already documented in the fork's
`isp_display_uint8_rgba.comp`: *"gamma > 0 adds an extra encode (gamma == 1 is
an identity encode and is skipped; the main chain already encodes in
isp.gamma)"* — the "main chain" is now the tone stage.

## TODO (deferred)

- `fullScale`: for `isp.display_uint8_rgba` / `isp.display_rgba_f16`, decide
  whether `VulkanFuse.cpp` should copy graph input[2] into `Uniforms[3]` so
  non-normalized (native sensor count) frames pack correctly. Currently
  guarded to 1.0, i.e. assumes `NormalizeBlock` already ran.
- `ToneLutBlock` is not in the profile chain; if it is wired in later, its
  `inv_gamma` default (1.0 = identity) must be driven from canonical
  `ToneParams::gamma` or the tonelut path will stay linear while the tone path
  encodes.
- Verify on-device that the fused `PackedRgb` path matches the primitive path
  pixel-for-pixel for a HEAVY profile after this change.

## Verification

- `cargo fmt --all -- --check`
- `cargo clippy -p cam-isp --lib --features rectifier -- -D warnings`
- `cargo test -p cam-isp --lib --features rectifier --no-fail-fast`
- `cargo check -p cam-isp --features "rectifier mnn"`
- New regression tests pin the identity-gamma rule.
