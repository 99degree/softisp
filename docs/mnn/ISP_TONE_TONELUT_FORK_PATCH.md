# MNN fork: `isp.tone` / `isp.tonelut` Vulkan shader contract

All Vulkan shaders live in the MNN fork (`libMNN_Vulkan.so`). softisp does **not**
ship standalone `.comp`/`.spv` files; it emits the `isp.*` ONNX nodes with
runtime tensors and relies on MNN's embedded SPIR-V lookup. This document is the
contract the MNN fork must satisfy for softisp's fused tone stages.

## softisp emission (already implemented)

- `ToneBlock` in `MnnIspMin` / `Custom` mode emits one `isp.tone` node with
  inputs `[frame, ToneBlock/exposure, ToneBlock/contrast, ToneBlock/brightness,
  ToneBlock/inv_gamma]` and `elementwise=1`. Gamma is owned by the tone stage —
  `isp.display*` must receive `gamma=1.0` so its encode is an identity.
- `ToneLutBlock` in `MnnIspMin` / `Custom` mode emits one `isp.tonelut` node with
  inputs `[frame, ToneLutBlock/y0, ToneLutBlock/s0..s5, ToneLutBlock/contrast,
  ToneLutBlock/brightness, ToneLutBlock/inv_gamma]` and `elementwise=1`.
- All coefficient tensors are written by `MnnEngine::set_extra_inputs()`;
  slopes `s0..s5` are derived on the CPU. No scalar derivation is in the graph.

## Binding map (universal `isp.*` rule in `VulkanFuse.cpp`)

| Binding | Contents |
|---|---|
| 0 | params SSBO — width/height hot-swapped by `elementwise=1` |
| 3 | frame in (`InBuf`, planar RGB float) |
| 4 | frame out (`OutBuf`, planar RGB float) |
| 5.. | extra inputs in declaration order |

## MNN fork changes required

### `isp.tone` — update the embedded shader

Current fork shader (`isp_spv/isp_tone.comp`) reads `gamma`/`contrast` from
binding-0 `Uniforms[2..3]`, processes only channel 0, and applies
`exp(log(v) * u.gamma) * contrast`. This does not match softisp's inputs and
silently drops two colour planes.

Required shader behaviour (compile and replace `g_isp_tone_spv` in
`IspSpvLookup.hpp`):

```glsl
#version 450
layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;
layout(set = 0, binding = 0, std430) buffer Uniforms {
    float width; float height; float pad0; float pad1;
} u;
layout(set = 0, binding = 3) readonly  buffer InBuf  { float in_data[]; };
layout(set = 0, binding = 4) writeonly buffer OutBuf { float out_data[]; };
layout(set = 0, binding = 5) readonly  buffer ExposureBuf  { float exposure[]; };
layout(set = 0, binding = 6) readonly  buffer ContrastBuf  { float contrast[]; };
layout(set = 0, binding = 7) readonly  buffer BrightnessBuf { float brightness[]; };
layout(set = 0, binding = 8) readonly  buffer InvGammaBuf  { float inv_gamma[]; };

void main() {
    uint x = gl_GlobalInvocationID.x, y = gl_GlobalInvocationID.y;
    int w = int(u.width), h = int(u.height);
    if (int(x) >= w || int(y) >= h) return;
    int plane = w * h, p = int(y) * w + int(x);
    float ig = max(inv_gamma[0], 1e-6);
    for (int ch = 0; ch < 3; ++ch) {
        float v = clamp(in_data[ch * plane + p]
                        * max(exposure[0], 0.05) * contrast[0] + brightness[0],
                        0.0, 1.0);
        out_data[ch * plane + p] = pow(v, ig);
    }
}
```

Gamma is applied here (`pow(v, 1/γ)` via the CPU-derived `inv_gamma` input),
matching `ToneBlock`'s primitive `Pow` node. The old fork shader's `u.gamma`
binding-0 exponent was removed; `GammaBlock` is no longer emitted in the
profile chain. The patched `isp_tone.comp` / `isp_tone_spv.h` are already staged
in the local MNN checkout.

### `isp.tonelut` — add a new embedded shader

The fork has no `isp.tonelut`. Add a shader with the same binding skeleton
above. Extra inputs are bound one-per-graph-input by VulkanFuse, so the six
slopes need six separate buffers at bindings 6..11 (a single `slopes[]` buffer
at binding 6 would only receive `s0`):

```glsl
layout(set = 0, binding = 5) readonly buffer Y0Buf  { float y0[]; };
layout(set = 0, binding = 6) readonly buffer S0Buf  { float s0[]; };
// ... bindings 7..11 for s1..s5
layout(set = 0, binding = 12) readonly buffer ContrastBuf   { float contrast[]; };
layout(set = 0, binding = 13) readonly buffer BrightnessBuf { float brightness[]; };
layout(set = 0, binding = 14) readonly buffer InvGammaBuf   { float inv_gamma[]; };

const float SEG_W = 1.0 / 6.0;
// ... per channel, after the bounds check:
float slopes[6] = float[6](s0[0], s1[0], s2[0], s3[0], s4[0], s5[0]);
float v = in_data[ch * plane + p];
float lut = y0[0];
for (int i = 0; i < 6; ++i)
    lut += clamp(v - float(i) * SEG_W, 0.0, SEG_W) * slopes[i];
v = clamp(lut * contrast[0] + brightness[0], 0.0, 1.0);
out_data[ch * plane + p] = pow(max(v, 1e-6), max(inv_gamma[0], 1e-6));
```

Then:
1. Compile to SPIR-V and add `g_isp_tonelut_spv` / `g_isp_tonelut_spv_len` to
   `IspSpvLookup.hpp` and register `{"isp.tonelut", {...}}` in `g_ispSpvMap`.
2. Add `"isp.tonelut"` to `kIspOpsetNames` in `IspOpsetValidate.hpp` so
   `MNN_ISP_TRAP=1` runs stay clean.

These changes are staged in the local MNN checkout at `~/MNN` on `build-branch`
(not committed there). Rebuild `libMNN_Vulkan.so` after applying them.

No `VulkanFuse.cpp` change is needed: `scanSpvBindings()` derives the frame
bindings from the SPIR-V and the universal map assigns extra inputs from 5.

## TODO — check `isp.display*` tone/gamma absorption

> **TODO (deferred):** verify whether any `isp.display*` shader already absorbs
> tone-stage work (exposure / contrast / brightness / LUT) in addition to gamma.
> Current inspection of `isp_display.comp`, `isp_display_rgb.comp`, and
> `isp_display_uint8_argb.comp` shows gamma only (`pow(v, 1/gamma)`), no tone
> terms. If a display variant does absorb tone, the `isp.tone` / `isp.tonelut`
> direct-emit stages may be redundant on that path and should be skipped to
> avoid applying the curve twice. Re-check after the fork shaders are updated.

## Verification

- `cargo test -p cam-isp --lib --features rectifier` — fused node emission tests.
- `cargo check -p cam-isp --features "rectifier mnn"` — MNN-gated path compiles.
- On a device with the patched `libMNN_Vulkan.so`: run the FUSED profile and
  compare the tone/tonelut output against the primitive path pixel-by-pixel.
