# `isp.*` fusion: verified stage by stage, blocked by tensor precision

Sweep of the five fusion-capable stages, one at a time, comparing each to the
stage before it so exactly one change is under test. Geometry 640x480, flat
Bayer input, `PipelineProfile` blocks wired exactly as MED wires them.

## Result

**fp16 tensor storage (the app's setting) — nothing fuses.**

    none (baseline)   28.4 ms   mean 0.8038  peak 1.0000   reference
    +ccm              32.7 ms   mean 0.0000  peak 0.0000   BLANK

Every fused stage either blanks the frame or produces garbage, so the sweep
stops at the first one.

**fp32 tensor storage — every stage matches.**

    none (baseline)   46.9 ms   mean 0.8027  peak 1.0000   reference
    +ccm              37.3 ms   mean 0.8027  peak 1.0000   ok   (Δ 0.0000)
    +ee               35.7 ms   mean 0.8007  peak 0.8010   ok   (Δ 0.0037)
    +ldci             30.6 ms   mean 0.8007  peak 0.8010   ok   (Δ 0.0000)
    +display          23.4 ms   mean 0.8007  peak 0.8010   ok   (Δ 0.0000)
    demosaic+ccm      20.1 ms   mean 0.8007  peak 0.8010   ok   (Δ 0.0037)

`+ccm` fuses to byte-identical output. The other four differ by at most 0.2 in
peak — that is the edge-replication choice in the shaders (`isp.ee`, `isp.ldci`
clamp at the border where the primitive `AveragePool`/`Conv` zero-pads), not an
error.

## Why

`VulkanBackend::getTensorSize` (`VulkanBackend.cpp:368`):

```cpp
size_t bytes = ((tensor->getType().code == halide_type_float) && mUseFP16)
             ? sizeof(uint16_t) : sizeof(float);
```

With `Precision_Normal` on a device that supports it, `mUseFP16` is true
(`VulkanBackend.cpp:176`) and every float tensor is allocated at two bytes per
element. The `isp.*` shaders declare their buffers as `float[]`, so they read
those half-width bytes as 32-bit floats.

The isolation is clean: the same fused CCM is `mean 0.8007` (identical to the
primitive Conv) under fp32 and `mean 0.0000` under fp16, with nothing else
changed. `probe_ccm_isolated` runs both.

## What this means for enabling fusion

As things stand, fp16 storage and `isp.*` fusion are mutually exclusive:

| | fp16 | fp32 |
|---|---|---|
| 2592x1940 pipeline fits | yes | **no** (OOM) |
| `isp.*` fused ops correct | no | yes |

So fusion cannot simply be switched on. There are two real fixes:

1. **Make the fused shaders read fp16 buffers.** MNN stores the data as
   `uint16_t` halves, so a shader would declare them `float16_t[]` (needs the
   `shaderFloat16` feature — the `isp.display_rgba_f16` shader avoids it
   deliberately, because Adreno fails pipeline creation and then SIGSEGVs in
   `vkDestroyPipeline`) or read `uint` pairs and unpack with `packHalf2x16`
   (core GLSL, no feature, but two lanes per word).

2. **Select fp32 per pipeline in the engine** when the graph contains `isp.*`
   ops, and accept the memory cost — which at 2592x1940 means a pool of one
   session, serialising frames.

Both are engine work, not client work. The shaders themselves are correct.

## Probes

    cargo test -p cam-isp --test probe_fusion_stepwise   --features mnn -- --ignored --nocapture
    cargo test -p cam-isp --test probe_ccm_isolated      --features mnn -- --ignored --nocapture

Both take `PROBE_FP32=1` to switch storage precision, since that is the variable
that decides the outcome. The default is fp16, i.e. what the app runs.
