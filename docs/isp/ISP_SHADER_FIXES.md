docs(isp): three placeholder shaders are now real

`isp.ee`, `isp.ldci` and `isp.demosaic_ccm` were `// placeholder compute
shader` stubs that copied their input, so enabling any of them silently deleted
the stage. All three are now implemented against the same maths as their
primitive forms:

  isp.ee             depthwise 3x3 unsharp conv per channel, kernels and bias
                     from binding 5/6 (the block's own 3.0/-0.5 kernel). Edge
                     pixels are replicated rather than zero-padded, so the frame
                     does not gain a dark rim at the border.
  isp.ldci           out = in + strength * (in - 3x3 local mean), strength from
                     binding 5 — the exact Pool/Sub/Mul/Add sequence the block
                     emits primitively.
  isp.demosaic_ccm   1x1 conv from the 4-plane CFA quad to planar RGB using the
                     engine's baked CCM x demosaic weights (binding 5) and bias
                     (binding 6), with the clip the primitive form expresses as a
                     separate node.

All three declare the frame at binding 3/4, which is what the SPIR-V binding
scan in VulkanFuse now reads, so no client-side change was needed. The embed
headers were regenerated from the new sources (3044 -> 4456/4908/4696 bytes).

## Still blocked

`probe_direct_opset` still reports MnnIspMin as a black frame. The reason is now
narrower and is not in these three shaders: `isp.display` receives no W/H. Its
`Uniforms.gamma` is never written by anything, and the dispatch geometry only
gets corrected through the `elementwise` attribute, which no softisp block sets.

Setting `elementwise` on the display node fails `resizeSession` with
COMPUTE_SIZE_ERROR instead — so the remaining work is in MNN's resize path for a
`const`-bearing Extra op, not in the shaders.

## What the deeper pass found elsewhere

Of the 81 shaders, 18 placeholders remain (aaf, auto_contrast, bilateral, bnf,
cct, ceh, cnf, colorspace, fcs, hdr_merge, nlm, pyramid, stats, tone_stats,
temporal_denoise, yuv_bcc, yuv_sat, zone_stats). Sixteen of them back blocks
`PipelineProfile` can enable (bilateral, colorspace, auto_contrast, fcs,
tone_stats, zone_stats), so any profile that turns those on under MnnIspMin will
delete them silently.

Binding-0 layout is consistent: 75 shaders start their params block with
`{width, height}`. The six that do not are the `algo_*` statistics ops (which
start with `{count, ...}`) and `isp.lsc` (which uses `vec2 wh` — the same two
floats, different declaration). That is fine for the W/H hot-swap, which writes
floats 0/1 regardless of whether they are two scalars or one vec2.
