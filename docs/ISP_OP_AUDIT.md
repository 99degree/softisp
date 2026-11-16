docs: audit which `isp.*` names are real, on both sides

The question was whether `isp.unpack_blc` is one of many broken fusion names.
It is — 17 of the 30 names softisp can emit have no runnable shader in MNN.
This records the evidence so the next person does not have to rediscover it.

## MNN side

`probe_isp_op_audit` cross-checks every `isp.*` name against the three things
that decide whether it can run:

  whitelist   `IspOpsetValidate.hpp`  — gates the converter's TRAP check
  lookup      `IspSpvLookup.hpp`      — the embedded-SPIR-V table the Creator reads
  shader      `isp_spv/isp_<name>.comp` — implemented, or `// placeholder`

Of the 63 names the converter can emit, **42 are dead ends**: either the shader
is missing entirely (37) or it is a placeholder that copies its input (5).

Shipped SPIR-V count is 82 while the whitelist has 82 entries, so the mismatch is
not stale headers — the shaders themselves are stubs.

## softisp side

Names actually emitted from `src/blocks/` at runtime:

    isp.demosaic_ccm   placeholder
    isp.display        implemented
    isp.ee             placeholder
    isp.ldci           placeholder
    isp.unpack_blc     not in MNN at all

The other 25 names appear only in `src/pipeline/fusion_patterns.rs`, a table
nothing consumes (`IspFusionPatterns` is exported but never called). It is not a
source of bugs — but it is also not a source of anything.

## The one that works

`isp.crop_blc_norm_cfa` is whitelisted, embedded, implemented, and does exactly
what softisp's fused block describes: normalise, subtract per-channel BLC, and
unpack a 2x2 Bayer quad into planar `[1,4,H/2,W/2]`. Its `height_downscale`
attribute is ignored by the shader, and `blc_offsets`/`sensor_max` ride as
attributes the shader cannot read — so those need to become graph inputs
(`binding 5` for BLC, `binding 0` uniforms for sensorMax) before the block can
use it.

Enabling it is a separate change; this only records the state.
