# Profile ↔ Block Comparison

How `cam-app` arranges ISP blocks per profile.

## How it works

`cam-app` calls `profile.build_blocks(width, bayer_pattern)` (`profile_builder.rs`).
That method walks the profile flags and appends either a **real block** or an **IdentityBlock** (passthrough placeholder) for each slot.

**Every profile produces exactly 24 main-chain blocks.** The difference is which slots are real vs identity.

## Block chain order (always 24 slots)

```
 #   Block slot               Real block when
───  ───────────────────────  ──────────────────────────────
 0   raw_input                always (packed_w if use_unpack, else full_w)
 1   unpack                   use_unpack=true  → UnpackBlock
 2   normalize                always            → NormalizeBlock
 3   cfa                      always            → CfaBlock
 4   blc                      always            → BlcBlock
 5   aux_hook_src             always (identity — stats tap point)
 6   lsc                      use_lsc=true      → CcmBlock (LSC via CCM)
 7   bayer_wb                 always            → BayerWbBlock
 8   demosaic                 always            → DemosaicBlock
 9   ccm                      always            → CcmBlock
10   warp                     use_warp=true     → extra CcmBlock (warp mesh)
11   tone                     always            → ToneBlock
12   aux_hook_out             always (identity — post-tone tap point)
13   fcs                      use_fcs=true      → FcsBlock
14   ldci                     use_ldci=true     → LdciBlock
15   ee                       use_ee=true       → EeBlock
16   bilateral                use_bilateral=true → BilateralBlock
17   vignetting               use_vignetting=true → VignettingBlock
18   saturation               use_saturation=true → SaturationBlock
19   colorspace               use_colorspace=true → ColorSpaceBlock
20   gamma                    use_gamma=true    → GammaBlock
21   sharpen                  use_sharpen=true  → SharpenBlock
22   wavelet_denoise          use_wavelet_denoise=true → WaveletDenoiseBlock
23   auto_contrast            use_auto_contrast=true → AutoContrastBlock
24   display                  always            → DisplayBlock
```

> When `use_unpack=false`, slot 0 is `RawInput(full_w)` and slot 1 is skipped (UnpackBlock not added), so the chain starts at 23 blocks. Slot indices shift accordingly.

> When `use_warp=true`, an extra CcmBlock is inserted at slot 10 (total 25 blocks).

## Per-profile flag table

| Flag                    | LITE | MED | HEAVY | PRO | TEST | UNIFIED | HDR |
|-------------------------|:----:|:---:|:-----:|:---:|:----:|:-------:|:---:|
| **Core**                |      |     |       |     |      |         |     |
| use_unpack              | Y    | Y   | Y     | Y   | Y    | **N**   | Y   |
| use_bad_pixel           | —    | Y   | Y     | Y   | —    | Y       | Y   |
| demosaic_quality        | HqL  | Std | Edge  | Edge| Std  | Std     | HqL |
| use_lsc                 | —    | —   | Y     | Y   | —    | Y       | Y   |
| use_warp                | —    | —   | —     | Y   | —    | —       | Y   |
| use_hdr                 | —    | —   | —     | Y   | —    | —       | Y   |
| **Post-processing**     |      |     |       |     |      |         |     |
| use_fcs                 | —    | —   | Y     | Y   | —    | Y       | Y   |
| use_ldci                | —    | —   | Y     | Y   | —    | Y       | —   |
| use_ee                  | —    | Y   | Y     | Y   | —    | Y       | —   |
| use_bilateral           | —    | —   | Y     | Y   | —    | Y       | —   |
| use_vignetting          | —    | —   | Y     | Y   | —    | Y       | Y   |
| use_saturation          | —    | Y   | —     | Y   | —    | Y       | —   |
| use_colorspace          | —    | —   | —     | Y   | —    | Y       | —   |
| use_gamma               | —    | —   | Y     | Y   | —    | Y       | —   |
| use_sharpen             | —    | —   | Y     | Y   | —    | Y       | —   |
| use_wavelet_denoise     | —    | —   | —     | Y   | —    | Y       | —   |
| use_auto_contrast       | —    | —   | —     | Y   | —    | Y       | —   |
| **Stats (aux)**         |      |     |       |     |      |         |     |
| use_zone_stats          | Y    | Y   | Y     | Y   | Y    | Y       | Y   |
| use_channel_means       | Y    | Y   | Y     | Y   | —    | Y       | Y   |
| use_tone_stats          | —    | Y   | Y     | Y   | —    | Y       | Y   |
| use_histogram           | —    | —   | Y     | Y   | —    | Y       | —   |
| **Tiling**              |      |     |       |     |      |         |     |
| use_tiled_rendering     | —    | —   | —     | —   | —    | Y       | —   |
| tile_count              | 1×1  | 1×1 | 1×1   | 1×1 | 1×1  | 2×2     | 1×1 |

## Actual block count per profile

| Profile | Main blocks | Aux blocks | Total | Notes |
|---------|:-----------:|:----------:|:-----:|-------|
| LITE    | 24          | 3          | 27    | zone_stats + channel_means + calibration |
| MED     | 24          | 4          | 28    | + tone_stats |
| HEAVY   | 24          | 5          | 29    | + tone_stats + histogram; stats_downscale_max=540 |
| PRO     | 25          | 5          | 30    | + warp (extra CcmBlock) |
| TEST    | 24          | 2          | 26    | zone_stats + calibration only |
| UNIFIED | 23          | 5          | 28    | no unpack (23 main); 2×2 tiled |
| HDR     | 25          | 4          | 29    | + warp (extra CcmBlock) |

## Per-profile real block map

### LITE — 7 real + 17 identity

```
[ 0] raw_input      ← RawInput (packed_w)
[ 1] unpack         ← UnpackBlock
[ 2] normalize      ← NormalizeBlock
[ 3] cfa            ← CfaBlock
[ 4] blc            ← BlcBlock
[ 5] aux_hook_src   ← IdentityBlock
[ 6] lsc            ← IdentityBlock
[ 7] bayer_wb       ← BayerWbBlock
[ 8] demosaic       ← DemosaicBlock
[ 9] ccm            ← CcmBlock
[10] tone           ← ToneBlock
[11] aux_hook_out   ← IdentityBlock
[12-23]             ← all IdentityBlock (fcs, ldci, ee, bilateral, vignetting,
                       saturation, colorspace, gamma, sharpen, wavelet, auto_contrast)
[24] display        ← DisplayBlock
```

### MED — 10 real + 14 identity

Adds: **EeBlock**, **SaturationBlock**, **ToneStatsBlock**

```
[ 6] ee             ← EeBlock
[18] saturation     ← SaturationBlock
```

### HEAVY — 16 real + 8 identity

Adds: **CcmBlock**(lsc), **FcsBlock**, **LdciBlock**, **EeBlock**, **BilateralBlock**, **VignettingBlock**, **GammaBlock**, **SharpenBlock**

```
[ 6] lsc            ← CcmBlock
[13] fcs            ← FcsBlock
[14] ldci           ← LdciBlock
[15] ee             ← EeBlock
[16] bilateral      ← BilateralBlock
[17] vignetting     ← VignettingBlock
[20] gamma          ← GammaBlock
[21] sharpen        ← SharpenBlock
```

### PRO — all 25 real (0 identity after aux hooks)

Every post-processing slot is a real block. Also enables **warp** (extra CcmBlock at slot 10).

```
[ 6] lsc            ← CcmBlock
[10] warp           ← CcmBlock (warp mesh)
[13] fcs            ← FcsBlock
[14] ldci           ← LdciBlock
[15] ee             ← EeBlock
[16] bilateral      ← BilateralBlock
[17] vignetting     ← VignettingBlock
[18] saturation     ← SaturationBlock
[19] colorspace     ← ColorSpaceBlock
[20] gamma          ← GammaBlock
[21] sharpen        ← SharpenBlock
[22] wavelet        ← WaveletDenoiseBlock
[23] auto_contrast  ← AutoContrastBlock
```

### TEST — identical to LITE

Same blocks, same flags. Minimal for fast unit tests.

### UNIFIED — 23 main (no UnpackBlock), all postproc real

Key difference: `use_unpack=false`, so slot 0 is `RawInput(full_w)` and **no UnpackBlock**.
Also: `use_normalize=true`, tiled 2×2.

```
[ 0] raw_input      ← RawInput (full_w, not packed)
[ 1] normalize      ← NormalizeBlock
[ 2] cfa            ← CfaBlock
[ 3] blc            ← BlcBlock
...
[13-23]              ← all real blocks
```

### HDR — 25 main (warp enabled), MED-level postproc

Per-frame: MED-quality ISP + warp for EIS alignment + FCS + vignetting.
HDR merge happens **after** per-frame ISP (not inside `build_blocks`).

```
[ 6] lsc            ← CcmBlock
[10] warp           ← CcmBlock (EIS alignment, margin=0.05)
[13] fcs            ← FcsBlock
[17] vignetting     ← VignettingBlock
```

## cam-app invocation

```bash
cam-app --profile heavy --width 1920 --convert-to-mnn
```

`build_pipeline()` in `main.rs`:
1. Picks `PipelineProfile::HEAVY`
2. Calls `profile.build_blocks(1920, 2)` → 24 blocks
3. Calls `GraphComposer::wire_blocks()` to connect block inputs/outputs
4. Calls `GraphComposer::compose_from_vec()` → ONNX model bytes
5. Optionally converts ONNX → MNN via `convert_onnx_to_mnn()`
