# Default-on AHB output rotation report — findings (2026-11-26)

## Symptom

After the signed default-on AHB APK was launched, the user reports that the pipeline output looks rotated. This is a new regression report following the switch from opt-in host-copy output to default-on AHB-backed output.

## Reproduction

1. Install `target/debug/apk/softisp-camera.apk`.
2. Launch the camera app from the launcher or `am start`.
3. Observe the preview: the image is reported as rotated.

## Root cause

The device log confirms `sensor_orientation=90` and `pipeline rotate_mode=3`, so the app deliberately asks the pipeline to counter-rotate. The production chain is `PipelineProfile::FUSED` with `OutputFormat::PackedRgb`, which selects the fused `isp.display_uint32_bgra` path. The MNN fork's `isp.display*` shaders do **not** implement rotation; `docs/isp/20261126-isp-display-absorption-audit.md` records that finding. `DisplayBlock::emit_mnn_isp_min_nodes` was passing `rotate_mode` as an attribute anyway, so the graph declared a rotated output shape while the shader wrote pixels in the sensor's original orientation. The host and AHB presenters both read those pixels as-is; the image therefore appears sideways. This is not introduced by making AHB output default-on — the switch only changed where the unrotated pixels are read from.

## Fix applied

`DisplayBlock::emit_mnn_isp_min_nodes()` now writes the fused display shader into an intermediate tensor and emits the same `Transpose` / `Slice` nodes the primitive path uses, ending on `DisplayBlock/frame`. The `rotate_mode` attribute handed to the shader is zero so a future shader-side rotation cannot double-apply the transform. When no rotation is requested the shader still writes straight to the final tensor, so the common path keeps its single dispatch. Regression tests cover both the no-op and every rotate mode.

## Findings

- Latest device app log: `.tmp/log-grab-20261126-0605/softisp-app.log.1` records `output AHB=on`, `sensor_orientation=90`, `pipeline rotate_mode=3`, and a steady `800x600` output geometry.
- The same log repeatedly reports `AHB output binding lost on 'DisplayBlock/frame'; falling back to host copy`. That is a separate zero-copy binding defect; it does not change orientation, because the fallback writes into the same AHB mapping.
- Neither the AHB slot nor the presenter contains rotation logic. Rotation belongs in the graph so both host-copy and AHB paths receive the same pixels.
- The binding-loss warning has since been addressed by oversizing the AHB slot
  to the fused graph's worst case, verifying the pointer is a real
  `AHardwareBuffer`, avoiding in-run resize, and adding detailed `-10`
  diagnostics (`docs/isp/20261126-output-copy-zero-copy-audit.md`). A new
  signed APK was rebuilt after both fixes.
- On-device validation is still pending: confirm the preview is upright, the
  reported output geometry is `600x800` for the `800x600` sensor frame with
  `rotate_mode=3`, and the `AHB output binding lost` warning no longer appears.
