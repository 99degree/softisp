# Fused rotation regression: `last 600x2`

Date: 2026-11-26

## Symptom

With a non-zero `rotate_mode` (reported as mode 3 / 270°) the device preview
degenerates to a single bright line instead of a picture:

```text
11-26 07:48:41.748 INFO [stats] 10 frames processed, last 600x2,
rgb mean [0.1,0.1,0.1], max 207, non-black 0.1%
```

The same run logs the display output buffer as:

```text
out_elems=1200 max_out=33554432 need=4800
```

and the model input as `[1,1,1200,800]`. `1200` words is exactly a `600x2`
packed-word image, so the graph output geometry collapsed to two columns.

## Reproduction

1. Build the runtime libs and shim test APK (`./build-test-libs.sh`,
   `./build-test-apk.sh`), then upload with `./push-test-libs.sh`.
2. Install the APK (user-side), push the core to `/sdcard/download`, launch
   `com.softisp.camera/com.softisp.camera.ShimActivity`, and wait for the
   pipeline to process ~10 frames.
3. Read `/sdcard/download/softisp-app.log`. With `rotate_mode = 3` the stats
   line reports `last 600x2`; the expected swapped geometry is `400x600`
   (or `600x400` before rotation, depending on the post-CCM tail).

Local unit tests do **not** reproduce the failure:

```bash
cargo test -p cam-isp --lib --features rectifier --no-fail-fast
# 858 passed; 0 failed; 2 ignored
```

The existing tests assert only node names and initializer presence:
`fused_display_applies_rotation_as_graph_nodes` and
`rotation_slice_params_are_initializers_not_runtime_inputs` in
`cam-rust/cam-isp/src/blocks/display.rs`. They never check the tensor shape
after rotation or that the pixels are non-degenerate.

## Suspect commits

- `f3c35f5a` — introduced the fused `isp.display_uint32_bgra` rotation path.
- `63ef80f4` — changed flip parameters from runtime inputs to initializers to
  survive MNN shape inference.

Neither was reverted; the fix must be structural, not a silent rollback.

## Root-cause analysis

The fused path in `DisplayBlock::emit_mnn_isp_min_nodes()` (before this fix):

1. Runs `isp.display_uint32_bgra` first, producing a **packed UINT32** tensor
   `[1,1,H,W]` (`one uint32 per pixel`).
2. For 90°/270°, emits `Transpose(perm=[0,1,3,2])` → `[1,1,W,H]`.
3. For flips, emits ONNX `Slice` nodes with 5 inputs
   (`data, starts, ends, axes, steps`) and int64 initializers.

MNN converts ONNX `Slice` to `StridedSlice`. In this checkout:

- `tools/converter/source/optimizer/onnxextra/OnnxSlice.cpp` maps the 5-input
  ONNX form to a `StridedSlice` with `fromType = 0`.
- `source/shape/ShapeStridedSlice.cpp` early-returns a copied shape when
  `fromType == 0 && inputs.size() == 5`, and its non-ellipsis `fromType == 0`
  branch reads `inputs[3]` as **steps** rather than the axes tensor. The
  explicit `axes=[2]`/`axes=[3]` initializers are therefore not applied the way
  the ONNX semantics require.

For rotate mode 3 the transpose makes the flip axis ambiguous to MNN's shape
computer; the resulting output shape collapses instead of staying `[1,1,W,H]`.
The same mechanism explains why a `Slice` on the packed display word tensor is
especially fragile: the shader has already packed BGRA into one word, so a
wrong axis/selection yields a narrow strip of valid words and everything else
is dropped.

The AHB path loss (`AHB output binding lost`, `rc=-2`) logged in the same run
is a separate issue. It is not the cause of the `600x2` geometry: the fused
output is only 1200 words regardless of which backend wrote it. It is tracked
separately in `docs/isp/20261126-ahb-output-binding-loss-debug.md`.

## Fix applied

Orientation is now a **standalone `RotateBlock`**, not part of `DisplayBlock`:

- Insert it in `profile_builder` immediately before `DisplayBlock` while the
  frame is still float RGB `[1,3,H,W]`.
- Decompose the transform into only `Transpose` and flips: 90°/270° is one
  `Transpose(perm=[0,1,3,2])`, and a flip is one `Gather(axis=2|3)` with a
  reversed int64 index initializer. This avoids both the packed-tensor
  `Slice → StridedSlice` failure and any float sampling grid. Arbitrary angles
  are explicitly not supported here; they belong to a later GPU warp stage.
- Remove `rotate_mode`, `with_rotate()`, the transpose/Slice emission, and the
  flip initializers from `DisplayBlock`. `DisplayBlock` now owns only format
  conversion/packing and is told the rotated input dimensions.
- Add geometry/content regression tests for every rotate mode. Asserting node
  names is not enough; the test must verify `out_h/out_w` and that output
  pixels are a permutation of the input rather than a degenerate row.

## Verification plan

```bash
cargo fmt --all -- --check
cargo test -p cam-isp --lib --features rectifier --no-fail-fast
cargo clippy -p cam-isp --lib --features rectifier -- -D warnings
./build-test-libs.sh --core-only
./build-test-apk.sh
```

## Verification status (2026-11-26)

Focused local checks passed after the rewrite:

```bash
cargo fmt --all -- --check
cargo test -p cam-isp --lib --features rectifier --no-fail-fast
cargo clippy -p cam-isp --lib --features rectifier -- -D warnings
./build-test-libs.sh --core-only
./build-test-apk.sh
```

After the user cleared the app data/cache, the installed APK was checked and
contains only `lib/arm64-v8a/libsoftisp_shim.so`; it has no bundled
`libsoftisp.so`, `libMNN*.so`, or `libc++_shared.so`. The running shim then found
the new core in `/sdcard/download`, staged it into app-private `files/`, and
logged:

```text
INFO [shim] md5 libsoftisp.so 3d8fcb0b9439710d9fae9ab28b68b6e1 134816153 bytes
```

`./shim-debug.sh verify` reported `PASS (7 libraries)`. In the fresh launch
segment after that stage, there were zero `last 600x2` lines, the stats showed
`last 400x600`, the display tensor had `out_elems=240000`, and there were no
`FATAL [shim]` lines. The separate AHB binding warning remains tracked in
`docs/isp/20261126-ahb-output-binding-loss-debug.md`.

This run is **not yet rotate-mode-3 acceptance**: camera permission was denied
after the data clear (`ACameraManager_openCamera -> PERMISSION_DENIED (-10013)`),
so the app fell back to `sensor_orientation=0, pipeline rotate_mode=0`.
Termux cannot grant the runtime permission (`pm grant` fails with
`SecurityException`); the user must grant Camera permission, then relaunch with
`./shim-debug.sh reload`. Final acceptance still requires a log segment with
`rotate_mode=3`, no `last 600x2`, and the expected swapped full-frame geometry.

## Follow-up: camera2 orientation mapping was inverted by 180°

After Camera permission was granted, the device log reported
`sensor_orientation=90, pipeline rotate_mode=3` and the preview geometry became
the expected `600x800`. The user then reported that the image needs one more
half-turn to fit the display. That is the signature of the metadata-to-block
mapping, not of the graph: mode 3 and the correct mode 1 differ by exactly 180°,
while both produce the same swapped dimensions.

`anw/src/app.rs` treated `ACAMERA_SENSOR_ORIENTATION` as a mounting angle that
had to be counter-rotated:

- 90° → mode 3 (`Transpose` + VFlip, i.e. 90° CCW / 270° CW)
- 270° → mode 1 (`Transpose` + HFlip, i.e. 90° CW)

Camera2 defines the field differently: it is the **clockwise angle through
which the output image must be rotated to be upright in the device's native
orientation**. `RotateBlock` modes 1/2/3 are clockwise quarter turns, so the
metadata maps directly:

- 90° → mode 1 (the old mapping was 180° away)
- 180° → mode 2 (unchanged)
- 270° → mode 3 (the old mapping was 180° away)

The fix replaces the ad-hoc match with a single tested helper and logs the
chosen mode. No rotation graph code changes: `RotateBlock` remains standalone,
uses only `Transpose` + `Gather`, and `DisplayBlock` remains packing-only.

This mapping assumes the activity is in the device's native orientation, which
is what the current fixed pipeline build does. If display-rotation tracking is
added later, the required clockwise angle becomes
`(sensor_orientation - display_degrees).rem_euclid(360)`; that is a separate
enhancement and must not be folded into this fix by guessing a rotation that is
not read from the window.

### Verification expected after relaunch

```bash
cargo fmt --all -- --check
cargo test -p softisp-shim --lib --no-fail-fast
cargo clippy -p softisp-shim --all-features --no-deps -- -D warnings
cargo test -p softisp-anw --lib --no-fail-fast
./build-test-libs.sh --core-only
./build-test-apk.sh
./push-test-libs.sh
./shim-debug.sh reload
./shim-debug.sh verify
```

On device the fresh launch segment must log `sensor_orientation=90,
pipeline rotate_mode=1`, keep the full-frame `600x800` geometry, and contain no
`FATAL [shim]`. MD5 verification must still pass for all uploaded libraries.
