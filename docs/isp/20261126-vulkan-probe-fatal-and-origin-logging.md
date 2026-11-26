# 2026-11-26 — Vulkan probe FATAL and `$ORIGIN` diagnostics

## Symptom

The APK log at `/sdcard/download/softisp-app.log` ends both recent launches with:

```
11-26 05:22:58.858 FATAL [engine]    2764ms engine.build failed: MNN error: probe session create fail (Vulkan backend unavailable)
11-26 05:27:04.803 FATAL [engine]    2361ms engine.build failed: MNN error: probe session create fail (Vulkan backend unavailable)
```

Both launches reach `MNN build backend=mnn_vulkan`, convert the ONNX graph, then
fail before `SessionPool` is built. The camera session is closed immediately
afterwards.

## Reproduction

Install/launch the current APK. No camera permission or external storage
permission issue precedes the failure — the log file is created and the camera
capture session becomes active first.

## Diagnosis

### Not a library-loading or rpath failure

`logcat -d -b main,system,crash` shows the Vulkan loader and the Adreno ICD both
initialise correctly at the failing timestamp:

```
D vulkan  : searching for layers in '/data/app/~~.../com.softisp.camera-.../lib/arm64'
D vulkan  : searching for layers in '/data/app/~~.../com.softisp.camera-.../base.apk!/lib/arm64-v8a'
I AdrenoVK-0: Driver Path : /vendor/lib64/hw/vulkan.adreno.so
I MNNJNI  : [VulkanLimits] ... gpu=Adreno (TM) 618
I MNNJNI  : [MNN-ISP-SESSION] model tensorNumber=0 oplists=157
```

There is no `dlopen failed`, no `cannot locate symbol`, no missing-`.so` line,
and no tombstone. `libsoftisp.so` is mapped from the private
`nativeLibraryDir` and all `DT_NEEDED` libraries load. Consequently the
`Vulkan backend unavailable` text is misleading: it is the generic message
attached to `create_session()` returning `None` in `mnnengine.rs`, not evidence
of a missing Vulkan library.

### Root cause

The line immediately before the FATAL is the real error:

```
11-26 13:27:04.683 E MNNJNI : Interpreter don't support case for shape compute need input content, please use module api instead
```

`f3c35f5a` (`fix(isp): apply rotation in fused display graph`) replaced the
`isp.display_*` `rotate_mode` attribute with ordinary graph nodes. For 90°/270°
and flips it appends `Slice` nodes whose `starts`/`ends`/`axes`/`steps` are
**runtime graph inputs** (`DisplayBlock/{hflip,vflip}_{starts,ends,axes,steps}`).
MNN computes `Slice` output shapes during `resizeSession`, before the engine can
write those tensors, so `Interpreter::createSession` returns null. The same
failure mode is already documented in
`cam-rust/cam-isp/src/blocks/adaptive_downscale.rs` for resize-scale inputs.

Supporting evidence: the converted ONNX size jumped from 15285 bytes (last
working launch, 11-25 22:16) to 15524 bytes (both failing launches), matching
the extra rotation nodes; the graph-input count stayed at 51 because those
tensors were converted from attributes to inputs.

## Fix applied

Added a boot-time diagnostic in `anw/src/app.rs` (`log_origin()`) that resolves
`$ORIGIN` from the mapped path of `libsoftisp.so` in `/proc/self/maps` and lists
the `.so` files visible there:

```
INFO [softisp::app] loader $ORIGIN: /data/app/~~.../com.softisp.camera-.../lib/arm64
INFO [softisp::app] libs visible in $ORIGIN: ...
```

This will confirm on the next launch that the private `nativeLibraryDir` is
`lib/arm64` (Android flattens the APK's `lib/arm64-v8a`), so the added
`$ORIGIN/lib/{arm64-v8a,aarch64-v8a}` rpaths point to non-existent subdirs on
this device. They are harmless but not required; plain `$ORIGIN` is what makes
the bundled libs resolve.

## Outstanding

The rotation/`Slice` regression is **not fixed** by this change. The next step
is to stop feeding `Slice` shape parameters as runtime inputs — e.g. emit them
as constant initializers sized from the known preview geometry, or replace the
`Slice` nodes with an attribute-driven primitive — so `createSession` no longer
needs input content for shape inference.
