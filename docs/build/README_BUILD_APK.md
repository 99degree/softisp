# Building the SoftISP Camera APK

This document describes how to cross-compile the `cam-camera-app` crate (source in `android-app/`) into an Android APK.

## Prerequisites

### Required Toolchain
- **Rust**: stable (pinned via `rust-toolchain.toml`)
- **Android NDK**: r29 (29.0.14206865) at `$HOME/android-sdk/ndk/29.0.14206865`
- **Android SDK**: build-tools 34.0.0 at `$HOME/android-sdk/build-tools/34.0.0-termux`
- **Java**: **Java 11** (`/usr/lib/jvm/java-11-openjdk`) — **NOT Java 21**. cargo-apk2's R8 (D8) rejects class files with major version 65 (Java 21).
- **MNN libraries**: prebuilt in `cam-rust/lib/aarch64-v8a/` (libMNN.so, libMNN_Vulkan.so, libMNNConvertDeps.so, libvulkan.so, libc++_shared.so)

### MNN is a subproject — build its libs first

MNN lives at `~/MNN` and is **not part of this repository**. It is treated as a
subproject: the APK bundles prebuilt `libMNN*.so` files from
`cam-rust/lib/<abi>/`, and those must be current with the MNN source you are
testing. Always build MNN before building the APK so the fresh `.so` files are
copied into the runtime dir:

```bash
cd /data/data/com.termux/files/home/softisp

# Build MNN Vulkan libs and copy every .so into cam-rust/lib/aarch64-v8a/
./scripts/build_mnn_libs.sh arm64-v8a

# Verify the expected files landed:
ls -l cam-rust/lib/aarch64-v8a/libMNN.so \
     cam-rust/lib/aarch64-v8a/libMNN_Vulkan.so \
     cam-rust/lib/aarch64-v8a/libMNN_Express.so \
     cam-rust/lib/aarch64-v8a/libMNNConvertDeps.so \
     cam-rust/lib/aarch64-v8a/libc++_shared.so
```

`scripts/build_mnn_libs.sh` runs `build_vulkan_android.sh` inside `~/MNN`, then
copies every `libMNN*.so` plus `libMNNConvertDeps.so` into
`cam-rust/lib/aarch64-v8a/`. It also rewrites `libvulkan.so.1` → `libvulkan.so`
in `libMNN_Vulkan.so` / `libMNNConvertDeps.so` so the device's system ICD
loader resolves the Vulkan entry point (the APK deliberately does not ship its
own `libvulkan.so`).

If you rebuild MNN and forget this step, the APK ships stale libraries and
you will chase MNN-side bugs that are already fixed upstream.

### Environment Variables
```bash
export JAVA_HOME=/data/data/com.termux/files/usr/lib/jvm/java-11-openjdk
export ANDROID_NDK_ROOT=/data/data/com.termux/files/home/android-sdk/ndk/29.0.14206865
export ANDROID_HOME=/data/data/com.termux/files/home/android-sdk
```

## Build Steps

```bash
cd /data/data/com.termux/files/home/softisp/cam-rust

# Build the APK with cargo-apk2 (default debug profile).
cargo apk2 build -p cam-camera-app
```

### Low-RAM build (Termux / memory-constrained hosts)

On devices with limited RAM, the default `debug = true` profile produces
large DWARF debug info and `overflow-checks = true` inflates codegen,
both of which can push peak RSS past available memory and OOM the linker.

Build with the `release` profile instead (added in `cam-rust/Cargo.toml`):
```bash
cd /data/data/com.termux/files/home/softisp/cam-rust

# Release profile: debug = 1, no overflow checks, codegen-units = 1.
cargo apk2 build -p cam-camera-app --release
```

The `release` profile uses `opt-level = 2`, `debug = 1` (line tables only,
no full DWARF), `overflow-checks = false`, `codegen-units = 1`, and
`panic = "abort"`. This cuts peak RSS during codegen/linking roughly in
half at the cost of a slower single-threaded codegen pass — acceptable on
memory-constrained hosts.

The APK will be at `target/release/apk/softisp-camera.apk`.

## Required NDK/SDK Packages
- NDK r29 (29.0.14206865)
- SDK platform 34 (for targetSdkVersion=34)
- Build tools 34.0.0 (termux variant)

## External Storage Permission (MANAGE_EXTERNAL_STORAGE)

The app logs to `/sdcard/download/softisp-app.log`. On Android 11+ (API 30+), this requires the special "All files access" permission (`MANAGE_EXTERNAL_STORAGE`), which **cannot be granted programmatically** — the user must manually enable it:

1. Settings → Apps → SoftISP Camera
2. Tap "All files access" (or "Manage all files")
3. Toggle "Allow access to manage all files"

This is a platform restriction: a `NativeActivity` has no `Activity` subclass to launch the settings intent, so the app cannot self-request it. The code in `crashlog.rs` / `storage.rs` falls back to the app's external data directory (`/sdcard/Android/data/com.softisp.camera/files/softisp-app.log`) when the public Download directory is unavailable.

The manifest declares the permission:
```xml
<uses-permission android:name="android.permission.MANAGE_EXTERNAL_STORAGE"/>
<uses-permission android:name="android.permission.WRITE_EXTERNAL_STORAGE" android:maxSdkVersion="29"/>
<uses-permission android:name="android.permission.READ_EXTERNAL_STORAGE" android:maxSdkVersion="29"/>
```

## MNN Runtime Libraries

The APK bundles these native libraries (from `cam-rust/lib/aarch64-v8a/`) into `lib/arm64-v8a/`:
- `libMNN.so` — MNN core
- `libMNN_Vulkan.so` — Vulkan backend plugin (dlopen-ed by MNN at runtime)
- `libMNN_Express.so` — Express module (required by converter)
- `libMNNConvertDeps.so` — ONNX→MNN converter deps
- `libvulkan.so` — Android Vulkan ICD loader (Termux doesn't search `/system/lib64/`)
- `libc++_shared.so` — C++ runtime

These are copied by cargo-apk2 via `runtime_libs = "../lib"` in `cam-camera-app/Cargo.toml`.

## Neural ISP Rectifier ONNX Models (bundled in APK)

The pre-trained rectifier ONNX models are compiled into the APK binary via
`include_bytes!` (in `cam-rust/cam-isp/src/neural_controller.rs`):

- `isp-rectifier/models/fusedispcontroller_light_int8.onnx` (default, INT8, fastest)
- `isp-rectifier/models/fusedispcontroller_light.onnx` (FP32 fallback)

The neural controller (`NeuralController::with_default_model()`) tries these
embedded bytes at runtime. If none load (e.g. model file missing or corrupt),
it falls back to the rule-based `IspController` so the preview stays alive.
No separate `.onnx` file needs to ship in `lib/` — the models are part of
`libcam_camera_app.so`.

The rectifier feature is enabled for the APK via
`cam-isp = { ..., features = ["rectifier"] }` in
`android-app/Cargo.toml`.

## CI Gates (must pass locally before push)

```bash
# Format
cargo fmt --all -- --check

# Clippy (per-crate; cam-isp with rectifier feature)
for crate in cam-types cam-hal cam-hal-android cam-hal-linux cam-isp cam-core cam-onnx cam-motion cam-binder; do
  feat=""
  [ "$crate" = "cam-isp" ] && feat="--features rectifier"
  cargo clippy -p "$crate" --lib $feat -- -D warnings
done
cargo clippy -p cam-app --all-features -- -D warnings
cargo clippy -p cam-camera-app --all-features -- -D warnings

# Tests
cargo test -p cam-isp --lib --features "rectifier mnn"
```

## Key Implementation Notes

### MED Profile Output Format
The `PipelineProfile::MED` in `cam-isp/src/profile.rs` now uses `OutputFormat::Bgra` (was `PackedRgb`). This matches the app's `ProcessParams::output_format = OutputFormat::Bgra` in `cam-camera-app/src/pipeline.rs`, so the MNN graph emits 4-byte BGRA pixels that `WindowTarget::present()` can directly blit.

### Non-Stop Preview Loop
- `POLL_TIMEOUT = 10 ms` (was 100 ms) for responsive rendering
- `WindowTarget` is created once per window lifetime and reused (via `transmute` to `'static`) — recreating it per frame caused `set_buffers_geometry` thrashing and Vulkan/Gralloc crashes
- On ISP error, the last good frame is repainted so the surface never goes blank
- Camera open is retried indefinitely with 1/sec rate-limited error logging

### MNN Reshape Fix
`CalibrationBlock` stats graph in `cam-isp/src/blocks/stats.rs` uses `Squeeze(axes=[2,3])` instead of `Reshape` because MNN's Reshape size computer requires input content for the shape tensor, which isn't available at init.

### build.rs Dead Code Fixes
Added `#[allow(dead_code)]` to `setup_ndk_linker` and `target_ndk_arch` in `cam-isp/build.rs` to satisfy `-D warnings` in CI.

## Troubleshooting

### "Unsupported class file major version 65"
You're using Java 21. Switch to Java 11:
```bash
export JAVA_HOME=/usr/lib/jvm/java-11-openjdk
```

### "Failed to read source.properties"
Ensure `ANDROID_NDK_ROOT` points to the versioned NDK directory:
```bash
export ANDROID_NDK_ROOT=/path/to/android-sdk/ndk/29.0.14206865
```

### `Signing ... with keystore ...` then `Error: No such file or directory (os error 2)`

Two distinct causes, both silent — the build gets all the way through
zipalign and only fails at the final `apksigner sign` step.

**1. `apksigner` / `d8` have the wrong interpreter (Termux only).**

cargo-apk2 shells out to the build-tools wrapper scripts, which ship with
`#!/bin/bash`. Termux has no `/bin/bash` (bash is at `$PREFIX/bin/bash`), so
the exec fails with `ENOENT` — which surfaces as the misleading
`No such file or directory` error even though the keystore and APK both exist.
Rewrite the shebangs once per SDK install:

```bash
termux-fix-shebang $ANDROID_HOME/build-tools/34.0.0/apksigner \
                   $ANDROID_HOME/build-tools/34.0.0/d8
```

Check with `grep -n '^#!' $ANDROID_HOME/build-tools/34.0.0/apksigner`.

**2. A wrong `signing.release.path` in `Cargo.toml` (release builds only).**

cargo-apk2 resolves that path as `crate_path.join(path)` — relative to the
`cam-camera-app/` crate directory, **not** the workspace root. So
`../android-app/...` means `cam-rust/android-app/...`, and one more `../`
overshoots to the repo root. Debug builds never consult this table (they use
`~/.android/debug.keystore`), so a wrong path only shows up under `--release`.

### Black Screen / No Camera Frames
- Check logcat for `cam_isp::mnnengine: MNN inference failed` — indicates graph output dimension mismatch
- Verify `PipelineProfile::MED.output_format == OutputFormat::Bgra`
- Ensure `ProcessParams.output_format == OutputFormat::Bgra` in the app

### Storage Permission Denied in Logs
Expected on API 30+ until user grants "All files access". The app falls back to its private external directory automatically.