# Building the SoftISP Camera APK

This document describes how to cross-compile the `cam-camera-app` crate into an Android APK.

## Prerequisites

### Required Toolchain
- **Rust**: stable (pinned via `rust-toolchain.toml`)
- **Android NDK**: r29 (29.0.14206865) at `$HOME/android-sdk/ndk/29.0.14206865`
- **Android SDK**: build-tools 34.0.0 at `$HOME/android-sdk/build-tools/34.0.0-termux`
- **Java**: **Java 11** (`/usr/lib/jvm/java-11-openjdk`) — **NOT Java 21**. cargo-apk2's R8 (D8) rejects class files with major version 65 (Java 21).
- **MNN libraries**: prebuilt in `cam-rust/lib/aarch64-v8a/` (libMNN.so, libMNN_Vulkan.so, libMNNConvertDeps.so, libvulkan.so, libc++_shared.so)

### Environment Variables
```bash
export JAVA_HOME=/data/data/com.termux/files/usr/lib/jvm/java-11-openjdk
export ANDROID_NDK_ROOT=/data/data/com.termux/files/home/android-sdk/ndk/29.0.14206865
export ANDROID_HOME=/data/data/com.termux/files/home/android-sdk
```

## Build Steps

```bash
cd /data/data/com.termux/files/home/softisp/cam-rust

# Build the APK with cargo-apk2
cargo apk2 build -p cam-camera-app
```

The APK will be at `target/debug/apk/softisp-camera.apk`.

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

### Black Screen / No Camera Frames
- Check logcat for `cam_isp::mnnengine: MNN inference failed` — indicates graph output dimension mismatch
- Verify `PipelineProfile::MED.output_format == OutputFormat::Bgra`
- Ensure `ProcessParams.output_format == OutputFormat::Bgra` in the app

### Storage Permission Denied in Logs
Expected on API 30+ until user grants "All files access". The app falls back to its private external directory automatically.