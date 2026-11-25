# Plan: Split ANativeWindow UI from android-app into anw/

## Goal
Split the ANativeWindow presentation logic from `android-app/` into the standalone `anw/` crate so it can grow into its own APK with a richer UI (egui). The `anw/` crate will own:
- ANativeWindow buffer lock/present logic (moved from android-app/src/preview.rs)
- Viewport/aspect-ratio letterbox logic
- cam-ipa IPA engine integration (process frames through IPA pipeline)
- Optional egui UI overlay for future rich UI

## Files to modify/create

### 1. anw/Cargo.toml — Add egui dependency and Android APK metadata
- Add `egui` + `egui_winit` + `winit` dependencies (feature-gated behind `egui`)
- Add `android-activity` workspace dep
- Add `[package.metadata.android]` for APK packaging (like android-app/)
- Keep `cam-ipa` dependency for IPA pipeline processing

### 2. anw/src/lib.rs — Expand from FFI stubs to full ANativeWindow consumer
- Move `WindowTarget`, `Viewport`, `Rect`, `BYTES_PER_PIXEL` from android-app/src/preview.rs
- Implement `present_planar_rgb` and `present` (BGRA → RGBA conversion)
- Implement `paint_bars`
- Keep existing `softisp_anw_init/start/stop` FFI exports
- Add `egui` integration (feature-gated)

### 3. anw/src/presentation.rs (new) — Presentation logic extracted
- `WindowTarget` struct
- `Viewport` struct with `fit()`
- `present_planar_rgb` / `present` methods
- `paint_bars` method

### 4. anw/AndroidManifest.xml (new) — Standalone APK manifest
- NativeActivity pointing to `softisp_anw` lib
- CAMERA permission
- Same metadata as android-app/Cargo.toml

### 5. anw/build.rs (new) — cargo-apk2 packaging config
- Similar to android-app/build.rs but for anw crate

### 6. android-app/ — Remove ANativeWindow presentation code (keep camera capture)
- Remove `preview.rs` or reduce to thin wrapper that calls into anw
- Keep camera capture pipeline (camera.rs, pipeline.rs, pipeline_runner.rs)
- The android-app becomes the capture + IPA processing frontend
- anw becomes the display/consumer frontend

### 7. softisp/Cargo.toml — Add anw to workspace members (already there)

## Build & Test
- `cargo build -p softisp_anw` — compiles on host (Android target for APK)
- `cargo build -p cam-camera-app` — android-app still builds (may need preview.rs adjusted)
- APK: `./build-apk.sh` builds cam-camera-app APK
- Future: `cargo apk2 build -p softisp_anw` builds anw APK

## Risks
- android-app/src/preview.rs is ~390 lines; moving it requires careful API boundary design
- egui on Android NativeActivity requires winit integration — may need additional setup
- cam-ipa IpaContext needs to be shared between android-app (capture) and anw (display) — may need IPC or shared memory