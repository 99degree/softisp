# Plan: make `anw/` the application code owner

> Status: implemented. Tracked as P2g in `docs/overview/ROADMAP.md`.

## Goal

`android-app/` becomes the non-code APK shell. It keeps the Android manifest,
`Sources/res`, and cargo-apk2 packaging metadata that points at those files.
All Rust/native application code moves to `anw/`, which produces the
`softisp` cdylib (`libsoftisp.so`) loaded by NativeActivity. The existing
`softisp_anw_*` C ABI symbol names are kept for compatibility; only the library
filename changes.

The shipped application identity stays `com.softisp.camera` / `softisp-camera`
unless the user explicitly chooses a new package ID. This is a code-location
change, not a new app.

## Target layout

### `android-app/` — resources and packaging shell only

- Keep `AndroidManifest.xml` and `Sources/res/**`.
- Keep no camera, ISP, pipeline, storage, crash-log, or presentation code.
- Remove the `cam-camera-app` Rust library target and `src/**`.
- Remove/move the linker setup in `build.rs`; it is application build code.

### `anw/` — all application code

- Keep `presentation.rs` as the ANativeWindow/viewport owner.
- Move in `android-app/src/camera.rs`, `crashlog.rs`, `pipeline.rs`,
  `pipeline_runner.rs`, `storage.rs`, and the `android_main` entry point from
  `lib.rs`.
- Keep the existing C ABI (`softisp_anw_init/start/stop`,
  `softisp_anw_process_and_present`) for callers that hand in raw frames.
- Own the cargo-apk2 metadata: package `com.softisp.camera`, APK name
  `softisp-camera`, `manifest_override = "../android-app/AndroidManifest.xml"`,
  `resources = "../android-app/Sources/res"`, and
  `runtime_libs = "../cam-rust/lib"`.
- NativeActivity `android.app.lib_name` becomes `softisp`, which loads
  `libsoftisp.so`.
- Own the Android linker build script with the flags currently in
  `android-app/build.rs`.

## Behaviour that must be preserved

- Final production output remains `OutputFormat::PackedRgb` with
  `isp.display_uint32_bgra` as the last op.
- Dual GPU engine still runs and is tested as a dual engine.
- Staged `prepare` / `run_prepared` / `finish` phases remain available.
- No float fallback or `FloatRgb should not happen` path in production.
- AHB remains reserved for large tensors; small tensors stay non-AHB.
- Crash log path and storage permission behaviour stay unchanged.

## Build changes

- `build-apk.sh` builds the APK from `anw`, not `cam-camera-app`.
- The stale `libcam_camera_app.so` copy/patch step is removed. cargo-apk2 builds
  the cdylib itself and packages the artifact from `target/`, so copying a
  separate library into `cam-rust/lib` would never affect the APK.
- `scripts/libsoftisp-rustc-wrapper.sh` runs `patchelf --replace-needed
  libc++_shared.so libsoftisp_cpp_shared.so` on the freshly linked `softisp`
  cdylib before cargo-apk2 copies it into the APK. `runtime_libs` remains only
  for MNN's prebuilt libraries.
- Verify that the target cdylib and APK entry hashes match, that the APK
  contains `lib/arm64-v8a/libsoftisp.so` and no `libcam_camera_app.so`, and that
  no app library is installed into `cam-rust/lib`.

## Verification

- Workspace format/clippy/tests with `anw` as the application crate.
- `./build-apk.sh` completes and the signed APK has the expected native lib.
- On-device logs (`/sdcard/download/softisp-app.log` and `logcat`) show camera
  startup, dual-engine ISP execution, and preview presentation from `anw`.
- Assistant does not install or share the APK; the user installs and launches it.
