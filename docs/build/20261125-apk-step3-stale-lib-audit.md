---
title: "APK build step 3 ships the previous app library"
created_at: 2026-11-25T00:00:00+00:00
status: reproduced; fix TODO
---

# APK build step 3 ships the previous app library

## Symptom

`./build-apk.sh` completes and signs the APK, but the packaged
`lib/arm64-v8a/libcam_camera_app.so` is the library from the *previous* build,
not the one just compiled. Step 3 of the script copies
`target/aarch64-linux-android/debug/libcam_camera_app.so` into
`cam-rust/lib/{arm64-v8a,aarch64-v8a}/` and patchelf's it **before** step 4 runs
`cargo apk2 build`. `cargo-apk2` packages `runtime_libs = "../cam-rust/lib"`,
so it bundles that pre-patched copy.

## Reproduction on 2026-11-25

Command: `./build-apk.sh 2>&1 | tail -80` (MNN built with the required
`scripts/build_mnn_libs.sh arm64-v8a`).

Artifacts after the build:

| File | Size | SHA-256 |
| --- | --- | --- |
| `cam-rust/lib/arm64-v8a/libcam_camera_app.so` (copied at 20:24 from the 17:40 build) | 25,763,641 | `98e2414ab98038abb52192c934dc18c00a9a1227070daf094d7771d0109d562d` |
| `target/aarch64-linux-android/debug/libcam_camera_app.so` (fresh, 20:51) | 113,719,008 | `fe3769556be4adfbd4143e9cac98e76ef3969fdf6e94563a35cfd8cb14886f3b` |
| `lib/arm64-v8a/libcam_camera_app.so` inside the signed APK | 25,763,641 | `98e2414ab98038abb52192c934dc18c00a9a1227070daf094d7771d0109d562d` |

The APK entry matches the stale runtime-lib copy, not the fresh build output.
The stale library still carries the unique `libsoftisp_cpp_shared.so` SONAME, so
the APK loads; the problem is that recent Rust/app changes can be absent from
the shipped binary.

## Root cause

The packaging source (`cam-rust/lib`) is populated before the Rust cdylib is
rebuilt. There is no post-build copy/patch step, so a rebuild that changes the
app library never reaches the APK until a *second* `./build-apk.sh` run.

## TODO fix

1. Build the cdylib explicitly first:
   `cargo build -p cam-camera-app --target aarch64-linux-android`.
2. Copy that fresh library into both `cam-rust/lib/arm64-v8a/` and
   `cam-rust/lib/aarch64-v8a/`.
3. Run the existing `patchelf --replace-needed libc++_shared.so
   libsoftisp_cpp_shared.so` step on the fresh copies (and on the MNN libs).
4. Run `cargo apk2 build -p cam-camera-app`, which will reuse the up-to-date
   Rust build and package the patched fresh library.
5. Verify with:

```bash
sha256sum target/aarch64-linux-android/debug/libcam_camera_app.so \
         cam-rust/lib/arm64-v8a/libcam_camera_app.so
unzip -p target/debug/apk/softisp-camera.apk \
       lib/arm64-v8a/libcam_camera_app.so | sha256sum
```

All three hashes must match. This remains unimplemented; it is recorded as
TODO P2e in `docs/overview/ROADMAP.md`.

## Target architecture: `anw/` replaces `libcam_camera_app.so`

Yes — per user direction, `anw/` is the intended owner of all application
code. `android-app/` is to become a non-code APK shell holding the manifest and
`Sources/res`, while `anw/` owns `android_main`, camera capture, pipeline,
presentation, storage, crash logging, and the `softisp_anw` cdylib. See
`docs/ipa/ANW_SPLIT_PLAN.md` and P2g in `docs/overview/ROADMAP.md`.

The current tree has not landed that migration yet: `anw/` has presentation and
the raw-frame `softisp_anw_process_and_present` ABI, but no `android_main` or
`ACameraManager` capture path; `./build-apk.sh` still builds `-p cam-camera-app`;
and `anw/Cargo.toml` still points at local `AndroidManifest.xml` and
`Sources/res` that should instead come from `android-app/`.

P2e is therefore only the interim diagnosis of the current packaging bug. Once
P2g lands, `libcam_camera_app.so` is retired and the APK must package a freshly
built `libsoftisp_anw.so` instead.
