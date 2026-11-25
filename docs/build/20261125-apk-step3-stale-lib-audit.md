---
title: "APK build step 3 ships the previous app library"
created_at: 2026-11-25T00:00:00+00:00
status: resolved by P2g
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

## Resolution (P2g)

The interim copy-into-`cam-rust/lib` fix was superseded. cargo-apk2 builds the
application cdylib itself and takes it from `target/`; changing a separate copy
in `cam-rust/lib` cannot affect that artifact. The shipped fix therefore:

1. Moves application code to `anw/` and produces `libsoftisp.so`.
2. Uses `scripts/libsoftisp-rustc-wrapper.sh` to apply
   `patchelf --replace-needed libc++_shared.so libsoftisp_cpp_shared.so` to the
   freshly linked cdylib before cargo-apk2 packages it.
3. Keeps `runtime_libs` in `anw/Cargo.toml` for MNN libraries only and removes
   all app-library copies from `cam-rust/lib`.
4. Verifies with:

```bash
sha256sum target/aarch64-linux-android/debug/libsoftisp.so
unzip -p target/debug/apk/softisp-camera.apk \
       lib/arm64-v8a/libsoftisp.so | sha256sum
```

Both hashes must match, and the APK must contain no `libcam_camera_app.so`.
P2e is closed by P2g; see `docs/overview/ROADMAP.md`.

## Target architecture: `anw/` replaces `libcam_camera_app.so`

Per user direction, `anw/` is now the owner of all application code.
`android-app/` is a non-code APK shell holding only the manifest and
`Sources/res`; `anw/` owns `android_main`, camera capture, pipeline,
presentation, storage, crash logging, and the `softisp` cdylib
(`libsoftisp.so`). See `docs/ipa/ANW_SPLIT_PLAN.md` and P2g in
`docs/overview/ROADMAP.md`.

P2e remains the historical diagnosis of the stale-library packaging bug; it is
closed by P2g. `libcam_camera_app.so` is no longer built or bundled.
