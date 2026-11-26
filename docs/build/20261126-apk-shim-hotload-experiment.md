# APK launcher shim experiment (2026-11-26)

## Goal

Allow ISP/MNN edits to be tested without repacking the APK. Android starts a
tiny launcher cdylib; the real application cdylib built by `anw/` is loaded from
device-writable storage at runtime. The production APK is untouched.

## Architecture

- `android-shim-app/` is a new crate: `softisp-shim`, cdylib
  `libsoftisp_shim.so`. It has no ISP, camera, or MNN dependencies. The
  production `android-app/` directory is untouched.
- `anw/` still owns the camera/ISP application code and still produces
  `libsoftisp.so`. It is not packaged in the test APK.
- `android.app.lib_name` is `softisp_shim`. The stock `NativeActivity` launches
  the shim's `android_main`.
- The test APK contains exactly one native library: `libsoftisp_shim.so`. No
  MNN libraries and no application core are bundled.
- The shim:
  1. resolves the APK native-library directory from `/proc/self/maps`;
  2. copies all dependency `.so` files found in its search roots (normally
     `/sdcard/download`, where the user pushes MNN libs too) into
     `internal_data_path()` (already the app's `files/` directory) so the
     core's `$ORIGIN` rpath resolves;
  3. searches, in order: staged files dir, `/sdcard/download`,
     `/sdcard/Download`, app external data `files/`, app external data root,
     APK native-library dir, and `.`;
  4. copies the first found `libsoftisp.so` / `libsoftisp_core.so` /
     `libsoftisp_hot.so` into the staged dir as `libsoftisp.so`;
  5. `dlopen`s the staged copy, resolves `android_main`, and forwards the live
     `AndroidApp` by value.
- Every launch mirrors the complete search-root list to logcat (tag
  `softisp-shim`) and to `/sdcard/download/softisp-app.log`, falling back to
  app-private storage when the public path is not writable (for example after
  a reinstall changes the app UID). A missing core, failed `dlopen`, failed
  `dlsym`, or missing internal data path emits a `FATAL [shim]` line with the
  same root list and `dlerror()` text.

The public Download mount is `noexec` (`/storage/emulated` is mounted
`noexec`), so the core and its dependencies are always staged into app-private
files before `dlopen`; loading directly from Download would fail.

## Build scripts

- `./build-apk.sh` remains the production path: it builds MNN, builds the
  `anw/` core, and packages `libsoftisp.so` plus MNN libs into
  `target/debug/apk/softisp-camera.apk`. No behaviour change.
- `./build-test-apk.sh` builds the launcher-only APK:
  1. builds MNN libs with `scripts/build_mnn_libs.sh arm64-v8a` and strips them;
  2. builds the core with `cargo apk2 -- build -p softisp-anw`;
  3. builds the test APK with `cargo apk2 build -p softisp-shim`;
  4. verifies the APK contains exactly `lib/arm64-v8a/libsoftisp_shim.so`, no
     MNN/core libs, and that its hash matches
     `target/aarch64-linux-android/debug/libsoftisp_shim.so`.

Outputs:

- Production APK: `target/debug/apk/softisp-camera.apk`
- Test APK: `target/debug/apk/softisp-test.apk`
- Hot-swappable core: `target/aarch64-linux-android/debug/libsoftisp.so`
- MNN libs for the device: `cam-rust/lib/arm64-v8a/lib*.so`

## Device flow

1. Clean-install `softisp-test.apk` (the activity library name changed).
2. Copy to `/sdcard/download/`:
   - `target/aarch64-linux-android/debug/libsoftisp.so` as `libsoftisp.so`;
   - every `cam-rust/lib/arm64-v8a/lib*.so` (MNN and renamed libc++).
3. Launch the app.
4. Read `/sdcard/download/softisp-app.log`. Expect:
   - `INFO [shim] core search roots: ...`
   - `INFO [shim] staged N dependency libs into ...`
   - `INFO [shim] found core /sdcard/download/libsoftisp.so`
   - `INFO [shim] staged core ...`
   - `INFO [shim] forwarding android_main to ...`
   - followed by the normal `anw` startup lines.
5. For later ISP/MNN changes, rebuild with `./build-test-apk.sh` (or just
   `cargo apk2 -- build -p softisp-anw` for ISP-only changes), copy the new
   `libsoftisp.so` and any changed MNN libs to Download, and relaunch. The test
   APK is unchanged.

## Constraints and risks

- `android_main` uses the Rust ABI. The shim and core must be built from the
  same workspace/toolchain and the same `android-activity` version/features.
  A version skew would be a runtime ABI mismatch, not a clean `dlopen` error.
- `RTLD_LOCAL` is used so the core's own `ANativeActivity_onCreate` cannot
  replace the shim's entry point.
- The test APK has no bundled MNN libs by design; pushing them to Download is
  mandatory. A missing dependency produces `dlopen ... failed: cannot locate ...`
  with the staged path in the log.
- This is an experiment; the stable P2g packaging description in
  `docs/overview/ROADMAP.md` remains the reference for the production APK.

## Verification

- `cargo fmt --all -- --check`, `git diff --check`.
- `cargo clippy -p softisp-shim --all-features --no-deps -- -D warnings`.
- `cargo test -p softisp-shim --lib --no-fail-fast` (4 tests).
- `./build-test-apk.sh` succeeds; the test APK contains exactly one native
  library and no MNN/core libs.
- On-device confirmation is user-side: this harness only reads logs.
