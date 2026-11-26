# APK launcher shim experiment (2026-11-26)

## Goal

Allow ISP/MNN edits to be tested without repacking the APK. Android starts a
tiny launcher cdylib; the real application cdylib built by `anw/` is loaded from
device-writable storage at runtime. The production APK is untouched.

For the day-to-day commands, MD5 verification, and troubleshooting, use
[`SHIM_DEBUG_WORKFLOW.md`](SHIM_DEBUG_WORKFLOW.md).

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
  2. searches, in order: `/sdcard/download`, `/sdcard/Download`, app external
     data, APK native-library dir, and `.`;
  3. removes any `.so` files left in app-private `files/` by older shim builds;
  4. finds the first `libsoftisp.so` / `libsoftisp_core.so` /
     `libsoftisp_hot.so` and uses its parent directory as the runtime dir;
  5. computes a streaming MD5 of every `.so` in that directory and logs
     `INFO [shim] md5 <name> <hex> <bytes>` before loading;
  6. copies each runtime dependency and the core from the uploaded files into
     anonymous memfds and loads them with
     `android_dlopen_ext(..., ANDROID_DLEXT_USE_LIBRARY_FD)`, in dependency order
     before the core, so their `DT_SONAME`s satisfy the core's `DT_NEEDED`
     entries without a filesystem path search. The linker identity path is
     `native_dir.join(name)`;
  7. resolves `android_main` in the core handle and forwards the live
     `AndroidApp` by value.
- Every launch mirrors the complete search-root list to logcat (tag
  `softisp-shim`) and to `/sdcard/download/softisp-app.log`, falling back to
  app external data when the public path is not writable (for example after a
  reinstall changes the app UID). A missing core, failed memfd-backed `dlopen`, or
  failed `dlsym` emits a `FATAL [shim]` line with the same root list and
  `dlerror()` text.

The public Download mount is `noexec` (`/storage/emulated` is mounted
`noexec`), so a plain `dlopen(path)` cannot be used. `ANDROID_DLEXT_USE_LIBRARY_FD`
maps the ELF from an already-open descriptor instead, so nothing is copied into
app-private storage. `ANDROID_DLEXT_FORCE_LOAD` ensures the pushed files are
used even if a library with the same SONAME is already present in the process.
The identity path given to the linker must be inside the APK native-library
directory; the descriptor still supplies all bytes from the uploaded file.

## Build scripts

- `./build-apk.sh` remains the production path: it builds MNN, builds the
  `anw/` core, and packages `libsoftisp.so` plus MNN libs into
  `target/debug/apk/softisp-camera.apk`. No behaviour change.
- `./build-test-libs.sh` builds the runtime libraries only:
  1. builds MNN libs with `scripts/build_mnn_libs.sh arm64-v8a`;
  2. strips them and retunes their RUNPATH/SONAME dependencies;
  3. builds the core with `cargo apk2 -- build -p softisp-anw` into
     `target/aarch64-linux-android/debug/libsoftisp.so`.
  Use `--core-only` to skip the MNN rebuild for an ISP-only change.
- `./build-test-apk.sh` builds only the launcher-only APK:
  1. builds the shim with `cargo apk2 build -p softisp-shim`;
  2. verifies the APK contains exactly `lib/arm64-v8a/libsoftisp_shim.so`, no
     MNN/core libs, and that its hash matches
     `target/aarch64-linux-android/debug/libsoftisp_shim.so`.
- `./push-test-libs.sh` copies that freshly built core plus the MNN libs from
  `cam-rust/lib/arm64-v8a/` into `/sdcard/download/`. The app cannot read
  Termux's private `/data/data/com.termux` tree, so this copy step is mandatory.
  The emulated-storage FUSE mount rewrites copied files to a fixed shared-storage
  owner (`u0_a170:media_rw 0660` on this device), which may differ from the
  current app UID after a reinstall. **All files access** must be enabled for
  `com.softisp.camera` or the app cannot read the pushed libs; the symptom is
  `FATAL [shim] cannot open ... for loading: Permission denied (os error 13)`
  plus a `MediaProvider: Permission to access file ... is denied` line in
  logcat.
  The script rejects a core that still links `libc++_shared.so`.
- A plain `cargo build` is **not** a substitute. The host triple is
  `aarch64-linux-android`, so it writes `target/debug/libsoftisp.so` (which still
  links `libc++_shared.so`), not the APK-target core under
  `target/aarch64-linux-android/debug/` (which links the renamed
  `libsoftisp_cpp_shared.so`). Quick core-only iteration is:

  ```
  ./build-test-libs.sh --core-only && ./push-test-libs.sh && ./shim-debug.sh reload
  ```

  No APK rebuild or reinstall is needed. The shim must load directly from
  `/sdcard/download`; older shim builds that copied libs into app-private
  storage must be rebuilt/reinstalled once
  (`docs/isp/20261126-shim-staged-core-shadowing-debug.md`).
- The MNN libs keep the Termux build-host RUNPATHs after
  `scripts/build_mnn_libs.sh arm64-v8a`. `build-test-libs.sh` rewrites them to
  `$ORIGIN` with `patchelf`; without that, the core loads its direct MNN deps but
  `libMNNConvertDeps.so` cannot find its own `libMNN_Express.so`, producing
  `dlopen failed: library "libMNN_Express.so" not found ... in namespace clns-7`.
  `push-test-libs.sh` verifies the `$ORIGIN` RUNPATH before copying.

Outputs:

- Production APK: `target/debug/apk/softisp-camera.apk`
- Test APK: `target/debug/apk/softisp-test.apk`
- Hot-swappable core: `target/aarch64-linux-android/debug/libsoftisp.so`
- MNN libs for the device: `cam-rust/lib/arm64-v8a/lib*.so`

## Device flow

1. The **user** installs `softisp-test.apk` (the activity class changed from
   `android.app.NativeActivity` to `com.softisp.camera.ShimActivity`) and enables
   **All files access** for `com.softisp.camera` in Settings. Reinstalling
   assigns a new UID, so without this the app cannot read files Termux copies
   into shared storage. The assistant never runs `install-apk.sh` or any
   `pm`/`am` package-lifecycle command (see `AGENTS.md` §1.7).
2. Copy to `/sdcard/download/`:
   - `target/aarch64-linux-android/debug/libsoftisp.so` as `libsoftisp.so`;
   - every `cam-rust/lib/arm64-v8a/lib*.so` (MNN and renamed libc++).
3. Launch the app.
4. Read `/sdcard/download/softisp-app.log`. Expect:
   - `INFO [shim] core search roots: ...`
   - `INFO [shim] removed N legacy staged .so files from ...`
   - `INFO [shim] found core /sdcard/download/libsoftisp.so`
   - `INFO [shim] loading runtime libraries from /sdcard/download`
   - `INFO [shim] loading /sdcard/download/libMNN.so as <apk native dir>/libMNN.so`
   - `INFO [shim] md5 libsoftisp.so <hex> <bytes>`
   - `INFO [shim] forwarding android_main to /sdcard/download/libsoftisp.so`
   - followed by the normal `anw` startup lines.
5. For later ISP/MNN changes, run
   `./build-test-libs.sh --core-only && ./push-test-libs.sh`, then
   `./shim-debug.sh reload`. The test APK is unchanged; only the pushed core
   needs refreshing.

## MD5 assurance and intent control

Every uploaded library is MD5-fingerprinted before `dlopen`. The log lines look
like `INFO [shim] md5 libsoftisp.so <32 hex> 134881689 bytes`. An external
script computes `md5sum` of its own build outputs and compares the two lists;
a mismatch means the device is running a stale or substituted library.

The activity uses the rare action `com.softisp.camera.action.CONTROL` with
`android:launchMode="singleTask"`, so a debug command reaches the live activity
instead of creating a second instance. The stock `android.app.NativeActivity`
does **not** override `onNewIntent()` or call `setIntent()`, so `getIntent()`
keeps returning the original launcher intent and the native watcher never sees
later commands. The test APK therefore compiles a tiny Java subclass,
`com.softisp.camera.ShimActivity`, whose `onNewIntent()` calls `setIntent(intent)`.
`hasCode` is `true` and `java_sources = "java"` is set in the shim crate's Cargo
manifest so cargo-apk2 compiles it into `classes.dex`.

```
am start -a com.softisp.camera.action.CONTROL \
  -n com.softisp.camera/com.softisp.camera.ShimActivity \
  --es softisp_cmd stop
```

Recognised values:

- `ping` — log a confirmation line.
- `cancel` — `dlsym` the core's exported `softisp_anw_cancel_pipeline()` and
  call it, so the ISP pipeline stops without killing the app.
- `clean` — delete legacy `.so` files from app-private `files/`. `reload` and
  `stop` also do this before exiting.
- `stop` / `kill` / `quit` / `exit` — call `Activity.finish()`, then force a
  process exit after two seconds. This is needed because Termux's shell UID
  cannot use `am force-stop`.

`ShimActivity` also sets `FLAG_SHOW_WHEN_LOCKED`, `FLAG_TURN_SCREEN_ON`,
`FLAG_DISMISS_KEYGUARD` and `FLAG_KEEP_SCREEN_ON` on both `onCreate()` and
`onNewIntent()`, so a control intent can bring the preview up while the device
is locked. Whether this is reliable in unattended runs is still under study
(TODO). A control intent cannot reach a **stale process** that was started from
an older APK without the Java subclass; that needs a one-time force-stop done
by the user.

`./auto-debug.sh [attempts] [wait_secs]` automates the whole loop: clear log,
launch, wait, compare logged MD5s with local `md5sum`, and on mismatch send the
`stop` intent, rebuild and re-push the core, then relaunch. Results land in
`.tmp/auto-debug/`. The user installs the rebuilt APK.

## Constraints and risks

- `android_main` uses the Rust ABI. The shim and core must be built from the
  same workspace/toolchain and the same `android-activity` version/features.
  A version skew would be a runtime ABI mismatch, not a clean `dlopen` error.
- `RTLD_LOCAL` is used so the core's own `ANativeActivity_onCreate` cannot
  replace the shim's entry point.
- The test APK has no bundled MNN libs by design; pushing them to Download is
  mandatory. A missing dependency produces `fd-backed dlopen ... failed` with
  the uploaded path in the log.
- All files access is mandatory after a reinstall; without it the shim finds the
  core but fails with `cannot open ... for loading: Permission denied (os error 13)`
  before `dlopen`.
- This is an experiment; the stable P2g packaging description in
  `docs/overview/ROADMAP.md` remains the reference for the production APK.

## Verification

- `cargo fmt --all -- --check`, `git diff --check`.
- `cargo clippy -p softisp-shim --all-features --no-deps -- -D warnings`.
- `cargo test -p softisp-shim --lib --no-fail-fast` (7 tests, including MD5
  known-answer vectors and dependency-order coverage).
- `./build-test-libs.sh` and `./build-test-apk.sh` succeed; the test APK
  contains exactly one native library and no MNN/core libs.
- On-device confirmation is user-side: this harness only reads logs.
