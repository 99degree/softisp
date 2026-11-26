# Fast on-device debug with `android-shim-app` (2026-11-26)

This is the practical runbook for iterating on ISP/MNN code without rebuilding
and reinstalling the full APK on every change. The production path
`./build-apk.sh` and `android-app/` are unchanged; everything here is the
test-only `softisp-test.apk` built from `android-shim-app/`.

The core idea is a tiny launcher (`libsoftisp_shim.so`) that stages a separately
pushed `libsoftisp.so` plus its MNN dependencies into app-private storage and
`dlopen`s them. Every staged library is MD5-fingerprinted into the log, so a
local script can prove the device is running the exact binaries just built.

For background and design rationale, see
[`20261126-apk-shim-hotload-experiment.md`](20261126-apk-shim-hotload-experiment.md).

---

## 0. Rules that are not negotiable

- **The user owns installation.** Build the APK, hand over the path, and let the
  user install it through the normal Android flow. Never run
  `install-apk.sh`, `pm install`, `pm uninstall`, `pm clear`, `am force-stop`,
  `am kill`, `pm disable`, or any other package/process lifecycle command from
  the harness (see `AGENTS.md` §1.7 and §3).
- `am start` is allowed for launching the activity and for sending this app's
  own control intent. That is how the app stops or cancels itself without
  shell-level force-stop permissions.
- A process started from an older APK (before `ShimActivity`) cannot receive
  control intents. It needs a one-time Force stop done by the user.
- Reinstalling changes the app UID. Re-enable **All files access** for
  `com.softisp.camera` after every reinstall, or the app cannot read files in
  `/sdcard/download`.

---

## 1. What is where

| Item | Path |
| --- | --- |
| Test APK | `target/debug/apk/softisp-test.apk` |
| Production APK (untouched) | `target/debug/apk/softisp-camera.apk` |
| Hot-swappable core | `target/aarch64-linux-android/debug/libsoftisp.so` |
| Shim native lib | `target/aarch64-linux-android/debug/libsoftisp_shim.so` |
| MNN + renamed libc++ libs | `cam-rust/lib/arm64-v8a/lib*.so` |
| Device staging directory | `/sdcard/download/` |
| App log | `/sdcard/download/softisp-app.log` |
| Activity component | `com.softisp.camera/com.softisp.camera.ShimActivity` |
| Control action | `com.softisp.camera.action.CONTROL` |
| Control extra | `softisp_cmd` |
| Script scratch output | `.tmp/shim-debug/` (gitignored) |

The test APK contains exactly one native library,
`lib/arm64-v8a/libsoftisp_shim.so`. It deliberately bundles no MNN library and
no `libsoftisp.so`. `build-test-apk.sh` verifies this and fails if the invariant
is broken.

---

## 2. Scripts available

All scripts live at the repository root and are executable.

| Script | Purpose |
| --- | --- |
| `./shim-debug.sh <cmd>` | One-entry wrapper for build/push/launch/control/logs/verify/cycle (see §6) |
| `./build-test-apk.sh` | Build MNN, core, and the shim-only APK; verify its contents |
| `./push-test-libs.sh` | Copy core + MNN libs into `/sdcard/download` |
| `./auto-debug.sh [attempts] [wait_secs]` | Full automatic MD5/self-restart loop |
| `./scripts/build_mnn_libs.sh arm64-v8a` | The only supported MNN build path (always `build_isp`, never `build_vk`) |

A quick core-only rebuild must use the APK target:

```bash
cargo apk2 -- build -p softisp-anw && ./push-test-libs.sh
```

A plain `cargo build` writes `target/debug/libsoftisp.so` and links
`libc++_shared.so`; it is the wrong binary and `push-test-libs.sh` rejects it.

---

## 3. One-time setup

1. Build the shim-only APK and the core:

   ```bash
   ./build-test-apk.sh
   ```

   This runs `scripts/build_mnn_libs.sh arm64-v8a`, strips and `$ORIGIN`-retunes
   the MNN libs, builds `libsoftisp.so` with the NDK target, builds
   `softisp-test.apk`, then checks the APK hash and native-lib contents.

2. Give `target/debug/apk/softisp-test.apk` to the user to install. Tell them to
   enable **All files access** for `com.softisp.camera` immediately after
   install.

3. Push the core and runtime libs:

   ```bash
   ./push-test-libs.sh
   ```

4. Launch and collect the first log:

   ```bash
   ./shim-debug.sh launch
   sleep 12
   ./shim-debug.sh logs
   ```

   Expected in `/sdcard/download/softisp-app.log`:

   ```text
   INFO [shim] core search roots: ...
   INFO [shim] staged N dependency libs into ...
   INFO [shim] found core /sdcard/download/libsoftisp.so
   INFO [shim] staged core ...
   INFO [shim] md5 libsoftisp.so <hex> <bytes>
   INFO [shim] forwarding android_main to ...
   ```

   followed by the normal `anw` startup lines. A `FATAL [shim]` line includes
   the full search-root list and `dlerror()` text.

5. Verify the fingerprints:

   ```bash
   ./shim-debug.sh verify
   ```

   It computes local `md5sum` values for `libsoftisp.so` and every
   `cam-rust/lib/arm64-v8a/lib*.so`, then compares them with the `md5` lines in
   the app log. PASS means the device is running exactly the local build.

---

## 4. Daily loop: ISP/MNN code changes

No APK rebuild and no reinstall is needed for a core-only change.

```bash
# 1. rebuild only the app core and push it
./shim-debug.sh core

# 2. stop the running app through its own control intent
./shim-debug.sh stop

# 3. launch again
./shim-debug.sh launch

# 4. wait, collect logs, compare MD5s
WAIT_SECS=12 ./shim-debug.sh logs
./shim-debug.sh verify
```

Or do all four steps in one command:

```bash
WAIT_SECS=12 ./shim-debug.sh cycle
```

`cycle` sends `stop`, waits three seconds, rebuilds and pushes the core,
launches, waits `WAIT_SECS`, dumps logs, and runs the MD5 verification.

If you changed MNN libraries themselves, rebuild and retune them first with
`./build-test-apk.sh` (or rerun `scripts/build_mnn_libs.sh arm64-v8a` and the
`patchelf` step from `build-test-apk.sh`), then push with
`./shim-debug.sh push`.

---

## 5. When the shim itself changes

If you edit `android-shim-app/src/lib.rs`, `AndroidManifest.xml`, `Cargo.toml`,
or `java/com/softisp/camera/ShimActivity.java`, you must rebuild and reinstall
the APK:

```bash
./build-test-apk.sh
```

Then the user installs `target/debug/apk/softisp-test.apk` and re-enables All
files access if the UID changed. A stale process that predates
`ShimActivity` cannot receive the control intent; the user must force-stop it
once from Settings before the new control path is live.

---

## 6. `shim-debug.sh` command reference

```bash
./shim-debug.sh help
```

| Command | What it does |
| --- | --- |
| `build` | Runs `./build-test-apk.sh` |
| `push` | Runs `./push-test-libs.sh` |
| `core` | `cargo apk2 -- build -p softisp-anw` then `push` |
| `launch` | `am start -n com.softisp.camera/com.softisp.camera.ShimActivity` |
| `ping` | Logs a control-intent confirmation; use to check delivery |
| `cancel` | Calls the core's `softisp_anw_cancel_pipeline` after in-flight work |
| `stop` | Asks the app to `finish()` and exit; no shell force-stop |
| `logs` | Dumps logcat and app log to `.tmp/shim-debug/`, prints a tail |
| `verify` | MD5 comparison between local build outputs and the app log |
| `cycle` | `stop` → `core` → `launch` → wait → `logs` → `verify` |

`WAIT_SECS` controls how long `launch`/`cycle` wait before reading logs
(default 12).

The raw control commands, if you prefer to type them yourself:

```bash
am start -a com.softisp.camera.action.CONTROL \
  -n com.softisp.camera/com.softisp.camera.ShimActivity \
  --es softisp_cmd ping

am start -a com.softisp.camera.action.CONTROL \
  -n com.softisp.camera/com.softisp.camera.ShimActivity \
  --es softisp_cmd cancel

am start -a com.softisp.camera.action.CONTROL \
  -n com.softisp.camera/com.softisp.camera.ShimActivity \
  --es softisp_cmd stop
```

`cancel` is for stopping the ISP loop without killing the activity. `stop` is
for replacing the core with a fresh build.

---

## 7. Fully automatic loop

```bash
./auto-debug.sh 3 12
```

It clears the log, launches, waits, verifies MD5s, and on mismatch sends the
`stop` intent, rebuilds and re-pushes the core, then relaunches. Results land in
`.tmp/auto-debug/`. Use `./shim-debug.sh cycle` for one clean pass; use
`auto-debug.sh` when you want repeated retries.

---

## 8. Troubleshooting

| Symptom | Likely cause | Action |
| --- | --- | --- |
| `FATAL [shim] cannot stage core ... Permission denied` | All files access not granted after reinstall | User enables All files access for `com.softisp.camera` |
| `dlopen failed: library "libMNN_Express.so" not found` | MNN libs still have Termux RUNPATHs | `./build-test-apk.sh` (runs `patchelf --set-rpath '$ORIGIN'`) |
| `verify` reports stale MD5s | Device is running an old core, or stop did not reach the app | `./shim-debug.sh ping`; if no confirmation, user force-stops once, then relaunch |
| No `[shim]` lines in the log | Process started from an older APK / core log rotated | Check `logcat -d -s softisp-shim:I`; `./shim-debug.sh logs`; relaunch fresh |
| Control intent does nothing | Stale `NativeActivity` process without `ShimActivity` | User installs the rebuilt APK and force-stops the old process once |
| APK contains more than one native lib | Production packaging path was used | Use `./build-test-apk.sh`, not `./build-apk.sh` |
| Core push fails with `libc++_shared.so` error | Host `cargo build` binary was used | `cargo apk2 -- build -p softisp-anw` |

The public log rotates at 512 KiB, so after a long run the startup `[shim] md5`
lines may no longer be present. Relaunch to regenerate them before verifying.

Locked-screen wake flags are compiled into `ShimActivity`, but their reliability
for unattended runs is still under study; do not depend on it yet.

---

## 9. Debugging the Vulkan backend ("probe session create fail")

The MNN engine's `build()` creates a probe Vulkan session. If it returns `null`,
the engine fails with:

```
FATAL [engine] engine.build failed: MNN error: probe session create fail (Vulkan backend unavailable)
```

This can be caused by any of: `libMNN_Vulkan.so` not being found, `libvulkan.so`
not resolving, or the Vulkan driver failing to initialize. The shim workflow
makes this easy to iterate on because you can swap the Vulkan `.so` files
without repacking the APK.

### 9.1 What to check in the log

Look for these `INFO`/`WARN` lines from `cam_isp::mnnengine`:

```
libs visible in $ORIGIN: libMNN.so, libMNNConvertDeps.so, libMNN_Express.so, libMNN_Vulkan.so, libsoftisp.so, libsoftisp_cpp_shared.so, libvulkan.so
MNN build backend=mnn_vulkan
MNN engine loaded (22768 bytes, backend=Vulkan, 4 sessions, 36 extra inputs)
```

If `libs visible in $ORIGIN` does **not** list `libvulkan.so`, the Vulkan ICD
loader is missing from `/sdcard/download/`. Copy it there:

```bash
# From the device system Vulkan loader
cp /system/lib64/libvulkan.so /sdcard/download/libvulkan.so

# Or from Termux (if it has a compatible build)
cp /data/data/com.termux/files/usr/lib/libvulkan.so.1.4.368 /sdcard/download/libvulkan.so
```

Then relaunch via `./shim-debug.sh launch` — no APK reinstall needed.

If `MNN engine loaded` never appears but `MNN build backend=mnn_vulkan` does,
the probe session creation failed. Check logcat for `softisp-shim` or
`cam_isp::mnnengine` lines showing `dlopen` results.

### 9.2 Pushing a fresh core + libs after a Vulkan fix

```bash
# 1. Rebuild only the app core (no APK repack)
./shim-debug.sh core

# 2. Stop the running app via its own control intent
./shim-debug.sh stop

# 3. Launch again (shim reloads the new core + libs)
./shim-debug.sh launch

# 4. Wait, dump logs, verify MD5 fingerprints
WAIT_SECS=12 ./shim-debug.sh logs
./shim-debug.sh verify
```

Or one command: `WAIT_SECS=12 ./shim-debug.sh cycle`.

### 9.3 If `libMNN_Vulkan.so` cannot be dlopen'ed

The shim's `$ORIGIN`-based RUNPATH means all MNN libs must live in the same
directory as `libsoftisp.so` on the device. `push-test-libs.sh` verifies
`libMNN_Vulkan.so` has `$ORIGIN` RUNPATH (retuned by `build-test-apk.sh` via
`patchelf --set-rpath '$ORIGIN'`). If a manually copied lib lacks this, nested
dependencies like `libMNN_Express.so` will fail with:

```
dlopen failed: library "libMNN_Express.so" not found ... in namespace clns-7
```

Fix by rebuilding: `./build-test-apk.sh` (which runs the `patchelf` step), then
`./shim-debug.sh push`.

### 9.4 If the Vulkan driver itself fails

If all libraries load but `probe session create fail` persists, the device's
Vulkan driver may not support the features MNN needs. Try:

1. Confirm `libvulkan.so` in `/sdcard/download/` is the device system copy
   (`/system/lib64/libvulkan.so`, SONAME `libvulkan.so`), not a Termux build
   (Termux's `libvulkan.so.1` has SONAME `libvulkan.so.1` and will not match
   `libvulkan.so` NEEDED entries).
2. Check `logcat -d` for Vulkan driver errors (may require `READ_LOGS` grant).
3. Test with a minimal MNN Vulkan model via the `test_warp_vulkan` example
   on host to isolate driver vs. graph issues.

---
