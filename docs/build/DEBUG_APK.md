# Debugging a SoftISP Camera APK Crash / Failure

The SoftISP camera app runs as an Android **NativeActivity** — there is no Java
layer, no `Activity` subclass, and **no `adb` on the dev machine** (the Termux
device is the test machine). You install the APK manually, launch it from the
launcher, and diagnose failures from the on-device crash log file plus `logcat`
run directly on the device.

> **Termux rule:** do not reach for `adb`. Everything below runs on the device
> itself (Termux shell on the same phone).

---

## 0. Build the APK (first)

See [`docs/build/README_BUILD_APK.md`](README_BUILD_APK.md) — short version:

```bash
cd /data/data/com.termux/files/home/softisp

# 1. Build MNN libs first (MNN is a subproject; see section 6).
./scripts/build_mnn_libs.sh arm64-v8a

# 2. Build the APK
./build-apk.sh

APK=target/debug/apk/softisp-camera.apk
ls -lh "$APK"
```

The debug APK uses `~/.android/debug.keystore` for signing and ships
`debuggable="true"` so logcat shows its output.

---

## 1. Install the APK (let the user install it — no `install-apk.sh` / `pm` in Termux)

Do **not** run `install-apk.sh`, `pm install-create`, `pm install-write`, or
`pm install-commit` from Termux. The APK is signed with the debug keystore and
is ready to sideload as-is:

```
target/debug/apk/softisp-camera.apk
```

Transfer that file to the device however you normally move files from Termux
(copy to `~/shared`, `termux-share`, etc.) and let the user install it through
the normal Android install flow (open the APK file, or drop it via the device's
file manager). The app id is `com.softisp.camera`.

If you already have the APK installed and just need to reinstall, uninstall the
old one first from Settings → Apps → SoftISP Camera → Uninstall, then install
the new APK.

---

## 2. Launch the app

Because it is a `NativeActivity` with a launcher intent, open it from the app
drawer ("SoftISP Camera"). There is no `am start` return value to inspect — the
process simply starts `android_main` in `anw/src/app.rs` inside `libsoftisp.so`.

`am start` is optional (you can try it from the device shell, but prefer the
launcher); it returns immediately with no output on success. The real signal is
whether the preview surface appears.

---

## 3. Read the crash log (primary diagnostic)

The app writes every startup line, every ISP frame result, every panic, and
every `log::` record to a file. `logcat` is a secondary source — on Termux it
usually needs the `READ_LOWS` grant (which itself needs adb) and its 256 KiB
ring buffers drain quickly — so **the file is the authoritative log**. Read it
directly on the device.

### Where the log lands

`crashlog::install()` tries these paths in order and keeps the first writable
one:

| # | Path | Notes |
|---|------|-------|
| 1 | `/sdcard/download/softisp-app.log` | Mandated spec path |
| 2 | `/sdcard/Download/softisp-app.log` | Capital-D spelling |
| 3 | `/storage/emulated/0/Download/softisp-app.log` | Full emulated storage path |
| 4 | `<app external data dir>/softisp-app.log` | Always writable fallback |

On **API 30+** (targetSdk 34), scoped storage is mandatory and the public
Download directory is only reachable with the special **"All files access"**
grant (`MANAGE_EXTERNAL_STORAGE`), which the user must enable in Settings
because a NativeActivity has no Activity object to show the permission dialog:

```
Settings → Apps → SoftISP Camera → All files access → Allow
```

Until that grant is given, the app falls back to
`/data/data/com.softisp.camera/files/softisp-app.log` (or the app's external
files dir equivalent) — still fully diagnostic, just not in Download.

### How to read it

All on-device, no adb:

```bash
# The spec path if you granted All files access:
cat /sdcard/download/softisp-app.log

# Or wherever crashlog resolved — the app logs the chosen path on boot:
#   log file: /sdcard/download/softisp-app.log   (or fallback)
```

Pull it off the device however you normally move files from Termux (copy to
`~/shared`, `termux-share`, etc.).

### What to look for

A healthy boot looks like:

```
MM-DD HH:MM:SS.mmm  INFO [boot] log file: /sdcard/download/softisp-app.log
MM-DD HH:MM:SS.mmm  INFO [boot] softisp camera 0.1.0 starting
MM-DD HH:MM:SS.mmm  INFO [cam_hal_rust] softisp camera starting (sdk 34, engine mnn_vulkan)
MM-DD HH:MM:SS.mmm  INFO [cam_hal_rust] log file: /sdcard/download/softisp-app.log
MM-DD HH:MM:SS.mmm  WARN [window] InitWindow callback reached: NativeWindow ptr=...
MM-DD HH:MM:SS.mmm  WARN [window] window geometry 1080x2400
MM-DD HH:MM:SS.mmm  INFO [isp_engine] ISP engine: mnn_vulkan
MM-DD HH:MM:SS.mmm  INFO [pipeline] pipeline built: profile FUSED at 1600x1200px (dual GPU engine)
MM-DD HH:MM:SS.mmm  INFO [capture] raw 1600x1200 min 0 max 1023 mean 412.5 (sensor_max 1023)
MM-DD HH:MM:SS.mmm  INFO [stats] 1 frames processed, last 1600x1200, rgb mean [...], max ..., non-black ...
```

Failure signatures:

| Symptom | Log lines |
|---|---|
| Camera permission missing | `ERROR [camera] Could not open camera...` |
| Storage grant missing | `WARN [storage] /sdcard/download unavailable...` |
| Engine build fails | `FATAL [engine] engine.build failed: ...` / `no ISP engine matching 'mnn_vulkan'` |
| MNN inference fails | `ERROR [isp] process failed: ...` |
| Present/display fails | `ERROR [display] present failed: ...` |
| Panic | `FATAL [panic] ...` |
| Frame too slow | `WARN [loop] frame took N ms (...)` |

---

## 4. Capture logcat (secondary, on-device only)

Run `logcat` directly on the device in Termux — do not use `adb logcat`. The
app's tag is `cam_hal_rust` (set by `init_logger_with_tag`), and panic messages
go under `softisp`.

```bash
logcat -d | grep -E "cam_hal_rust|softisp|MNN|ISP"
logcat -d -s cam_hal_rust   # only our tag
```

Caveats:

* `logcat` needs `READ_LOGS` (`pm grant com.softisp.camera
  android.permission.READ_LOGS`) — which itself usually needs adb, so on a
  plain install you may get nothing.
* The ring is ~256 KiB and drains fast under system load, so a per-frame error
  can push out the startup lines that actually explain the failure. That is why
  the **file log** (section 3) is the primary source.

---

## 5. Environment variables / feature flags

There is **no runtime env var** read by the APK itself — the build profile and
features are compiled in at build time:

| Build knob | Where it is set | Effect in the APK |
|---|---|---|
| `cargo apk2 build` (default) | `cam-rust/Cargo.toml` `[profile.dev]` | `opt-level = 2`, `debug = 1`, no overflow checks — fast enough for preview |
| `--release` | `[profile.release]` | `debug = 1`, `overflow-checks = false`, `codegen-units = 1`, `panic = "abort"` — smaller, lower RSS |
| `mnn` feature | `android-app/Cargo.toml` `[features] default = ["mnn"]` | Bundles `libMNN.so`, `libMNN_Vulkan.so`, `libMNN_Express.so`, `libMNNConvertDeps.so`; engine = `mnn_vulkan` |
| no `mnn` | `--no-default-features` | CPU engine only; no MNN runtime libs in the APK |

If you need to switch engines without rebuilding, that is not supported — the
engine name is a `const` selected by the `mnn` feature:

```rust
// android-app/src/lib.rs
#[cfg(feature = "mnn")]
const ENGINE_NAME: &str = "mnn_vulkan";
#[cfg(not(feature = "mnn"))]
const ENGINE_NAME: &str = "cpu";
```

---

## 6. MNN is a subproject — build its libs before the APK

MNN lives at `~/MNN` and is **not part of this repository**. It is treated as a
subproject: the APK bundles prebuilt `libMNN*.so` files from
`cam-rust/lib/<abi>/`, and those must be current with the MNN source you are
testing.

The correct order is:

```
1. ./scripts/build_mnn_libs.sh arm64-v8a  # called by build-apk.sh
2. ./build-apk.sh                         # packages softisp-anw and verifies libsoftisp.so
```

`scripts/build_mnn_libs.sh` runs `build_vulkan_android.sh` inside `~/MNN`,
copies every `libMNN*.so` plus `libMNNConvertDeps.so` into
`cam-rust/lib/aarch64-v8a/` (and `lib/arm64-v8a/`), and rewrites the
`libvulkan.so.1` NEEDED entry to `libvulkan.so` so the device's system ICD
loader resolves it. The APK then bundles those `.so` files under
`lib/arm64-v8a/` via `runtime_libs = "../cam-rust/lib"` in
`android-app/Cargo.toml`.

If you rebuild MNN and forget to rerun `build_mnn_libs.sh`, the APK ships
stale libraries and you will chase MNN-side bugs that are already fixed
upstream.

---

## 7. Common failure checklist

| Check | Command |
|---|---|
| APK exists and is signed | `ls -l target/debug/apk/softisp-camera.apk` |
| Native libs inside APK | `unzip -l target/debug/apk/softisp-camera.apk \| grep lib/arm64-v8a` |
| MNN libs are current | `ls -l cam-rust/lib/aarch64-v8a/libMNN*.so` — timestamps should be newer than the last APK build |
| "All files access" granted | Settings → Apps → SoftISP Camera → All files access |
| Log file chosen path | First lines of whichever `softisp-app.log` you find |
| Logcat tag | `logcat -d -s cam_hal_rust` |

---

## 8. Ignored P2h device check

The P2h warm-up/cancel-restart check is intentionally ignored in the default
test run. Run it on the device after installing a fresh APK:

```bash
cargo test -p softisp-anw --all-features \
  device_warmup_and_cancel_restart_no_stale_frame -- --ignored --nocapture
```

It reports `warm-up allocation <bytes> bytes in <duration>`, cancels and
restarts the two-worker runner, and asserts that the first frame after restart
is non-blank and carries sequence number 0. The camera and surface stay
user-side: this test does not install, launch, or drive the APK. Use the normal
launch flow, then confirm in `softisp-app.log` that the first frame after a
stop/restart has non-black pixels and no stale sequence gap.
