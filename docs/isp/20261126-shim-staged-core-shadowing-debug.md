# Shim reload shadowed by app-private staged core (2026-11-26)

## Symptom

After the new split build flow:

```bash
./build-test-libs.sh --core-only
./push-test-libs.sh
./shim-debug.sh reload
```

`./shim-debug.sh verify` failed. The freshly built core was
`libsoftisp.so 3d8fcb0b9439710d9fae9ab28b68b6e1`, but the running app logged
`libsoftisp.so dc21de9baf2436ac76da491c916ed0dd`. The device stats still showed
`last 600x2`, so the rotation fix was not actually loaded.

## Evidence

`.tmp/shim-debug/app.log` after the reload:

```text
INFO [shim] core search roots: /data/user/0/com.softisp.camera/files | /sdcard/download | ...
INFO [shim] found core /data/user/0/com.softisp.camera/files/libsoftisp.so
INFO [shim] md5 libsoftisp.so dc21de9baf2436ac76da491c916ed0dd 134881689 bytes
INFO [shim] forwarding android_main to /data/user/0/com.softisp.camera/files/libsoftisp.so
```

There was no `staged core ...` line after the fresh push. `/sdcard/download`
contained the new core; the app-private staged copy was selected first.

## Root cause

`android-shim-app/src/lib.rs::search_roots()` put `staged_dir` (app-private
`files/`) first so that an earlier staged core could beat the APK copy. That
ordering is wrong for hot-swap: once any core has been staged, it permanently
shadows every later `/sdcard/download` push. `copy_core()` then sees
`source == staged` and returns without copying, so reload reuses the old core.

## Fix

Prefer explicit upload roots over the app-private staged copy:

1. `/sdcard/download`
2. `/sdcard/Download`
3. app external data dir
4. staged app-private dir
5. APK native library dir
6. `.`

The staged dir still precedes the APK native dir, so a staged update cannot be
shadowed by an old APK copy, but a fresh push now replaces it. Dependency
syncing uses the same root order, so MNN/C++ libs also refresh before `dlopen`.

## Why staging itself must be removed

Reordering roots is not enough: every `fs::copy()` into app-private storage
creates a second snapshot that can outlive the upload it came from. Even with
the corrected order, a reload before a fresh push reuses the cached copy, and
the cached libs must be kept in sync with the core by a separate code path.

The durable fix is to remove cache staging entirely. The shim opens the
uploaded `.so` files directly and uses Android's `android_dlopen_ext()` with
`ANDROID_DLEXT_USE_LIBRARY_FD`, which loads an ELF from an already-open FD. That
works for `/sdcard/download` even though the filesystem is mounted noexec. All
runtime dependencies are opened from the same directory as the selected core
and loaded in dependency order before the core, so their `DT_SONAME`s satisfy
the core's `DT_NEEDED` entries without any path search. No library is copied and
no app-private staged directory is consulted.

## Linker namespace constraint

Passing the `/sdcard/download/*.so` path itself as the `android_dlopen_ext()`
identity failed on device:

```text
FATAL [shim] fd-backed dlopen /sdcard/download/libc++_shared.so failed:
dlopen failed: library "/sdcard/download/libc++_shared.so" needed or dlopened by
".../lib/arm64/libsoftisp_shim.so" is not accessible for the namespace "clns-7"
```

The app's linker namespace permits its APK native-library directory, not shared
storage. The fix keeps the FD pointed at the uploaded file but passes a loader
identity path of `native_dir.join(name)` — a permitted path that need not
contain the file. The linker does namespace checks against the identity path and
maps the ELF bytes from the supplied FD, so the runtime still comes only from
`/sdcard/download`.

## Verification

- Unit tests: `search_roots` prefers `/sdcard/download`, contains no app-private
  staged dir, and runtime dependencies are ordered before the core.
- Focused checks: `cargo test -p softisp-shim --lib --no-fail-fast` (7 passed),
  `cargo clippy -p softisp-shim --all-features --no-deps -- -D warnings`.
- `./shim-debug.sh reload` now sends `clean` before `stop`; the shim also cleans
  legacy `.so` files at startup. The clean path only deletes app-private `.so`
  files from older shim builds; it never touches `/sdcard/download`.
- MD5 verification is unchanged: `./push-test-libs.sh &&
  ./shim-debug.sh reload && ./shim-debug.sh verify` must report a PASS with the
  new core MD5.
- This changes the shim itself, so `./build-test-apk.sh` must be rerun and the
  user must reinstall `softisp-test.apk`.
- Device evidence after the user cleared app data/cache: the installed APK
  contains only `lib/arm64-v8a/libsoftisp_shim.so`; despite the old installed
  shim hash, the empty app-private staged dir let the shim copy the fresh core
  from `/sdcard/download` and `verify` reported `PASS (7 libraries)`. This is a
  one-time workaround, not a durable fix: once staged, the old shim again
  shadows future pushes. Reinstalling the rebuilt shim APK is still required.

## Follow-up: `libc++_shared.so` was not owned by the push script

After the memfd shim landed, a fresh `--core-only` rebuild produced
`libsoftisp.so cc30a34ef848e80aba243932ee6ac416` and `sensor_orientation=90,
pipeline rotate_mode=1` appeared in the device log, but `./shim-debug.sh verify`
still reported the old core MD5. Two separate faults were involved.

First, `scripts/build_mnn_libs.sh` only stages a **renamed** Termux libc++ as
`libsoftisp_cpp_shared.so`, because MNN's `DT_NEEDED` entries are retuned to that
unique SONAME. It does not promise to create a plain
`cam-rust/lib/arm64-v8a/libc++_shared.so`. `push-test-libs.sh` happened to copy
a leftover copy with the `lib*.so` wildcard, but the shim's
`RUNTIME_DEPENDENCIES` explicitly preloads `libc++_shared.so` before MNN and the
core. A clean rebuild would therefore leave that dependency stale or absent
in `/sdcard/download`. The push script now treats plain `libc++_shared.so` as a
developer-managed Termux library like `libvulkan.so`: verify its SONAME, copy it
explicitly, and skip the wildcard copy so no stale MNN-dir file can replace it.
`shim-debug.sh verify` hashes the same Termux source instead of the incidental
MNN-dir copy.

Second, `anw/src/crashlog.rs::install()` truncated the app log on every core
start. The shim logs its MD5 lines immediately before `forwarding android_main`,
so the core erased the only file-based proof of which binaries had just been
loaded. The fresh launch was visible in logcat with the correct `cc30…` MD5 but
not in `softisp-app.log`. `install()` now rotates an oversized log instead of
truncating it; the shim's pre-core lines survive in the rotated generation and
`verify` can compare the current launch.

Device proof after the fix: `md5 verification: PASS (8 libraries)` with core
`5d8eea3937eb79d058b119c0f062f205`, `sensor_orientation=90, pipeline
rotate_mode=1`, `rotate: mode 1 (800x600 -> 600x800)`, steady `last 600x800`
stats, plain `libc++_shared.so 381f48bf739852249dce82817a269b3d` pushed from
Termux, and no `FATAL [shim]`.
