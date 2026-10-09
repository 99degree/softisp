# SoftISP Camera (Android)

A native camera preview that runs the whole image pipeline in Rust. There is
no Java, no C, and no C++ in the shipped APK: `android-activity` provides the
`NativeActivity` entry point, the NDK camera2 API is called through raw
`ndk-sys` FFI, frames are processed by the `cam-isp` crate in
[`../cam-rust`](../cam-rust), and the result is blitted straight to an
`ANativeWindow`.

| | |
|---|---|
| Application ID | `com.softisp.camera` |
| Activity | `android.app.NativeActivity` |
| min / target SDK | 24 / 34 |
| ABIs | `arm64-v8a` |
| Native code | `libcam_camera_app.so` (Rust `cdylib`) |
| Crash log | `/sdcard/download/softisp-app.log` |

## Layout

```
android-app/                 packaging only
  Makefile                   drives cargo-apk2
  Sources/res/               launcher icon + strings
  .gitignore
cam-rust/cam-camera-app/     the app itself, all Rust
  AndroidManifest.xml        shipped verbatim (see "Manifest" below)
  src/lib.rs                 event loop, lifecycle, wiring
  src/camera.rs              NDK camera2 capture (RAW16, YUV fallback)
  src/pipeline.rs            RawFrame → cam-isp ProcessParams
  src/preview.rs             ISP output → ANativeWindow
  src/crashlog.rs            rotating log file with panic hook
  src/storage.rs             writability probe + permission guidance
```

## Manifest

The manifest is hand-written and shipped as-is, via
`manifest_override = "AndroidManifest.xml"` in
`cam-rust/cam-camera-app/Cargo.toml`. `cargo-apk2` 1.4.2 serialises
`android:allowNativeHeapPointerTagging` and
`android:requestLegacyExternalStorage` as *child elements* of `<application>`
rather than attributes (the `@` prefix that tells quick-xml to emit an
attribute is missing from the serde rename in `ndk-build2`), and it emits them
even when the fields are unset. Supplying the file whole avoids the bug and
keeps every attribute under review.

`[package.metadata.android]` still declares the application id, SDK range,
resources, runtime libraries, signing key and the activity list; that table
remains the source of truth for what `cargo-apk2` needs in memory. The two must
be kept in step, which is what `make verify-manifest` checks.

## Building

```sh
make            # -> cam-rust/target/release/apk/softisp-camera.apk
make run        # install, then launch the camera
make logs       # tail the crash log
make clean
```

`make build` also runs `make verify-manifest`, which asserts the packaged
manifest with `aapt dump xmltree`: application id, `android.app.lib_name`,
the exported `NativeActivity` with its LAUNCHER filter, min/target SDK and the
permissions. A manifest that parses but has lost `lib_name` produces an app
that installs and then dies in `dlopen` with nothing in logcat, so the check is
part of the build rather than a separate step.

`make toolchain` (run automatically) shims the Android SDK for this host. On
Termux/aarch64 the NDK's `prebuilt/linux-x86_64` binaries and the
`build-tools` `aapt`/`aapt2`/`zipalign` are the wrong architecture, so they are
replaced with symlinks to the Termux equivalents. On a normal x86_64 Linux or
macOS host those shims are no-ops and the stock SDK tools are used as-is.

`cargo apk2` dexes the generated `R.java` with its bundled R8, which rejects
class files newer than Java 11. The Makefile therefore pins `JAVA_HOME` to a
JDK ≤ 17 and fails early if neither JDK 11 nor 17 is present.

### Requirements

* `cargo`, `cargo-apk2`
* Android SDK with `platforms/android-34` and `build-tools/34.0.0`
* Android NDK (29.x is what this was built against)

## Pipeline

```
ACameraManager ──► AImageReader(RAW16)  ──► Vec<u16> Bayer
                                          │
                            cam_isp::CpuEngine (16-stage pipeline)
                                          │
                                    Vec<u8> BGRA
                                          │
                          ANativeWindow_lock / unlockAndPost  (RGBA8888)
```

Capture is RAW16-first because that is the only NDK image format that delivers
the `u16` Bayer samples the ISP expects. When a camera does not advertise
`REQUEST_AVAILABLE_CAPABILITIES_RAW`, the app falls back to `YUV_420_888` and
expands the luma plane into a `GG` mosaic, so the pipeline always receives
Bayer-shaped input of the same geometry. Which path was taken is written to the
crash log at startup.

The engine is selected explicitly as `cpu`: the MNN backend would need its own
native runtime library bundled into the APK.

## Crash log

Every ISP or camera error and every panic is appended to
`/sdcard/download/softisp-app.log`, rotated once past 512 KB to
`softisp-app.log.1`. The same records go to logcat under the `softisp` tag.

That directory is not always writable, and which permission is needed depends
on the API level:

| API | Permission | How it is granted |
|-----|-----------|-------------------|
| ≤ 29 | `WRITE_EXTERNAL_STORAGE` | runtime; a `NativeActivity` cannot show the dialog, so grant it with `adb shell pm grant com.softisp.camera android.permission.WRITE_EXTERNAL_STORAGE` |
| ≥ 30 | `MANAGE_EXTERNAL_STORAGE` | Settings → Apps → SoftISP Camera → **All files access** |

`storage::probe` re-tests the mandated path at startup. If it is unwritable the
log falls back to the app's external directory (no permission needed, so there
is always somewhere to write) and the first WARN line of the log names the
exact grant to make: the active path is recorded as the very first line, e.g.

```
06-21 09:14:02.881 INFO  [boot]      0ms log file: /storage/emulated/0/Android/data/com.softisp.camera/files/softisp-app.log
06-21 09:14:02.883 WARN  [storage]   2ms /sdcard/download/softisp-app.log unavailable: Permission denied (...); the log went to the app's external directory — grant 'All files access' for com.softisp.camera in Settings to use this path
```

So `adb logcat`, or `adb pull /sdcard/Download/softisp-app.log` once the grant
is made, always shows what the app was doing.

## Permissions

`CAMERA` is required for `ACameraManager_openCamera` and is granted at install
time. A `NativeActivity` has no Java layer to show the runtime permission
dialog, so the app cannot grant itself `CAMERA` if it was revoked; uninstall
and reinstall, or grant it with `adb shell pm grant com.softisp.camera
android.permission.CAMERA`.

## Signing

The APK is signed with a throwaway key committed to nothing:
`my-release-key.keystore` (alias `standkey`, password `password`), generated by
the previous rawdrawandroid scaffolding and reused here. It is gitignored.
Generate a real key before publishing:

```sh
keytool -genkey -v -keystore my-release-key.keystore -alias app \
        -keyalg RSA -keysize 2048 -validity 10000
```
