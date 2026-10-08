# Technical Summary: Softisp Camera App Build and Fixes

## Goal
Build a debug APK of the Softisp camera app, install it, and verify that the app launches without crashing, using logcat to check for any softisp-related errors.

## Constraints
- Use logcat to inspect messages.
- Build the debug APK with `./gradlew assembleDebug`.
- Install the APK via `adb` or `pm`.
- Ensure the latest build is used (rebuild if source changes).
- No additional constraints were specified.

## Progress
- Retrieved the last 50 lines of logcat and searched for softisp‑related messages (initially no crashes found).
- Located and refactored `SoftispPreviewView.java` to add the required constructors (with Context, AttributeSet, and defStyleAttr) that delegate to a common `init()` method for proper XML inflation.
- Updated `CameraActivity.java` to use `SoftispPreviewView` instead of `androidx.camera.view.PreviewView` and properly initialized the frame processor.
- Fixed `SoftispJni.java` to load the native library via `System.loadLibrary("cam_isp")` instead of a hardcoded path.
- Attempted to build the debug APK; the first build failed due to a malformed edit, but succeeded after correcting the file.
- Copied the resulting APK to `/data/local/tmp`, `/sdcard`, and the current directory, and installed it using `pm install -r`.
- Verified the package appears in `pm list packages | grep softisp`.
- Examined logcat for errors related to softisp, camera, or runtime exceptions; the app launches and shows a window but crashes shortly after due to a missing JNI symbol: `java.lang.UnsatisfiedLinkError: No implementation found for void com.softisp.camera.SoftispJni.setPreviewSurface(android.graphics.SurfaceTexture)`.
- All Java/Kotlin code changes have been committed to the repository.
- Worked on the native library linking in the cam-isp Rust crate: created `jni.c`, modified `build.rs` to conditionally compile and link the JNI library when targeting Android and the `jni` feature is enabled, and adjusted `Cargo.toml` to include the `jni` feature by default for Android builds.
- Built the native library with the `jni` feature and copied `libcam_isp.so` to the app's `src/main/jniLibs/arm64-v8a` directory, then rebuilt the APK.

## Decisions
- Refactored `SoftispPreviewView.java` to centralize initialization in an `init()` method called from all three constructors to eliminate duplication and ensure proper view inflation.
- Chose to build the debug APK and install it via `pm` after copying to `/sdcard` as a fallback when direct `adb install` failed.
- Decided to trigger a clean rebuild if the existing APK is older than source changes.
- Modified the JNI approach to use a native window (ANativeWindow) via `SurfaceTexture` for display, moving the display responsibility to the native code.
- Implemented a minimal JNI C file (`jni.c`) in the cam-isp crate that logs the call and declares the Rust init function.
- Updated the cam-isp `build.rs` to conditionally compile and link the JNI library for Android targets when the `jni` feature is enabled, using `cc::Build` to compile `src/jni.c` into a static library and linking it into the final shared library.
- Set the default features to include `jni` for Android builds to ensure the JNI code is linked.

## Next steps
- Verify that the `jni` feature is enabled when building the cam-isp crate for Android.
- Check the build output to ensure `jni.c` is being compiled and linked into `libcam_isp.so`.
- Use `nm` or `readelf` to confirm that the JNI symbols (`JNI_OnLoad` and `Java_com_softisp_camera_SoftispJni_setPreviewSurface`) are present in the built library.
- If missing, adjust the `build.rs` to correctly link the JNI library (e.g., using whole-archive linking if necessary).
- Ensure the library is packaged correctly in the APK (check that `libcam_isp.so` is in `/lib/arm64-v8a/` inside the APK).
- After fixing the native library, reinstall the APK and verify that the `setPreviewSurface` JNI call resolves and the app does not crash.
- Then, implement the actual native window setup in the JNI function (create `ANativeWindow` from `SurfaceTexture`, set buffer geometry, etc.).
- Finally, ensure the frame processing pipeline works and the preview is displayed.

## Files
- `/data/data/com.termux/files/home/softisp/android-app/src/main/java/com/softisp/camera/SoftispPreviewView.java` – refactored constructors to use a common `init()` method.
- `/data/data/com.termux/files/home/softisp/android-app/src/main/java/com/softisp/camera/CameraActivity.java` – replaced `PreviewView` with `SoftispPreviewView` and initialized frame processor.
- `/data/data/com.termux/files/home/softisp/android-app/src/main/java/com/softisp/camera/SoftispJni.java` – changed static initializer to `System.loadLibrary("cam_isp")` and updated native method signature for `setPreviewSurface`.
- `/data/data/com.termux/files/home/softisp/cam-rust/cam-isp/src/jni.c` – created JNI implementation file with `JNI_OnLoad` and `setPreviewSurface` functions.
- `/data/data/com.termux/files/home/softisp/cam-rust/cam-isp/build.rs` – modified to conditionally compile and link the JNI library for Android with the `jni` feature.
- `/data/data/com.termux/files/home/softisp/cam-rust/cam-isp/Cargo.toml` – adjusted features to include `jni` by default for Android targets.
- `/data/data/com.termux/files/home/softisp/android-app/src/main/jniLibs/arm64-v8a/libcam_isp.so` – copied built native library into the APK's `jniLibs` directory.