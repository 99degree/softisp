#!/data/data/com.termux/files/usr/bin/bash
# Rebuild the SoftISP camera APK with the pinned JDK 11 + NDK r29 toolchain.
#
# MNN is a subproject: its .so files are bundled into the APK, so build them
# first and copy them into cam-rust/lib/<abi>/ before the APK build. This
# guarantees the APK ships current MNN libraries rather than stale ones.
#
# libc++ fix: MNN is built by NDK clang against its libc++_shared, which needs
# symbols like __ndk1::__hash_memory that the device's old system
# libc++_shared.so (/vendor/lib64/) lacks. Android's dynamic linker resolves
# libc++_shared.so from the system path first, so simply bundling our own
# libc++_shared.so does not help — the system's stale one wins and libMNN.so
# fails to load with "cannot locate symbol".
#
# Fix: give our Termux libc++ a unique SONAME (libsoftisp_cpp_shared.so) and
# retune every MNN lib + the Rust app lib to depend on it, so the dynamic
# linker loads our copy instead of the stale system one.
set -euo pipefail

export JAVA_HOME=/data/data/com.termux/files/usr/lib/jvm/java-11-openjdk
export ANDROID_NDK_ROOT="$HOME/android-sdk/ndk/29.0.14206865"
export ANDROID_HOME="$HOME/android-sdk"

cd "$(dirname "$0")"

# 1. Build MNN Vulkan libs and copy all .so into cam-rust/lib/aarch64-v8a/
./scripts/build_mnn_libs.sh arm64-v8a

# 2. Build the Rust app library so we can patch its libc++ NEEDED entry
#    before cargo-apk2 bundles it into the APK.
cargo build -p cam-camera-app --target aarch64-linux-android

# 3. Patch every bundled .so's libc++_shared.so dependency to point at our
#    uniquely-named libsoftisp_cpp_shared.so instead of the stale system one.
if command -v patchelf >/dev/null 2>&1; then
    RUST_LIB="target/aarch64-linux-android/debug/libcam_camera_app.so"

    # Patch and copy into both ABI dirs cargo-apk2 may read from:
    #   arm64-v8a — runtime_libs/<android_abi> for APK bundling
    #   aarch64-v8a — cam-isp build.rs abi_dir() for rustc -L/rpath
    for DIST in cam-rust/lib/arm64-v8a cam-rust/lib/aarch64-v8a; do
        mkdir -p "$DIST"
        cp -f "$RUST_LIB" "$DIST/libcam_camera_app.so"
        patchelf --replace-needed libc++_shared.so libsoftisp_cpp_shared.so "$DIST/libcam_camera_app.so"
        echo "  patchelf: $DIST/libcam_camera_app.so libc++_shared.so -> libsoftisp_cpp_shared.so"
    done

    # Patch all MNN libs already in the runtime dir (copied there by
    # build_mnn_libs.sh).
    for DIST in cam-rust/lib/arm64-v8a cam-rust/lib/aarch64-v8a; do
        for target in "$DIST"/libMNN.so "$DIST"/libMNN_Vulkan.so \
                      "$DIST"/libMNN_Express.so "$DIST"/libMNNConvertDeps.so; do
            if [ -f "$target" ]; then
                patchelf --replace-needed libc++_shared.so libsoftisp_cpp_shared.so "$target"
                echo "  patchelf: $target libc++_shared.so -> libsoftisp_cpp_shared.so"
            fi
        done
    done
fi

# 4. Build the APK with cargo-apk2 (default debug profile).
#    cargo-apk2 will use the already-built libcam_camera_app.so from step 2.
cargo apk2 build -p cam-camera-app
