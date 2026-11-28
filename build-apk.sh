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
# retune every MNN lib + the app's libsoftisp.so to depend on it, so the dynamic
# linker loads our copy instead of the stale system one.
set -euo pipefail

export JAVA_HOME=/data/data/com.termux/files/usr/lib/jvm/java-11-openjdk
export ANDROID_NDK_ROOT="$HOME/android-sdk/ndk/29.0.14206865"
export ANDROID_HOME="$HOME/android-sdk"
cd "$(dirname "$0")"
export RUSTC_WRAPPER="$PWD/scripts/libsoftisp-rustc-wrapper.sh"

RUST_LIB="target/aarch64-linux-android/debug/libsoftisp.so"
APK="target/debug/apk/softisp-camera.apk"

# 1. Build MNN Vulkan libs and copy all .so into cam-rust/lib/aarch64-v8a/.
./scripts/build_mnn_libs.sh arm64-v8a

# 1b. Strip MNN shared libraries to reduce APK size. llvm-strip is preferred
#     (clang/NDK-friendly ELF handling), with strip as fallback.
if command -v llvm-strip >/dev/null 2>&1; then
    STRIP=llvm-strip
elif command -v strip >/dev/null 2>&1; then
    STRIP=strip
else
    STRIP=""
fi
if [ -n "$STRIP" ]; then
    for DIST in cam-rust/lib/arm64-v8a cam-rust/lib/aarch64-v8a; do
        for so in "$DIST"/libMNN*.so; do
            [ -f "$so" ] || continue
            $STRIP --strip-unneeded "$so"
            echo "  stripped: $so ($STRIP --strip-unneeded)"
        done
    done
else
    echo "  WARN: neither llvm-strip nor strip found; MNN libs shipped unstripped"
fi

# 2. Drop the retired app library from the runtime lib dirs. cargo-apk2 takes
#    the app cdylib from target/ (it builds softisp-anw itself); runtime_libs is
#    only for MNN's prebuilt .so files.
for DIST in cam-rust/lib/arm64-v8a cam-rust/lib/aarch64-v8a; do
    rm -f "$DIST/libcam_camera_app.so" "$DIST/libsoftisp.so"
done

# 3. Build the APK with cargo-apk2 (default debug profile). It compiles
#    softisp-anw, packages its cdylib from target/, and reads the manifest and
#    resources from android-app/. RUSTC_WRAPPER retunes the cdylib's libc++
#    dependency before cargo-apk2 copies it into the APK.
cargo apk2 build -p softisp-anw
sync

# 4. The cdylib must depend on our renamed libc++ to avoid the device's stale
#    system libc++_shared.so. RUSTC_WRAPPER should patch it during link; if it
#    didn't (e.g. wrapper not propagated), patch now and rebuild the APK so the
#    packaged artifact is correct.
if command -v patchelf >/dev/null 2>&1 && [ -f "$RUST_LIB" ]; then
    if readelf -d "$RUST_LIB" | grep -q '\[libc++_shared\.so\]'; then
        echo "  WARN: RUSTC_WRAPPER did not retune $RUST_LIB; patching and rebuilding APK"
        patchelf --replace-needed libc++_shared.so libsoftisp_cpp_shared.so "$RUST_LIB"
        cargo apk2 build -p softisp-anw
        sync
    fi
fi

# 5. Fail loudly if the APK still ships the retired library, the wrong libc++, or
#    a byte-different copy of the just-built cdylib.
if command -v unzip >/dev/null 2>&1 && command -v sha256sum >/dev/null 2>&1; then
    APK_LIB="lib/arm64-v8a/libsoftisp.so"
    found=0
    for attempt in 1 2 3; do
        if unzip -Z1 "$APK" 2>/dev/null | grep -Fxq "$APK_LIB"; then
            found=1
            break
        fi
        sleep 1
    done
    if [ "$found" -ne 1 ]; then
        echo "ERROR: $APK is missing $APK_LIB" >&2
        exit 1
    fi
    if unzip -Z1 "$APK" | grep -Fxq "lib/arm64-v8a/libcam_camera_app.so"; then
        echo "ERROR: $APK still contains the retired libcam_camera_app.so" >&2
        exit 1
    fi
    if [ ! -f "$RUST_LIB" ]; then
        echo "ERROR: expected cdylib $RUST_LIB was not produced" >&2
        exit 1
    fi

    target_sha=$(sha256sum "$RUST_LIB" | awk '{print $1}')
    apk_sha=$(unzip -p "$APK" "$APK_LIB" | sha256sum | awk '{print $1}')
    if [ "$target_sha" != "$apk_sha" ]; then
        echo "ERROR: libsoftisp.so hash mismatch (target $target_sha, APK $apk_sha)" >&2
        exit 1
    fi
    tmp_apk_lib=$(mktemp /data/data/com.termux/files/usr/tmp/opencode/libsoftisp.XXXXXX.so)
    unzip -p "$APK" "$APK_LIB" > "$tmp_apk_lib"
    if readelf -d "$tmp_apk_lib" | grep -q '\[libc++_shared\.so\]'; then
        rm -f "$tmp_apk_lib"
        echo "ERROR: $APK_LIB still depends on libc++_shared.so" >&2
        exit 1
    fi
    rm -f "$tmp_apk_lib"
    echo "  verified: $APK_LIB sha256 $apk_sha"

    APK_LOADER="lib/arm64-v8a/libvulkan.so"
    if ! unzip -Z1 "$APK" | grep -Fxq "$APK_LOADER"; then
        echo "ERROR: $APK is missing $APK_LOADER" >&2
        exit 1
    fi
    loader_sha=$(unzip -p "$APK" "$APK_LOADER" | sha256sum | awk '{print $1}')
    local_loader_sha=$(sha256sum "cam-rust/lib/arm64-v8a/libvulkan.so" | awk '{print $1}')
    if [ "$local_loader_sha" != "$loader_sha" ]; then
        echo "ERROR: $APK_LOADER hash mismatch (target $local_loader_sha, APK $loader_sha)" >&2
        exit 1
    fi
    echo "  verified: $APK_LOADER sha256 $loader_sha"

    if [ -f /vendor/lib64/hw/vulkan.adreno.so ]; then
        APK_HAL="lib/arm64-v8a/libsoftisp_vulkan_adreno.so"
        LOCAL_HAL="cam-rust/lib/arm64-v8a/libsoftisp_vulkan_adreno.so"
        if ! unzip -Z1 "$APK" | grep -Fxq "$APK_HAL"; then
            echo "ERROR: $APK is missing $APK_HAL" >&2
            exit 1
        fi
        if [ ! -f "$LOCAL_HAL" ]; then
            echo "ERROR: expected staged Adreno HAL $LOCAL_HAL is missing" >&2
            exit 1
        fi
        hal_sha=$(unzip -p "$APK" "$APK_HAL" | sha256sum | awk '{print $1}')
        local_hal_sha=$(sha256sum "$LOCAL_HAL" | awk '{print $1}')
        if [ "$local_hal_sha" != "$hal_sha" ]; then
            echo "ERROR: $APK_HAL hash mismatch (target $local_hal_sha, APK $hal_sha)" >&2
            exit 1
        fi
        echo "  verified: $APK_HAL sha256 $hal_sha"
    fi
fi
