#!/data/data/com.termux/files/usr/bin/bash
# Build the test-only launcher APK.
#
# This APK deliberately contains NO MNN libraries and NO libsoftisp.so. It ships
# only libsoftisp_shim.so, whose job is to stage a separately pushed core cdylib
# and its MNN dependencies from /sdcard/download into app-private storage.
# The production APK remains build-apk.sh.
set -euo pipefail

export JAVA_HOME=/data/data/com.termux/files/usr/lib/jvm/java-11-openjdk
export ANDROID_NDK_ROOT="$HOME/android-sdk/ndk/29.0.14206865"
export ANDROID_HOME="$HOME/android-sdk"
cd "$(dirname "$0")"
export RUSTC_WRAPPER="$PWD/scripts/libsoftisp-rustc-wrapper.sh"

CORE_LIB="target/aarch64-linux-android/debug/libsoftisp.so"
SHIM_LIB="target/aarch64-linux-android/debug/libsoftisp_shim.so"
APK="target/debug/apk/softisp-test.apk"
MNN_DIR="cam-rust/lib/arm64-v8a"

# MNN is still built locally: the core links against these libs and the shim
# stages them from the device copy pushed next to libsoftisp.so.
./scripts/build_mnn_libs.sh arm64-v8a

STRIP="$(command -v llvm-strip || true)"
if [ -n "$STRIP" ]; then
    for so in "$MNN_DIR"/libMNN*.so; do
        [ -f "$so" ] || continue
        "$STRIP" --strip-unneeded "$so"
    done
else
    echo "  WARN: llvm-strip not found; device MNN libs stay unstripped" >&2
fi

rm -f target/debug/apk/softisp-test-unaligned.apk target/debug/apk/classes.dex

# Build the hot-swappable core with the NDK environment, then the shim APK.
cargo apk2 -- build -p softisp-anw
cargo apk2 build -p softisp-shim
sync

# The core links libc++ via NDK; the shim does not. Patch the core if the
# wrapper was not propagated. No APK rebuild is needed for a core-only patch.
if command -v patchelf >/dev/null 2>&1 && [ -f "$CORE_LIB" ]; then
    if readelf -d "$CORE_LIB" | grep -q '\[libc++_shared\.so\]'; then
        echo "  WARN: RUSTC_WRAPPER did not retune $CORE_LIB; patching now"
        patchelf --replace-needed libc++_shared.so libsoftisp_cpp_shared.so "$CORE_LIB"
    fi
fi

if command -v unzip >/dev/null 2>&1 && command -v sha256sum >/dev/null 2>&1; then
    APK_LIB="lib/arm64-v8a/libsoftisp_shim.so"
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

    lib_count=$(unzip -Z1 "$APK" | grep -c '^lib/' || true)
    if [ "${lib_count:-0}" -ne 1 ]; then
        echo "ERROR: $APK must contain exactly one native library, found ${lib_count:-0}:" >&2
        unzip -Z1 "$APK" | grep '^lib/' >&2 || true
        exit 1
    fi
    if unzip -Z1 "$APK" | grep -Eq '^lib/arm64-v8a/(libMNN|libsoftisp\.so|libsoftisp_core|libsoftisp_hot|libc\+\+_shared)'; then
        echo "ERROR: $APK must not bundle MNN or the application core" >&2
        exit 1
    fi
    if [ ! -f "$SHIM_LIB" ]; then
        echo "ERROR: expected shim cdylib $SHIM_LIB was not produced" >&2
        exit 1
    fi
    if [ ! -f "$CORE_LIB" ]; then
        echo "ERROR: expected core cdylib $CORE_LIB was not produced" >&2
        exit 1
    fi

    target_sha=$(sha256sum "$SHIM_LIB" | awk '{print $1}')
    apk_sha=$(unzip -p "$APK" "$APK_LIB" | sha256sum | awk '{print $1}')
    if [ "$target_sha" != "$apk_sha" ]; then
        echo "ERROR: libsoftisp_shim.so hash mismatch (target $target_sha, APK $apk_sha)" >&2
        exit 1
    fi
    tmp_apk_lib=$(mktemp /data/data/com.termux/files/usr/tmp/opencode/libsoftisp_shim.XXXXXX.so)
    unzip -p "$APK" "$APK_LIB" > "$tmp_apk_lib"
    if readelf -d "$tmp_apk_lib" | grep -q '\[libc++_shared\.so\]'; then
        rm -f "$tmp_apk_lib"
        echo "ERROR: $APK_LIB still depends on libc++_shared.so" >&2
        exit 1
    fi
    rm -f "$tmp_apk_lib"
    echo "  verified: $APK_LIB sha256 $apk_sha"
fi

echo "  test APK: $APK"
echo "  refresh device libs with: ./push-test-libs.sh"
echo "    $CORE_LIB -> /sdcard/download/libsoftisp.so"
echo "    $MNN_DIR/lib*.so -> /sdcard/download/"
