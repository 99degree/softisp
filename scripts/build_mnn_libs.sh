#!/data/data/com.termux/files/usr/bin/bash
# Build MNN Vulkan libraries and copy all .so files to softisp runtime dirs.
#
# Usage: ./scripts/build_mnn_libs.sh [aarch64-v8a|arm64-v8a|both]
#
# Default: build arm64-v8a (Termux native clang targets this).
# The "both" target builds both ABIs.
#
# After a successful build, every *.so from build_isp/ is copied to
# cam-rust/lib/<abi>/ so the softisp runtime picks up the fresh binaries.

set -euo pipefail

MNN_ROOT="/data/data/com.termux/files/home/MNN"
SOFTISP_ROOT="/data/data/com.termux/files/home/softisp"
RUNTIME_DIR="${SOFTISP_ROOT}/cam-rust/lib"

ABI_TARGET="${1:-arm64-v8a}"

# ── validate ─────────────────────────────────────────────────────────────────
if [ ! -d "$MNN_ROOT" ]; then
    echo "ERROR: MNN root not found at $MNN_ROOT" >&2
    exit 1
fi
if [ ! -d "$SOFTISP_ROOT" ]; then
    echo "ERROR: softisp root not found at $SOFTISP_ROOT" >&2
    exit 1
fi

# Map ABI → build dir and runtime subdir
abi_map() {
    case "$1" in
        aarch64-v8a|arm64-v8a) echo "arm64-v8a" ;;
        armeabi-v7a)           echo "armeabi-v7a" ;;
        x86_64)                echo "x86_64" ;;
        x86)                   echo "x86" ;;
        *)                     echo "$1" ;;
    esac
}

build_and_copy() {
    local abi="$1"
    local build_dir="build_isp"

    echo "============================================================"
    echo "Building MNN for ABI=$abi"
    echo "  MNN_ROOT   = $MNN_ROOT"
    echo "  BUILD_DIR  = $build_dir"
    echo "  RUNTIME    = $RUNTIME_DIR/$abi"
    echo "============================================================"

    cd "$MNN_ROOT"

    # Build MNN with Vulkan, converter enabled (ON).
    ./build_vulkan_android.sh "$build_dir" "$abi" "android-21" "ON"

    # Create runtime dir if missing
    mkdir -p "$RUNTIME_DIR/$abi"

    # Copy every shared object produced by the build.
    local copied=0
    for so in build_isp/libMNN*.so build_isp/tools/converter/libMNNConvertDeps.so; do
        if [ -f "$so" ]; then
            cp -f "$so" "$RUNTIME_DIR/$abi/"
            echo "  copied: $so -> $RUNTIME_DIR/$abi/$(basename "$so")"
            copied=$((copied + 1))
        fi
    done

    # libMNN_Vulkan.so links against libvulkan.so.1, but on-device Android
    # ships libvulkan.so (soname libvulkan.so). Do not ship our own copy;
    # instead retune libMNN_Vulkan.so's NEEDED entry to libvulkan.so so the
    # device's system ICD loader resolves it (cargo-apk2 bundles runtime_libs
    # but we deliberately exclude libvulkan.so from the APK).
    if command -v patchelf >/dev/null 2>&1; then
        local target="$RUNTIME_DIR/$abi/libMNN_Vulkan.so"
        if [ -f "$target" ]; then
            patchelf --replace-needed libvulkan.so.1 libvulkan.so "$target"
            echo "  patchelf: $target libvulkan.so.1 -> libvulkan.so"
            copied=$((copied + 1))
        fi
    else
        echo "  WARN: patchelf not found; libMNN_Vulkan.so keeps libvulkan.so.1 NEEDED"
    fi

    echo "============================================================"
    echo "Done for ABI=$abi — $copied file(s) copied."
    echo "============================================================"
}

case "$ABI_TARGET" in
    both)
        build_and_copy "arm64-v8a"
        build_and_copy "armeabi-v7a"
        ;;
    *)
        build_and_copy "$ABI_TARGET"
        ;;
esac
