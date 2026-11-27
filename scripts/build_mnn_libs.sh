#!/data/data/com.termux/files/usr/bin/bash
# Build MNN Vulkan libraries and copy all .so files to softisp runtime dirs.
#
# Usage: ./scripts/build_mnn_libs.sh [aarch64-v8a|arm64-v8a|armeabi-v7a|both]
#
# Default: build arm64-v8a (Termux native clang targets this).
# The "both" target builds arm64-v8a and armeabi-v7a.
#
# After a successful build every libMNN*.so produced under build_isp/ is
# installed into cam-rust/lib/<abi>/ and patchelf'd (libvulkan.so.1 ->
# libvulkan.so). The same arm64 build is installed under BOTH directory
# names the Rust side uses:
#
#   arm64-v8a    — cargo-apk2's runtime_libs/<android_abi>: bundled into the APK
#   aarch64-v8a  — cam-isp build.rs abi_dir(): rustc -L search path + rpath
#
# build_vulkan_android.sh only accepts Android ABI names, so aarch64-v8a is
# normalized to arm64-v8a before the build starts.

set -euo pipefail

SOFTISP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -d "$SOFTISP_ROOT/third_party/MNN" ]; then
    MNN_ROOT="$SOFTISP_ROOT/third_party/MNN"
else
    MNN_ROOT="${MNN_DIR:-$HOME/MNN}"
fi
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

# Map ABI → Android ABI name understood by build_vulkan_android.sh
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
    local build_abi
    build_abi="$(abi_map "$1")"
    local build_dir="build_isp"

    echo "============================================================"
    echo "Building MNN for ABI=$build_abi (requested: $1)"
    echo "  MNN_ROOT   = $MNN_ROOT"
    echo "  BUILD_DIR  = $build_dir"
    echo "  RUNTIME    = $RUNTIME_DIR/{${build_abi},aarch64-v8a}"
    echo "============================================================"

    cd "$MNN_ROOT"

    # Build MNN with Vulkan, converter enabled (ON).
    ./build_vulkan_android.sh "$build_dir" "$build_abi" "android-21" "ON"

    # With MNN_SEP_BUILD=ON the outputs land in different subtrees of the
    # build dir (root, source/, express/, tools/converter/), so search the
    # whole tree instead of relying on a single glob.
    local built=()
    while IFS= read -r so; do
        built+=("$so")
    done < <(find "$build_dir" -name 'libMNN*.so' | sort)

    if [ "${#built[@]}" -eq 0 ]; then
        echo "ERROR: no libMNN*.so produced under $MNN_ROOT/$build_dir" >&2
        exit 1
    fi

    # One arm64 build serves both on-disk ABI names (see header).
    local dests=("$RUNTIME_DIR/$build_abi")
    if [ "$build_abi" = "arm64-v8a" ]; then
        dests+=("$RUNTIME_DIR/aarch64-v8a")
    fi

    local copied=0
    local dest so target
    for dest in "${dests[@]}"; do
        mkdir -p "$dest"

        for so in "${built[@]}"; do
            cp -f "$so" "$dest/"
            echo "  copied: $so -> $dest/$(basename "$so")"
            copied=$((copied + 1))
        done

        # libMNN_Vulkan.so / libMNNConvertDeps.so link against libvulkan.so.1, but
        # on-device Android ships libvulkan.so (soname libvulkan.so). Do not ship
        # our own copy; instead retune each NEEDED entry to libvulkan.so so the
        # device's system ICD loader resolves it (cargo-apk2 bundles runtime_libs
        # but we deliberately exclude libvulkan.so from the APK).
        if command -v patchelf >/dev/null 2>&1; then
            for target in "$dest/libMNN_Vulkan.so" "$dest/libMNNConvertDeps.so"; do
                if [ -f "$target" ]; then
                    patchelf --replace-needed libvulkan.so.1 libvulkan.so "$target"
                    echo "  patchelf: $target libvulkan.so.1 -> libvulkan.so"
                fi
            done

            # MNN is built by NDK clang against its libc++_shared, which needs
            # symbols like __ndk1::__hash_memory that the device's old system
            # libc++_shared.so (/vendor/lib64/) lacks. Android's dynamic linker
            # resolves libc++_shared.so from the system path first, so simply
            # bundling our own libc++_shared.so in the APK does not help — the
            # system's stale one wins and libMNN.so fails to load.
            #
            # Fix: give our bundled libc++ a unique SONAME so the linker loads
            # ours instead of the system's, and retune every MNN lib's NEEDED
            # entry to point at it.
            termux_libcpp="/data/data/com.termux/files/usr/lib/libc++_shared.so"
            if [ -f "$termux_libcpp" ]; then
                local cpp_dest="$dest/libsoftisp_cpp_shared.so"
                cp -f "$termux_libcpp" "$cpp_dest"
                patchelf --set-soname libsoftisp_cpp_shared.so "$cpp_dest"
                echo "  copied+soname: $termux_libcpp -> $cpp_dest (SONAME=libsoftisp_cpp_shared.so)"

                for target in "$dest/libMNN.so" "$dest/libMNN_Vulkan.so" \
                              "$dest/libMNN_Express.so" "$dest/libMNNConvertDeps.so"; do
                    if [ -f "$target" ]; then
                        patchelf --replace-needed libc++_shared.so libsoftisp_cpp_shared.so "$target"
                        echo "  patchelf: $target libc++_shared.so -> libsoftisp_cpp_shared.so"
                    fi
                done
            else
                echo "  WARN: Termux libc++_shared.so not found at $termux_libcpp; APK may fail to load libMNN.so"
            fi
        else
            echo "  WARN: patchelf not found; libs keep libc++_shared.so NEEDED"
        fi
    done

    echo "============================================================"
    echo "Done for ABI=$build_abi — $copied file(s) installed."
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
