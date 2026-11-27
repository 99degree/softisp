#!/data/data/com.termux/files/usr/bin/bash
# Build the shim-debug runtime libraries without building an APK.
#
# Default: rebuild MNN, strip it, retune its RUNPATH/SONAME dependencies, then
# build the hot-swappable core with the NDK target. Use --core-only for the
# fast ISP iteration loop when MNN itself has not changed.
set -euo pipefail

export JAVA_HOME=/data/data/com.termux/files/usr/lib/jvm/java-11-openjdk
export ANDROID_NDK_ROOT="$HOME/android-sdk/ndk/29.0.14206865"
export ANDROID_HOME="$HOME/android-sdk"
cd "$(dirname "$0")"
export RUSTC_WRAPPER="$PWD/scripts/libsoftisp-rustc-wrapper.sh"

CORE_LIB="target/aarch64-linux-android/debug/libsoftisp.so"
MNN_DIR="cam-rust/lib/arm64-v8a"
MNN_DIRS=(cam-rust/lib/arm64-v8a cam-rust/lib/aarch64-v8a)

usage() {
    cat <<'EOF'
Usage: ./build-test-libs.sh [--core-only]

Build the libraries used by softisp-test.apk without building the APK.

  (no flags)    Build MNN, strip/retune it, then build libsoftisp.so
  --core-only   Rebuild only libsoftisp.so; keep the current MNN libs
EOF
}

MNN_BUILD=1
case "${1:-}" in
    "") ;;
    --core-only|--no-mnn) MNN_BUILD=0 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown option '$1'" >&2; usage >&2; exit 2 ;;
esac
if [ "$#" -gt 1 ]; then
    echo "ERROR: too many arguments" >&2
    usage >&2
    exit 2
fi

strip_mnn_libs() {
    local strip dist so
    strip="$(command -v llvm-strip || true)"
    if [ -z "$strip" ]; then
        echo "  WARN: llvm-strip not found; MNN libs stay unstripped" >&2
        return
    fi
    for dist in "${MNN_DIRS[@]}"; do
        for so in "$dist"/libMNN*.so; do
            [ -f "$so" ] || continue
            "$strip" --strip-unneeded "$so"
        done
    done
}

retune_mnn_libs() {
    if ! command -v patchelf >/dev/null 2>&1; then
        echo "ERROR: patchelf is required to retune MNN RUNPATHs for staging" >&2
        exit 1
    fi
    local so
    for so in "$MNN_DIR"/libMNN*.so "$MNN_DIR"/libsoftisp_cpp_shared.so; do
        [ -f "$so" ] || continue
        patchelf --set-rpath '$ORIGIN' "$so"
    done
    if [ -f "$MNN_DIR/libMNNConvertDeps.so" ] \
        && ! readelf -d "$MNN_DIR/libMNNConvertDeps.so" | grep -Fq 'Library runpath: [$ORIGIN]'; then
        echo "ERROR: $MNN_DIR/libMNNConvertDeps.so is missing the \$ORIGIN RUNPATH" >&2
        exit 1
    fi
}

if [ "$MNN_BUILD" -eq 1 ]; then
    ./scripts/build_mnn_libs.sh arm64-v8a
    strip_mnn_libs
    retune_mnn_libs
else
    echo "  keeping current MNN libs (--core-only)"
fi

EXTRA_FEATURES=()
if [ "${DIRECT_VULKAN:-0}" = "1" ]; then
    EXTRA_FEATURES=(--features direct_vulkan)
fi

cargo apk2 -- build -p softisp-anw "${EXTRA_FEATURES[@]}"
sync

if [ ! -f "$CORE_LIB" ]; then
    echo "ERROR: expected core cdylib $CORE_LIB was not produced" >&2
    exit 1
fi

if readelf -d "$CORE_LIB" | grep -q '\[libc++_shared\.so\]'; then
    if command -v patchelf >/dev/null 2>&1; then
        echo "  WARN: RUSTC_WRAPPER did not retune $CORE_LIB; patching now"
        patchelf --replace-needed libc++_shared.so libsoftisp_cpp_shared.so "$CORE_LIB"
    else
        echo "ERROR: $CORE_LIB links libc++_shared.so and patchelf is unavailable" >&2
        exit 1
    fi
fi

if readelf -d "$CORE_LIB" | grep -q '\[libc++_shared\.so\]'; then
    echo "ERROR: $CORE_LIB still links libc++_shared.so" >&2
    exit 1
fi

echo "  core: $CORE_LIB"
echo "  runtime libs: $MNN_DIR/lib*.so"
echo "  upload with: ./push-test-libs.sh"
