#!/data/data/com.termux/files/usr/bin/bash
# Copy the freshly built shim-test libraries into the public Download directory.
#
# The Android app cannot read Termux's private /data/data/com.termux tree, so a
# copy into /sdcard/download is mandatory; the shim stages from there into
# app-private files before dlopen (Download is mounted noexec).
#
# Because the host triple is aarch64-linux-android, a plain `cargo build` writes
# target/debug/libsoftisp.so, NOT the file below, and links libc++_shared.so.
# The APK-target build is the one that links libsoftisp_cpp_shared.so, so use:
#
#   cargo apk2 -- build -p softisp-anw && ./push-test-libs.sh
#
# for a quick core-only update; no APK rebuild or reinstall is needed.
#
# Device system libs (libvulkan.so, libc++_shared.so) are NOT bundled in the
# APK by design. Instead they are pushed from Termux so the developer manages
# them directly:
#   - libvulkan.so: copied from /data/data/com.termux/files/usr/lib/libvulkan.so
#     (SONAME patched to libvulkan.so so MNN's NEEDED entry resolves)
#   - libsoftisp_cpp_shared.so: built by scripts/build_mnn_libs.sh from Termux
#     libc++ with a unique SONAME, copied into cam-rust/lib/arm64-v8a/
#
set -euo pipefail

cd "$(dirname "$0")"

CORE="target/aarch64-linux-android/debug/libsoftisp.so"
MNN_DIR="cam-rust/lib/arm64-v8a"
DEST="/sdcard/download"
TERMUX_LIB_DIR="/data/data/com.termux/files/usr/lib"

if [ ! -f "$CORE" ]; then
    echo "ERROR: missing $CORE; run ./build-test-apk.sh first" >&2
    exit 1
fi
if [ ! -d "$DEST" ]; then
    echo "ERROR: missing $DEST" >&2
    exit 1
fi
if readelf -d "$CORE" | grep -q '\[libc++_shared\.so\]'; then
    echo "ERROR: $CORE links libc++_shared.so; it is a host build, not the APK core" >&2
    echo "       rebuild with: cargo apk2 -- build -p softisp-anw" >&2
    exit 1
fi
if ! readelf -d "$MNN_DIR/libMNNConvertDeps.so" | grep -Fq 'Library runpath: [$ORIGIN]'; then
    echo "ERROR: MNN libs have build-host RUNPATHs; nested deps will not resolve" >&2
    echo "       rebuild with: ./build-test-apk.sh" >&2
    exit 1
fi

# ── Push the app core ──
cp -f "$CORE" "$DEST/libsoftisp.so"

# ── Push libvulkan.so from Termux (developer-managed, not system-provided) ──
if command -v patchelf >/dev/null 2>&1; then
    # Patchelf SONAME to libvulkan.so so MNN's NEEDED [libvulkan.so] resolves.
    # Termux's libvulkan.so ships with SONAME libvulkan.so.1 which does not match.
    tmp_vulkan="$(mktemp /data/data/com.termux/files/usr/tmp/libvulkan.XXXXXX.so)"
    cp -f "$TERMUX_LIB_DIR/libvulkan.so" "$tmp_vulkan"
    patchelf --set-soname libvulkan.so "$tmp_vulkan"
    cp -f "$tmp_vulkan" "$DEST/libvulkan.so"
    rm -f "$tmp_vulkan"
    echo "  pushed libvulkan.so (from Termux, SONAME=libvulkan.so) -> $DEST/libvulkan.so"
else
    echo "WARN: patchelf not found; copying Termux libvulkan.so without SONAME patch" >&2
    cp -f "$TERMUX_LIB_DIR/libvulkan.so" "$DEST/libvulkan.so"
    echo "  pushed libvulkan.so (from Termux, SONAME may be libvulkan.so.1) -> $DEST/libvulkan.so"
fi

# ── Push all MNN + renamed libc++ libs from cam-rust/lib/arm64-v8a/ ──
count=0
for so in "$MNN_DIR"/lib*.so; do
    [ -f "$so" ] || continue
    cp -f "$so" "$DEST/"
    count=$((count + 1))
done
sync

echo "  pushed core: $CORE -> $DEST/libsoftisp.so"
echo "  pushed $count runtime libs from $MNN_DIR -> $DEST/"
echo "  now relaunch the installed softisp-test.apk (no reinstall needed)"
