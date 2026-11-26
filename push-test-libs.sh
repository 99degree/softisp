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
set -euo pipefail

cd "$(dirname "$0")"

CORE="target/aarch64-linux-android/debug/libsoftisp.so"
MNN_DIR="cam-rust/lib/arm64-v8a"
DEST="/sdcard/download"

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

cp -f "$CORE" "$DEST/libsoftisp.so"
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
