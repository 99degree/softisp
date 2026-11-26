#!/data/data/com.termux/files/usr/bin/bash
# Automated hot-swap debug loop for the shim-only test APK.
#
# Every launch fingerprints all staged libraries with MD5 and prints the values
# to /sdcard/download/softisp-app.log. This script computes md5sum of the local
# build outputs and compares them with the log. On a mismatch it sends a
# dedicated control intent, waits for the app to terminate itself, rebuilds and
# re-pushes the core, and launches again.
set -euo pipefail

export JAVA_HOME=/data/data/com.termux/files/usr/lib/jvm/java-11-openjdk
export ANDROID_NDK_ROOT="$HOME/android-sdk/ndk/29.0.14206865"
export ANDROID_HOME="$HOME/android-sdk"
cd "$(dirname "$0")"
export RUSTC_WRAPPER="$PWD/scripts/libsoftisp-rustc-wrapper.sh"

COMPONENT="com.softisp.camera/com.softisp.camera.ShimActivity"
ACTION="com.softisp.camera.action.CONTROL"
EXTRA="softisp_cmd"
CORE="target/aarch64-linux-android/debug/libsoftisp.so"
MNN_DIR="cam-rust/lib/arm64-v8a"
LOG="/sdcard/download/softisp-app.log"
OUT=".tmp/auto-debug"
ATTEMPTS="${1:-3}"
WAIT_SECS="${2:-12}"

mkdir -p "$OUT"

expected_sums() {
    for file in "$CORE" "$MNN_DIR"/lib*.so; do
        [ -f "$file" ] || continue
        printf '%s %s\n' "$(basename "$file")" "$(md5sum "$file" | cut -d' ' -f1)"
    done
}

launch() {
    logcat -c 2>/dev/null || true
    rm -f "$LOG"
    am start -n "$COMPONENT" >/dev/null
    sleep "$WAIT_SECS"
    logcat -d > "$OUT/logcat.txt" 2>/dev/null || true
    cp -f "$LOG" "$OUT/app.log" 2>/dev/null || : > "$OUT/app.log"
}

send_control() {
    am start -a "$ACTION" -n "$COMPONENT" --es "$EXTRA" "$1" >/dev/null
}

wait_stopped() {
    for _ in $(seq 1 30); do
        cp -f "$LOG" "$OUT/app.log" 2>/dev/null || : > "$OUT/app.log"
        if grep -q "control intent '$1'" "$OUT/app.log" 2>/dev/null \
            || grep -q "forced exit after stop intent" "$OUT/app.log" 2>/dev/null; then
            return 0
        fi
        sleep 1
    done
    echo "  WARN: no confirmation for control intent '$1'" >&2
    return 1
}

verify() {
    expected_sums | sort > "$OUT/expected.txt"
    awk '$3 == "md5" { print $4, $5 }' "$OUT/app.log" | sort > "$OUT/actual.txt"
    if diff -u "$OUT/expected.txt" "$OUT/actual.txt" > "$OUT/diff.txt"; then
        echo "  md5 verification: PASS ($(wc -l < "$OUT/actual.txt") libraries)"
        return 0
    fi
    echo "  md5 verification: FAIL" >&2
    cat "$OUT/diff.txt" >&2
    return 1
}

rebuild_and_push() {
    echo "  rebuilding $CORE"
    cargo apk2 -- build -p softisp-anw >/dev/null
    ./push-test-libs.sh
}

for attempt in $(seq 1 "$ATTEMPTS"); do
    echo "=== attempt $attempt/$ATTEMPTS ==="
    launch
    if verify; then
        echo "OK: loaded libraries match local build outputs"
        exit 0
    fi

    echo "  sending stop intent"
    send_control stop || true
    wait_stopped || true
    if [ "$attempt" -lt "$ATTEMPTS" ]; then
        rebuild_and_push
    fi
done

echo "ERROR: loaded libraries never matched after $ATTEMPTS attempts" >&2
exit 1
