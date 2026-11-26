#!/data/data/com.termux/files/usr/bin/bash
# Painless wrapper around the shim-only APK hot-swap workflow.
#
# This script never installs, uninstalls, clears, force-stops, or disables the
# app. Installation is user-side (AGENTS.md 1.7). Only `am start` is used to
# launch the activity and to send the shim's own control intent.
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
OUT=".tmp/shim-debug"
WAIT_SECS="${WAIT_SECS:-12}"

usage() {
    cat <<'EOF'
Usage: ./shim-debug.sh <command> [args]

Commands:
  build          Build MNN, the hot-swappable core, and softisp-test.apk
  push           Push the current core + MNN libs to /sdcard/download
  core           Rebuild only the core, then push (fast ISP/MNN iteration)
  launch         Launch ShimActivity with am start
  ping           Send the control intent's ping command
  cancel         Ask the running pipeline to cancel after in-flight work
  stop           Ask the app to finish itself (no shell force-stop needed)
  logs           Dump logcat + app log into .tmp/shim-debug and print a tail
  verify         Compare logged MD5s with local md5sum of build outputs
  cycle          stop -> core -> launch -> wait -> verify
  help           Show this message

Examples:
  ./shim-debug.sh core
  ./shim-debug.sh launch
  ./shim-debug.sh verify
  WAIT_SECS=20 ./shim-debug.sh cycle

Installation of softisp-test.apk is always done by the user.
EOF
}

expect_device() {
    if [ ! -d /sdcard/download ]; then
        echo "ERROR: /sdcard/download is not available" >&2
        exit 1
    fi
}

do_build() {
    ./build-test-apk.sh
}

do_push() {
    ./push-test-libs.sh
}

do_core() {
    cargo apk2 -- build -p softisp-anw
    ./push-test-libs.sh
}

do_launch() {
    expect_device
    am start -n "$COMPONENT" >/dev/null
    echo "  launched $COMPONENT"
}

do_control() {
    local cmd="$1"
    case "$cmd" in
        ping|cancel|stop|kill|quit|exit) ;;
        *) echo "ERROR: unknown control command '$cmd'" >&2; usage >&2; exit 2 ;;
    esac
    am start -a "$ACTION" -n "$COMPONENT" --es "$EXTRA" "$cmd" >/dev/null
    echo "  sent control intent: $cmd"
}

do_logs() {
    mkdir -p "$OUT"
    logcat -d > "$OUT/logcat.txt" 2>/dev/null || true
    if [ -f "$LOG" ]; then
        cp -f "$LOG" "$OUT/app.log"
    else
        : > "$OUT/app.log"
        echo "  WARN: $LOG missing; app may be using private fallback" >&2
    fi
    echo "  logcat: $OUT/logcat.txt"
    echo "  app log: $OUT/app.log"
    tail -n 40 "$OUT/app.log" || true
}

expected_sums() {
    for file in "$CORE" "$MNN_DIR"/lib*.so; do
        [ -f "$file" ] || continue
        printf '%s %s\n' "$(basename "$file")" "$(md5sum "$file" | cut -d' ' -f1)"
    done
}

do_verify() {
    mkdir -p "$OUT"
    if [ ! -f "$LOG" ]; then
        echo "ERROR: $LOG missing; run '$0 launch' then '$0 logs'" >&2
        exit 1
    fi
    cp -f "$LOG" "$OUT/app.log"
    expected_sums | sort > "$OUT/expected.txt"
    awk '$3 == "md5" { print $4, $5 }' "$OUT/app.log" | sort > "$OUT/actual.txt"
    if diff -u "$OUT/expected.txt" "$OUT/actual.txt" > "$OUT/diff.txt"; then
        echo "  md5 verification: PASS ($(wc -l < "$OUT/actual.txt") libraries)"
        return 0
    fi
    echo "  md5 verification: FAIL" >&2
    cat "$OUT/diff.txt" >&2
    echo "  details: $OUT/diff.txt" >&2
    return 1
}

do_cycle() {
    do_control stop || true
    sleep 3
    do_core
    do_launch
    sleep "$WAIT_SECS"
    do_logs >/dev/null
    do_verify
}

if [ $# -lt 1 ]; then
    usage
    exit 2
fi

case "$1" in
    build)  do_build ;;
    push)   do_push ;;
    core)   do_core ;;
    launch) do_launch ;;
    ping)   do_control ping ;;
    cancel) do_control cancel ;;
    stop)   do_control stop ;;
    logs)   do_logs ;;
    verify) do_verify ;;
    cycle)  do_cycle ;;
    help|-h|--help) usage ;;
    *) echo "ERROR: unknown command '$1'" >&2; usage >&2; exit 2 ;;
esac
