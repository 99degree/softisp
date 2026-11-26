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
TERMUX_LIB_DIR="/data/data/com.termux/files/usr/lib"
TERMUX_LIBCPP="$TERMUX_LIB_DIR/libc++_shared.so"
TERMUX_VULKAN="$TERMUX_LIB_DIR/libvulkan.so"
LOG="/sdcard/download/softisp-app.log"
OUT=".tmp/shim-debug"
WAIT_SECS="${WAIT_SECS:-12}"

usage() {
    cat <<'EOF'
Usage: ./shim-debug.sh <command> [args]

Commands:
  build          Build the shim-only softisp-test.apk
  libs [--core-only]
                 Build MNN + core runtime libs (strip + patchelf, no APK)
  push           Push core + MNN libs + Termux Vulkan/libc++ runtime libs
  core           Rebuild only the core, then push (fast ISP iteration)
  reload         Ask the app to clean legacy staged libs, stop, then relaunch
  deploy         core + reload
  launch         Launch ShimActivity with am start
  ping           Send the control intent's ping command
  cancel         Ask the running pipeline to cancel after in-flight work
  clean          Ask the app to delete legacy .so files from its private cache
  stop           Ask the app to finish itself (no shell force-stop needed)
  logs           Dump logcat + app log into .tmp/shim-debug and print a tail
  verify         Compare logged MD5s with local md5sum of build outputs
  cycle          core -> reload -> wait -> verify
  help           Show this message

Examples:
  ./shim-debug.sh core
  ./shim-debug.sh reload
  ./shim-debug.sh deploy
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

do_libs() {
    ./build-test-libs.sh "$@"
}

do_push() {
    ./push-test-libs.sh
}

do_core() {
    do_libs --core-only
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
        ping|cancel|clean|stop|kill|quit|exit) ;;
        *) echo "ERROR: unknown control command '$cmd'" >&2; usage >&2; exit 2 ;;
    esac
    am start -a "$ACTION" -n "$COMPONENT" --es "$EXTRA" "$cmd" >/dev/null
    echo "  sent control intent: $cmd"
}

do_reload() {
    do_control clean || true
    do_control stop || true
    sleep 3
    do_launch
}

do_deploy() {
    do_core
    do_reload
}

do_logs() {
    mkdir -p "$OUT"
    logcat -d > "$OUT/logcat.txt" 2>/dev/null || true
    if [ -f "$LOG" ]; then
        if [ -f "$LOG.1" ]; then
            cat "$LOG.1" "$LOG" > "$OUT/app.log"
        else
            cp -f "$LOG" "$OUT/app.log"
        fi
    else
        : > "$OUT/app.log"
        echo "  WARN: $LOG missing; app may be using private fallback" >&2
    fi
    echo "  logcat: $OUT/logcat.txt"
    echo "  app log: $OUT/app.log"
    tail -n 40 "$OUT/app.log" || true
}

expected_sums() {
    if [ -f "$TERMUX_VULKAN" ]; then
        if command -v patchelf >/dev/null 2>&1; then
            tmp_vulkan="$(mktemp /data/data/com.termux/files/usr/tmp/vulkan-expected.XXXXXX.so)"
            cp -f "$TERMUX_VULKAN" "$tmp_vulkan"
            patchelf --set-soname libvulkan.so "$tmp_vulkan"
            printf '%s %s\n' libvulkan.so "$(md5sum "$tmp_vulkan" | cut -d' ' -f1)"
            rm -f "$tmp_vulkan"
        else
            printf '%s %s\n' libvulkan.so "$(md5sum "$TERMUX_VULKAN" | cut -d' ' -f1)"
        fi
    fi
    for file in "$CORE" "$TERMUX_LIBCPP" "$MNN_DIR"/lib*.so; do
        [ -f "$file" ] || continue
        # push-test-libs.sh selects plain libc++ from Termux, so an incidental
        # cam-rust/lib copy must not create a duplicate expected hash.
        if [ "$file" = "$MNN_DIR/libc++_shared.so" ]; then
            continue
        fi
        printf '%s %s\n' "$(basename "$file")" "$(md5sum "$file" | cut -d' ' -f1)"
    done
}

do_verify() {
    mkdir -p "$OUT"
    if [ ! -f "$LOG" ]; then
        echo "ERROR: $LOG missing; run '$0 launch' then '$0 logs'" >&2
        exit 1
    fi
    if [ -f "$LOG.1" ]; then
        cat "$LOG.1" "$LOG" > "$OUT/app.log"
    else
        cp -f "$LOG" "$OUT/app.log"
    fi
    expected_sums | sort > "$OUT/expected.txt"
    awk '
        BEGIN { start = 1 }
        /\[shim\] launcher started/ { start = NR }
        { line[NR] = $0 }
        END {
            for (i = start; i <= NR; i++) {
                n = split(line[i], field, " ")
                if (field[3] == "md5") print field[4], field[5]
            }
        }
    ' "$OUT/app.log" | sort > "$OUT/actual.txt"
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
    do_core
    do_reload
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
    libs)   shift; do_libs "$@" ;;
    push)   do_push ;;
    core)   do_core ;;
    reload) do_reload ;;
    deploy) do_deploy ;;
    launch) do_launch ;;
    ping)   do_control ping ;;
    cancel) do_control cancel ;;
    clean)  do_control clean ;;
    stop)   do_control stop ;;
    logs)   do_logs ;;
    verify) do_verify ;;
    cycle)  do_cycle ;;
    help|-h|--help) usage ;;
    *) echo "ERROR: unknown command '$1'" >&2; usage >&2; exit 2 ;;
esac
