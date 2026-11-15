#!/data/data/com.termux/files/usr/bin/bash
# Install the built APK through a PackageManager install session.
# `pm install <path>` cannot read Termux's private data dir (SELinux), but
# `install-write` accepts the APK on stdin, which sidesteps the file access.
set -euo pipefail

APK="${1:-$(dirname "$0")/cam-rust/target/debug/apk/softisp-camera.apk}"
[ -f "$APK" ] || { echo "no such apk: $APK" >&2; exit 1; }
SIZE=$(stat -c %s "$APK")

OUT=$(pm install-create -r -t --user 0 -i com.termux -S "$SIZE")
echo "$OUT"
SID=$(printf '%s' "$OUT" | sed -n 's/.*\[\([0-9][0-9]*\)\].*/\1/p')
[ -n "$SID" ] || { echo "could not parse session id" >&2; exit 1; }
echo "session=$SID"

cat "$APK" | pm install-write -S "$SIZE" "$SID" base -
pm install-commit "$SID"
