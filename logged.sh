#!/data/data/com.termux/files/usr/bin/bash
# logged.sh — run a command and record it in log.txt.
#
# Every tool call goes through here so log.txt is a faithful transcript of what
# was executed, in order, with the working directory. Useful when a long session
# needs to be reconstructed after the fact.
#
# The log always lands next to this script (the repo root), not in the caller's
# cwd — a command run from ~/MNN should still be recorded in the softisp log.
set -uo pipefail
LOG="$(cd "$(dirname "$0")" && pwd)/log.txt"
{
  printf '[%s] cwd=%s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$(pwd)"
  printf '  $ %s\n' "$*"
} >> "$LOG"
bash -c "$*"
rc=$?
if [ $rc -ne 0 ]; then
  printf '  -> exit %d\n' "$rc" >> "$LOG"
fi
exit $rc
