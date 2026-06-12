#!/bin/bash
# Test benchmark — runs tests with per-test timing.
# Usage:
#   bash bench-tests.sh              # quick: unit tests only (fast)
#   bash bench-tests.sh extended     # full: unit + integration + ignored

set -e
cd "$(cd "$(dirname "$0")" && pwd)"

case "${1:-quick}" in
  quick)
    echo "=== QUICK: unit tests (skipping ignored) ==="
    cargo test --lib -p cam-isp
    ;;
  extended)
    echo "=== EXTENDED: all tests ==="
    echo ""
    echo "─── Unit tests ───"
    cargo test --lib -p cam-isp -- --include-ignored --test-threads=1
    echo ""
    echo "─── Integration tests ───"
    cargo test --test pipeline_test -p cam-isp -- --test-threads=1
    ;;
  bench)
    echo "=== BENCH: pipeline benchmark ==="
    cargo run --example bench -p cam-isp -- "${@:2}"
    ;;
  list)
    echo "=== TEST LIST ==="
    echo ""
    echo "Unit tests:"
    cargo test --lib -p cam-isp -- --list 2>/dev/null | grep "^    "
    echo ""
    echo "Integration tests:"
    cargo test --test pipeline_test -p cam-isp -- --list 2>/dev/null | grep "^    "
    ;;
  *)
    echo "Usage: $0 [quick|extended|bench|list]"
    exit 1
    ;;
esac
