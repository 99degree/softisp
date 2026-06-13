#!/bin/bash
# Test and benchmark helper for SoftISP
#
# Quick tests run only fast unit tests.
# Extended tests include CPU pipeline (very slow).
# Benchmarks require MNN/ONNX libraries for full comparison.

set -e
cd "$(cd "$(dirname "$0")" && pwd)"

case "${1:-quick}" in
  quick)
    echo "=== QUICK: unit tests (skipping CPU pipeline tests) ==="
    cargo test --lib -p cam-isp
    ;;
  extended)
    echo "=== EXTENDED: unit tests including CPU pipeline tests ==="
    cargo test --lib -p cam-isp -- --include-ignored --test-threads=1
    ;;
  bench)
    echo "=== BENCH: old benchmark (use 'perf' for new one) ==="
    cargo run --example bench -p cam-isp -- "${@:2}"
    ;;
  perf)
    echo "=== PERF: CPU performance benchmark ==="
    cargo run --release --example perf_bench -p cam-isp -- "${@:2}"
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
    echo "Usage: $0 [quick|extended|bench|perf|list]"
    exit 1
    ;;
esac
