#!/data/data/com.termux/files/usr/bin/bash
# Cross-device SoftISP test runner
# Usage: bash scripts/cross_device_test.sh 2>&1 | tee /sdcard/softisp_test.log

set -euo pipefail
cd "$(dirname "$0")/../cam-rust"

export RUSTFLAGS="-C link-arg=-Wl,--warn-unresolved-symbols -C link-arg=-Wl,--noinhibit-exec"

echo "============================================"
echo " SoftISP Cross-Device Test Suite"
echo " Date: $(date)"
echo " Device: $(getprop ro.product.model 2>/dev/null || echo 'unknown')"
echo " SoC: $(getprop ro.board.platform 2>/dev/null || echo 'unknown')"
echo " GPU: $(getprop ro.hardware.egl 2>/dev/null || echo 'unknown')"
echo " Kernel: $(uname -r)"
echo " Rust: $(rustc -V)"
echo "============================================"
echo ""

# ---- A: Unit Tests ----
echo "--- A: Unit Tests ---"
timeout 120 cargo test -p cam-isp --features mnn --lib 2>&1 | tail -3
echo ""

# ---- B: Integration Tests ----
echo "--- B: Integration Tests ---"
timeout 30 cargo test -p cam-isp --features mnn --test test_new_blocks 2>&1 | tail -3
echo ""

# ---- C1: 4K→FHD Benchmark ----
echo "--- C1: 4K→FHD Benchmark ---"
cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn 2>&1 | \
  grep -E "Engine:|Average:|FPS:|===|tensor_assign needs_resize"
echo ""

# ---- C2: Unified ARGB ----
echo "--- C2: Unified ARGB ---"
cargo run --release --example bench_unified_argb -p cam-isp --features mnn 2>&1 | \
  grep -E "Pipeline build:|Avg:|FPS:|==="
echo ""

# ---- C3: Profile Comparison ----
echo "--- C3: Profile Comparison ---"
cargo run --release --example bench_profiles -p cam-isp --features mnn 2>&1 | \
  grep -E "\[(LITE|MED|HEAVY|PRO|UNIFIED)"
echo ""

# ---- C4: 8K Pipeline ----
echo "--- C4: 8K→FHD ---"
cargo run --release --example bench_8k_to_fhd -p cam-isp --features mnn 2>&1 | \
  grep -E "Average:|FPS:|==="
echo ""

echo "============================================"
echo " Test suite complete."
echo " Results: /sdcard/softisp_test.log"
echo "============================================"
