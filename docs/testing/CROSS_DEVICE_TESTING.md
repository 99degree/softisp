# Cross-Device Test Coverage & Benchmark Suite

Run the same tests on **two phones** and compare results side-by-side.
This document defines the standardized test matrix, output format, and comparison methodology.

---

## Quick Start

```bash
# 1. Clone & build
git clone https://github.com/99degree/softisp
cd softisp/cam-rust

# 2. Run full cross-device test suite (logs everything)
bash scripts/cross_device_test.sh 2>&1 | tee /sdcard/softisp_$(date +%Y%m%d_%H%M%S).log

# 3. Compare results between two phones
#    Collect logs from both → paste into comparison table below
```

---

## Device Info (Fill Per Phone)

| Field | Phone 1 | Phone 2 |
|-------|---------|---------|
| **Model** | | |
| **SoC** | | |
| **GPU** | | |
| **RAM** | | |
| **Android version** | | |
| **Kernel** | | |
| **Termux version** | | |
| **Rust version** | | |
| **MNN lib version** | | |
| **Test date** | | |

Collect with:
```bash
echo "Model: $(getprop ro.product.model)"
echo "SoC: $(getprop ro.board.platform)"
echo "GPU: $(getprop ro.hardware.egl)"
echo "RAM: $(free -h | grep Mem | awk '{print $2}')"
echo "Android: $(getprop ro.build.version.release)"
echo "Kernel: $(uname -r)"
echo "Rust: $(rustc -V)"
echo "MNN: $(strings lib/aarch64-v8a/libMNN.so | grep -i 'mnn_version' | head -1)"
```

---

## Test Matrix

Each test produces a **row** in the comparison table. Run all tests on both phones.

### A — Library Unit Tests

```bash
RUSTFLAGS="-C link-arg=-Wl,--warn-unresolved-symbols -C link-arg=-Wl,--noinhibit-exec" \
  cargo test -p cam-isp --features mnn --lib 2>&1
```

| Metric | Phone 1 | Phone 2 |
|--------|---------|---------|
| Total tests | `grep "test result:"` | |
| Passed | | |
| Failed | | |
| Ignored | | |
| Duration | | |

### B — Integration Tests

```bash
# Integration test suite
RUSTFLAGS="-C link-arg=-Wl,--warn-unresolved-symbols -C link-arg=-Wl,--noinhibit-exec" \
  cargo test -p cam-isp --features mnn --test test_new_blocks 2>&1

# Engine-specific tests
RUSTFLAGS="-C link-arg=-Wl,--warn-unresolved-symbols -C link-arg=-Wl,--noinhibit-exec" \
  cargo test -p cam-isp --features mnn --test test_mnn_engine 2>&1

# Pipeline tests
RUSTFLAGS="-C link-arg=-Wl,--warn-unresolved-symbols -C link-arg=-Wl,--noinhibit-exec" \
  cargo test -p cam-isp --features mnn --test pipeline_test 2>&1
```

| Test Suite | Phone 1 | Phone 2 |
|------------|---------|---------|
| `test_new_blocks` | ✅/❌ ___/___ | ✅/❌ ___/___ |
| `test_mnn_engine` | ✅/❌ ___/___ | ✅/❌ ___/___ |
| `pipeline_test` | ✅/❌ ___/___ | ✅/❌ ___/___ |

### C — Pipeline Benchmarks

All benchmarks run at **release** level with **20 iterations** after **3 warmup frames**.

#### C1 — 4K Bayer → FHD ISP (flagship pipeline)

```bash
RUST_LOG=cam_isp::mnnengine=info \
  cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn 2>&1
```

| Metric | Phone 1 | Phone 2 |
|--------|---------|---------|
| Engine selected | `grep "Engine:"` | |
| Avg latency | `grep "Average:"` | |
| FPS | `grep "FPS:"` | |
| First frame (init) | `grep "infer_done.*total=" | head -1` | |
| Steady-state infer | `grep "infer_done.*prep=" | tail -5 | awk ...` | |

#### C2 — Unified Pipeline ARGB

```bash
cargo run --release --example bench_unified_argb -p cam-isp --features mnn 2>&1
```

| Metric | Phone 1 | Phone 2 |
|--------|---------|---------|
| Pipeline build time | `grep "Pipeline build:"` | |
| Avg latency | `grep "Avg:"` | |
| FPS | `grep "FPS:"` | |

#### C3 — Profile Comparison

```bash
RUSTFLAGS="-C link-arg=-Wl,--warn-unresolved-symbols -C link-arg=-Wl,--noinhibit-exec" \
  cargo run --release --example bench_profiles -p cam-isp --features mnn 2>&1
```

| Profile | Phone 1 FPS | Phone 2 FPS | Speedup Factor |
|---------|-------------|-------------|----------------|
| LITE HD | | | |
| LITE 4K | | | |
| MED HD | | | |
| MED 4K | | | |
| HEAVY HD | | | |
| HEAVY 4K | | | |

Extract with:
```bash
grep -E "\[(LITE|MED|HEAVY|PRO|UNIFIED)" benchmark_output.log
```

#### C4 — 8K Pipeline

```bash
RUSTFLAGS="-C link-arg=-Wl,--warn-unresolved-symbols -C link-arg=-Wl,--noinhibit-exec" \
  cargo run --release --example bench_8k_to_fhd -p cam-isp --features mnn 2>&1
```

| Metric | Phone 1 | Phone 2 |
|--------|---------|---------|
| Avg latency | | |
| FPS | | |

#### C5 — Heavy Format Bench

```bash
RUSTFLAGS="-C link-arg=-Wl,--warn-unresolved-symbols -C link-arg=-Wl,--noinhibit-exec" \
  cargo run --release --example bench_heavy_formats -p cam-isp --features mnn 2>&1
```

| Format | Phone 1 FPS | Phone 2 FPS |
|--------|-------------|-------------|
| FloatRgb | | |
| PackedRgb | | |
| FloatBgra | | |
| Argb | | |

### D — Burst Capacity Test

Measures how many consecutive frames can be processed within a **33ms budget (30fps)**.

```bash
# Custom burst test (override bench for 5-frame burst)
RUST_LOG=cam_isp::mnnengine=info \
  cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn 2>&1 | \
  grep "infer_done.*prep=" | awk '{print $NF}' | while read t; do
    total=$(echo "$t * 1000" | bc | cut -d. -f1)
    echo "Frame: ${total}µs"
  done
```

| Burst Depth | Phone 1 Budget Used | Phone 2 Budget Used |
|-------------|---------------------|---------------------|
| 1 frame | | |
| 3 frames | | |
| 5 frames | | |
| Max at 30fps | `floor(33ms / frame_ms)` | |

### E — sess.resize() Optimization Verification

Confirm the cached-shape optimization is active:

```bash
RUST_LOG=cam_isp::mnnengine=info \
  cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn 2>&1 | \
  grep "tensor_assign"
```

Expected output pattern:
```
needs_resize=true  elapsed=~25ms   ← first frame (full resize + init)
needs_resize=false elapsed=~10µs   ← all subsequent frames (shape cached)
```

| Metric | Phone 1 | Phone 2 |
|--------|---------|---------|
| First frame `tensor_assign` | `grep "needs_resize=true"` | |
| Steady-state `tensor_assign` | `grep "needs_resize=false" | tail -1` | |

---

## Comparison Summary Table

| Category | Metric | Phone 1 | Phone 2 | Delta |
|----------|--------|---------|---------|-------|
| **Unit** | Tests passed | | | |
| **Integration** | Tests passed | | | |
| **4K→FHD** | FPS | | | |
| **4K→FHD** | Latency (ms) | | | |
| **4K→FHD** | Steady infer (ms) | | | |
| **4K→FHD** | `tensor_assign` after opt (µs) | | | |
| **Unified ARGB** | FPS | | | |
| **LITE HD** | FPS | | | |
| **LITE 4K** | FPS | | | |
| **HEAVY HD** | FPS | | | |
| **HEAVY 4K** | FPS | | | |
| **8K→FHD** | FPS | | | |
| **Burst max** | Frames at 30fps | | | |

---

## Automated Runner Script

Save as `scripts/cross_device_test.sh`:

```bash
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
cargo test -p cam-isp --features mnn --lib 2>&1 | tail -3
echo ""

# ---- B: Integration Tests ----
echo "--- B: Integration Tests ---"
cargo test -p cam-isp --features mnn --test test_new_blocks 2>&1 | tail -3
echo ""

# ---- C1: 4K→FHD ----
echo "--- C1: 4K→FHD Benchmark ---"
cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn 2>&1 | grep -E "Engine:|Average:|FPS:|===|tensor_assign needs_resize"
echo ""

# ---- C2: Unified ARGB ----
echo "--- C2: Unified ARGB ---"
cargo run --release --example bench_unified_argb -p cam-isp --features mnn 2>&1 | grep -E "Pipeline build:|Avg:|FPS:|==="
echo ""

# ---- C3: Profile Comparison ----
echo "--- C3: Profile Comparison ---"
cargo run --release --example bench_profiles -p cam-isp --features mnn 2>&1 | grep -E "\[(LITE|MED|HEAVY|PRO|UNIFIED)"
echo ""

# ---- C4: 8K Pipeline ----
echo "--- C4: 8K→FHD ---"
cargo run --release --example bench_8k_to_fhd -p cam-isp --features mnn 2>&1 | grep -E "Average:|FPS:|==="
echo ""

echo "============================================"
echo " Test suite complete."
echo "============================================"
```

---

## Quick Comparison Command (paste results here)

Collect outputs from both phones and paste into the tables above.
For a quick diff between two log files:

```bash
# Side-by-side comparison
echo "=== Phone 1 FPS ===" && grep "FPS:" phone1.log
echo "=== Phone 2 FPS ===" && grep "FPS:" phone2.log
echo "=== Phone 1 Errors ===" && grep -c "ERROR\|FAILED\|panic" phone1.log
echo "=== Phone 2 Errors ===" && grep -c "ERROR\|FAILED\|panic" phone2.log
```
