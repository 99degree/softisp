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
| **Model** | Redmi Note 9 Pro (Xiaomi) | 2109119DG (Xiaomi) |
| **SoC** | Snapdragon 720G (atoll / ATOLL-AB) | Snapdragon 888 (lahaina) |
| **GPU** | Adreno 618 @ 750 MHz (vulkan.adreno.so) | Adreno (vulkan.adreno.so) |
| **RAM** | 5.5 GiB | — |
| **Android version** | 15 (API 35) | 14 (API 34) |
| **Kernel** | 4.14.336-g803b55865869 | 5.4.302-qgki-g7ede20c8692e |
| **Termux version** | — | — |
| **Rust version** | 1.95.0 (59807616e 2026-04-14) | 1.96.1 (31fca3adb 2026-06-26) |
| **MNN lib version** | custom build (libMNN.so 3.1M, libMNN_Vulkan.so 2.1M) | custom build |
| **Test date** | 2026-07-13 | 2026-07-13 |

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
| Total tests | 730 | |
| Passed | 730 | |
| Failed | 0 | |
| Ignored | 8 | |
| Duration | 18.57s | |

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
| `test_new_blocks` | ✅ 51/51 passed | ✅ 51/51 passed |
| `test_mnn_engine` | ✅ (GPU required) | ✅ (GPU required) |
| `pipeline_test` | ✅ (slow, CPU) | ✅ |

### C — Pipeline Benchmarks

All benchmarks run at **release** level with **20 iterations** after **3 warmup frames**.

#### C1 — 4K Bayer → FHD ISP (flagship pipeline)

```bash
RUST_LOG=cam_isp::mnnengine=info \
  cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn 2>&1
```

| Metric | Phone 1 | Phone 2 |
|--------|---------|---------|
| Engine selected | `mnn_vulkan (priority 99)` | `mnn_vulkan (priority 99)` |
| Avg latency | 31.7 ms | **16.9 ms** |
| FPS | 31.6 | **59.2** |
| First frame init | 8.4ms tensor_assign | 51.5ms tensor_assign |
| Steady-state | ~25µs tensor_assign | **~5µs tensor_assign** |

#### C2 — Unified Pipeline ARGB

```bash
cargo run --release --example bench_unified_argb -p cam-isp --features mnn 2>&1
```

| Metric | Phone 1 | Phone 2 |
|--------|---------|---------|
| Pipeline build time | 598 ms | 425 ms |
| Avg latency | 111 ms | **179 ms** |
| FPS | 9 | **6** |

> Note: SD888 is slower on Unified ARGB despite faster 4K→FHD. Likely memory-bandwidth-bound — the 25-block Unified pipeline has high memory traffic that doesn't scale with additional compute units.

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
| First frame `tensor_assign` | 8.4 ms (needs_resize=true) | |
| Steady-state `tensor_assign` | 25 µs (needs_resize=false) | |

---

## Comparison Summary Table

| Category | Metric | Phone 1 (SD720G) | Phone 2 (SD888) | Delta |
|----------|--------|-----------------|-----------------|-------|
| **Unit** | Tests passed | 730/730 | **734/734** | +4 enabled |
| **Integration** | Tests passed | 51/51 | 51/51 | — |
| **4K→FHD** | FPS | 31.6 | **59.2** | **+87%** |
| **4K→FHD** | Latency (ms) | 31.7 | **16.9** | **-47%** |
| **4K→FHD** | First tensor_assign | 8.4 ms | 51.5 ms | slower init |
| **4K→FHD** | Steady tensor_assign (µs) | 25 | **5** | **5× faster** |
| **Unified ARGB** | FPS | 9 | **6** | -33% |
| **Unified ARGB** | Pipeline build | 598 ms | **425 ms** | -29% |
| **Unified ARGB** | Latency | 111 ms | **179 ms** | +61% |

---

## Automated Runner Script

Save as `scripts/cross_device_test.sh`:

```bash
#!/data/data/com.termux/files/usr/bin/bash
# Cross-device SoftISP test runner
# Usage: bash scripts/cross_device_test.sh 2>&1 | tee /sdcard/softisp_test.log

set -euo pipefail
cd "$(dirname "$0")/../../cam-rust"

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
