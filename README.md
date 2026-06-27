# SoftISP — Vulkan ISP Pipeline

GPU-accelerated camera ISP pipeline running on MNN's Vulkan backend.

## Architecture

```
Bayer RAW → [unpack_demosaic] → [fcs] → [ee_ldci] → [display] → RGB output
              BLC+WB+CCM+       linear    Laplacian    sRGB
              demosaic           transform  + LDCI      gamma
```

**3-dispatch pipeline** (optimal):
1. `unpack_demosaic` — Bayer unpack + BLC + WB + CCM demosaic
2. `ee_ldci` — Edge enhancement + local contrast
3. `display` — sRGB gamma

Each stage is a custom SPIR-V compute shader loaded via `OpType_Extra` (VulkanFuse).

## Performance (Vulkan, Snapdragon)

| Resolution | Latency | FPS |
|---|---|---|
| HD 1280×720 | 8.77 ms | **114** |
| FHD 1920×1080 | 20.09 ms | **50** |
| 4K 3840×2160 | 73.25 ms | **14** |

## Pipeline Profiles

| Profile | Stages | Ops |
|---|---|---|
| LITE | unpack→display | 4 |
| MED | unpack→ee→display | 5 |
| HEAVY | unpack→fcs→ee→ldci→display | 7 |
| PRO | All stages + extras | 10+ |

## Key Components

### Converter (`IspChainFusion.cpp`)
ONNX → MNN fusion pass that detects ISP stage patterns and fuses them into custom `OpType_Extra` ops with embedded SPIR-V.

- **Pass 1**: Standard ops → ISP Extra ops (R1-R6)
- **Pass 2**: Extra chain → fused Extras (R8-R10)
- Each stage stores named parameters (`blc`, `wb`, `ccm`, `fcs`, `ee`, `ldci`, `display`)

### Shaders (`vulkan_isp/`)
- `shader_unpack_demosaic.comp` — Bayer → RGB with CCM
- `shader_ee_ldci_fused.comp` — EE + LDCI combined
- `shader6_display_simple.comp` — sRGB gamma
- `shader_fused_6in1.comp` — All-in-one (disabled: 2-4× slower due to 5×5 FCS redundancy)

### Rust FFI (`cam-rust/cam-isp/`)
- `mnn_wrapper.cpp` — C wrapper for MNN session/tensor operations
- `test_e2e_isp_pipeline.rs` — End-to-end: ONNX gen → MNN convert → Vulkan run → verify

## Build

```bash
# MNN converter
cd ~/MNN/build_vk
make MNNConvert -j$(nproc)
cp tools/converter/OFF/libMNNConvertDeps.so ~/softisp/cam-rust/lib/aarch64-v8a/

# Rust FFI
cd ~/softisp/cam-rust
cargo build --package cam-isp --features mnn
```

## Test

```bash
# E2E (requires --test-threads=1 due to Vulkan device queue serialization)
cargo test --test test_e2e_isp_pipeline --features mnn -- --ignored --test-threads=1

# Profiles
cargo test --test test_profile_onnx --features mnn
```

## Notes

- Vulkan backend requires `Precision_High` for correct fp32 output
- Concurrent Vulkan sessions need `--test-threads=1` (device queue race)
- R11 (fused_6in1) disabled — each thread recomputes 5×5 FCS grid, overwhelming buffer savings
