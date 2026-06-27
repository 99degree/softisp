# SoftISP — Project Tracking

## Current State: Vulkan ISP Pipeline (MNN backend)

GPU-accelerated camera ISP pipeline running on MNN's Vulkan backend.
3-dispatch fused pipeline: unpack_demosaic_fcs → ee_ldci → display.

### Performance (Vulkan, Snapdragon)
| Resolution | Latency | FPS |
|---|---|---|
| HD 1280×720 | 8.77 ms | **114** |
| FHD 1920×1080 | 20.09 ms | **50** |
| 4K 3840×2160 | 73.25 ms | **14** |

### Pipeline Profiles
| Profile | Stages | Ops |
|---|---|---|
| LITE | unpack→display | 4 |
| MED | unpack→ee→display | 5 |
| HEAVY | unpack→fcs→ee→ldci→display | 7 |
| PRO | All stages + extras | 10+ |

## Repo Layout

### `softisp/` (root)
- `vulkan_isp/` — GLSL compute shaders, Python ONNX generators, C++ test harnesses
- `cam-rust/cam-isp/` — Rust FFI bindings, ONNX proto encoder, E2E tests

### `MNN/` (fork)
- `tools/converter/source/optimizer/postconvert/IspChainFusion.cpp` — ISP fusion pass
- `tools/converter/source/optimizer/postconvert/isp_spirv_embedded.h` — Embedded SPIR-V
- `source/backend/vulkan/` — Vulkan backend (with concurrent session mutex fix)

## Converter Fusion Rules
| Rule | Pattern | Result |
|---|---|---|
| R1 | unpack_blc | `isp.unpack_blc` |
| R2 | Conv1x1 4ch→3ch | `isp.demosaic_ccm` |
| R3a | Conv3x3 group=3 | `isp.ee` |
| R3b | Mul+Add | `isp.fcs` |
| R4 | AvgPool+Sub+Mul+Add | `isp.ldci` |
| R5 | Pow+Clip | `isp.display` |
| R6 | Pow+Clip (alt pattern) | `isp.display` |
| R7 | fcs+display | `isp.fcs_display` |
| R8 | fcs+display (adjacent) | `isp.fcs_display` |
| R9 | ee+ldci → ee_ldci | `isp.ee_ldci` |
| R10 | unpack_blc+demosaic_ccm | `isp.unpack_demosaic` |
| R12 | unpack_demosaic+fcs | `isp.unpack_demosaic` (FCS fused in) |

## Key Files
| File | Purpose |
|---|---|
| `IspChainFusion.cpp` | Pass 1 (standard→Extra) + Pass 2 (chain→fused) |
| `isp_spirv_embedded.h` | SPIR-V for each Extra op type |
| `shader_unpack_demosaic.comp` | Bayer→RGB with BLC+WB+CCM+FCS |
| `shader_ee_ldci_fused.comp` | Edge enhancement + local contrast |
| `shader6_display_simple.comp` | sRGB gamma |
| `gen_isp_onnx_standard.py` | Python ONNX generator (standard ops) |
| `mnn_wrapper.cpp` | C FFI for MNN session/tensor ops |
| `test_e2e_isp_pipeline.rs` | E2E: ONNX→MNN→Vulkan→verify |

## Completed
- [x] R12: FCS fused into unpack_demosaic (13→12 ops)
- [x] Concurrent Vulkan sessions (mutex on vkQueueSubmit)
- [x] HEAVY profile test (ONNX generation + conversion)
- [x] Named ISP params (blc/wb/ccm/fcs/ee/ldci/display)
- [x] VulkanFuse autoTuning bugfix
- [x] Compiler warnings cleanup
- [x] README.md updated
- [x] R11 fused_6in1 removed (2-4× slower than 3-dispatch)

## Remaining Work
- [ ] Rust pipeline ONNX → IspChainFusion alignment (Rust uses Mod+Div+Cast+Div+Concat+Conv for unpack, Python uses Cast+Conv)
- [x] Display gamma LUT — attempted, 2× slower (shared memory barrier + 6 reads worse than 3 pow() calls)
- [ ] FP16 output support
- [ ] Bayer pattern configurability (non-uniform Gr≠Gb)
- [ ] 8K support

## Testing
```bash
# E2E (serial, requires Vulkan)
cargo test --test test_e2e_isp_pipeline --features mnn -- --ignored --test-threads=1

# Profiles
cargo test --test test_profile_onnx --features mnn

# Concurrent sessions
./test_concurrent_vk model.mnn 4
```
