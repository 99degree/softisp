# ISP Display Output Format Design

## Problem

The current SoftISP code uses a single `isp.display` op with an `output_format` attribute to select the output pixel format. However, MNN's VulkanFuse shader selection works by **op type name**, not by reading attributes. The `IspSpvLookup` map in MNN maps exact strings like `"isp.display_uint32_rgba"`, `"isp.display_uint8_argb"`, etc. to their embedded SPIR-V shaders.

The `output_format` attribute passed to `isp.display` is **ignored** by the VulkanFuse runtime — it only reads the op's type string to look up the shader. This means all frames were being processed by the base `isp.display` shader (planar float RGB output) regardless of the requested format.

## Root Cause

In `cam-rust/cam-isp/src/blocks/display.rs`, the `emit_mnn_isp_min_nodes()` function:

```rust
attrs.push(Proto::attribute_int(
    "output_format",
    self.output_format as i64,
));
vec![Proto::node_with_domain(
    "isp.display",  // <-- Always uses base shader
    "isp",
    &[&inp, &gamma],
    &[final_output],
    &attrs,
)]
```

But MNN's converter (`IspOnnxOps.cpp`) registers **separate transforms** for each format:
- `IspDisplay` → `"isp.display"` (planar float RGB)
- `IspDisplayUint8Argb` → `"isp.display_uint8_argb"`
- `IspDisplayUint8Rgba` → `"isp.display_uint8_rgba"`
- `IspDisplayUint8Bgra` → `"isp.display_uint8_bgra"`
- `IspDisplayUint32Argb` → `"isp.display_uint32_argb"`
- `IspDisplayUint32Rgba` → `"isp.display_uint32_rgba"`
- `IspDisplayUint32Bgra` → `"isp.display_uint32_bgra"`
- `IspDisplayRgb` → `"isp.display_rgb"`
- `IspDisplayRgbaF16` → `"isp.display_rgba_f16"`

## Solution

Map `OutputFormat` enum variants to the correct `isp.display_*` op type in `emit_mnn_isp_min_nodes()`, so the generated ONNX node carries the exact type string that VulkanFuse expects.

## OutputFormat → isp.display_* Mapping (IMPLEMENTED)

| OutputFormat | isp.* Op Type | Output Layout |
|--------------|---------------|---------------|
| `FloatRgb` | `isp.display` | `[1,3,H,W]` float32 planar RGB [0,1] |
| `FloatBgra` | `isp.display_rgb` | `[1,3,H,W]` float32 planar (fallback, uses Conv in primitive path) |
| `PackedRgb` | `isp.display_uint32_rgba` | `[1,1,H,W]` uint32 packed (2 pixels/word) |
| `Bgra` | `isp.display_uint8_bgra` | `[1,4,H,W]` uint8 interleaved BGRA |
| `Rgba` | `isp.display_uint8_rgba` | `[1,4,H,W]` uint8 interleaved RGBA |
| `Argb` | `isp.display_uint8_argb` | `[1,4,H,W]` uint8 interleaved ARGB |
| `Abgr` | `isp.display_uint8_bgra` | `[1,4,H,W]` uint8 interleaved ABGR (same shader as BGRA, diff channel order) |
| `Rgb` | `isp.display_rgb` | `[1,3,H,W]` float32 planar RGB [0,255] |
| `Bgr` | `isp.display_rgb` | `[1,3,H,W]` float32 planar BGR [0,255] |
| `Float16Rgb` | `isp.display_rgba_f16` | `[1,4,H,W]` float16 interleaved RGBA [0,1] |
| `Float16Bgra` | `isp.display_rgba_f16` | `[1,4,H,W]` float16 interleaved BGRA [0,1] |

## Implementation

### 1. Added `isp_display_op_type()` method to `OutputFormat` enum (engine.rs)

```rust
pub fn isp_display_op_type(self) -> &'static str {
    match self {
        Self::FloatRgb => "isp.display",
        Self::FloatBgra => "isp.display_rgb",
        Self::PackedRgb => "isp.display_uint32_rgba",
        Self::Bgra => "isp.display_uint8_bgra",
        Self::Rgba => "isp.display_uint8_rgba",
        Self::Argb => "isp.display_uint8_argb",
        Self::Abgr => "isp.display_uint8_bgra",
        Self::Rgb => "isp.display_rgb",
        Self::Bgr => "isp.display_rgb",
        Self::Float16Rgb => "isp.display_rgba_f16",
        Self::Float16Bgra => "isp.display_rgba_f16",
    }
}
```

### 2. Updated `emit_mnn_isp_min_nodes()` in DisplayBlock (display.rs)

- Removed the `output_format` attribute (was ignored by VulkanFuse)
- Use `self.output_format.isp_display_op_type()` to select the correct op type
- The `pack_two_pixels` attribute is still passed (used by `isp.display_uint32_rgba` for PackedRgb)

## Verification

- ✅ `cargo check -p cam-isp --features rectifier` — compiles
- ✅ `cargo fmt --all -- --check` — formatting passes
- ✅ `cargo clippy -p cam-isp --lib --features rectifier -- -D warnings` — no warnings
- ✅ `cargo test -p cam-isp --lib --features rectifier` — 813 tests pass

## Next Steps for Full Validation

When MNN Vulkan backend is available:
1. Build and run with `FUSED` profile (which uses `opset_mode: 2` and `fuse_stages.display: true`)
2. Check that the converted `.mnn` model contains the correct `isp.display_*` op type
3. Verify output bytes match expected format:
   - `PackedRgb` → 2 bytes/pixel (INT32 packed, 2 pixels per word)
   - `Bgra`/`Rgba`/`Argb`/`Abgr` → 4 bytes/pixel interleaved
   - `FloatRgb` → 12 bytes/pixel (float32 planar)
   - `Float16Rgb`/`Float16Bgra` → 8 bytes/pixel (float16 interleaved)