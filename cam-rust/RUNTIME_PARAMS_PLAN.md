# Runtime Parameters Implementation Plan

## Goal
Convert blocks using ONNX initializers for parameters to use runtime-input tensors via `extra_inputs()`.

## Background
- **ONNX initializers**: Model constants loaded at model build
- **ONNX inputs**: Runtime-feedable tensors passed to `interpreter->runSessionWithTensors()`
- **`extra_inputs()`**: IspBlock trait method declaring runtime-feedable tensors
- **`SessionPool`**: Manages MNN session input tensors for `extra_inputs`

## Current Architecture
```
IspController → ProcessParams → MnnEngine::set_extra_inputs() → 
    SessionPool.tensor_pool → MNN session.run()
```

## Why Make Parameters Runtime-Allowable?
- Per-frame updates (auto-exposure, auto-white balance, stabilization)
- Scene adaptation (saturation, contrast, denoise strength)
- Preview vs. capture mode tuning
- User preferences


## Block Audit: Parameters Currently Embedded as Initializers

| Block                | Parameter Name       | Type/Tensor Shape      | Current Value Source | Notes |
|----------------------|----------------------|------------------------|----------------------|-------|
| **SaturationBlock**   | scale                 | Float[3]               | self.saturation       | Color boost factor       |
| **SharpnessBlock**    | strength              | Float[1]               | self.strength          | Sharpening strength       |
| **LdciBlock**         | strength              | Float[1]               | self.strength          | Local contrast strength   |
| **FcsBlock**          | gain                  | Float[3,1,1]           | self.gain              | Black level compensation  |
|                      | bias                  | Float[3,1,1]           | self.bias              | Brightness offset         |
| **AutoContrastBlock** | lift                  | Float[1]               | Computed from stats    | Tone mapping params       |
|                      | half                  | Float[1]               | Computed from stats    |                        |
|                      | contrast_w            | Float[1]               | Computed from stats    |                        |
|                      | zero                  | Float[1]               | Computed from stats    |                        |
|                      | one                   | Float[1]               | Computed from stats    |                        |
| **GammaBlock**        | inv_gamma             | Float[1]               | self.inv_gamma          | Gamma correction          |
|                      | lift                  | Float[1]               | self.lift              |                        |
|                      | norm                  | Float[1]               | Computed from inputs   |                        |
|                      | min                   | Float[1]               | self.min                |                        |
|                      | max                   | Float[1]               | self.max                |                        |
| **NormalizeBlock**    | max_val               | Float[1]               | 65535.0                 | Output scaling            |
| **DisplayBlock**      | scale                 | Float[1]               | self.scale              | Brightness control        |
|                      | gamma_exp             | Float[1]               | Catalyst::gamma_conv()  | Optional gamma tone map   |
| **VignettingBlock**   | gain_map              | Float[...]             | self.gain_map           | Radial gain map           |
| **BilateralBlock**    | sigma_spatial         | Float[...]             | self.sigma_spatial      | Denoise tuning            |
|                      | sigma_range           | Float[...]             | self.sigma_range        |                        |


## Conversion Strategy: Initializer → Runtime Input

### 1. Remove from `initializers()`, Add to `extra_inputs()`
```rust
// Before
fn initializers(&self) -> Vec<Vec<u8>> {
    vec![Proto::tensor_proto_float("param/", shape, &data)]
}
fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> { vec![] }

// After
fn initializers(&self) -> Vec<Vec<u8>> { vec![] }
fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
    vec![("param/".into(), elem_type, shape)]
}
```

### 2. Ensure Tensor Name Stability
Fixed prefix — no dynamic names based on runtime values:
✅ Good: `"block/param"`
❌ Bad:  `"block/param_{value}"`

### 3. Extend SessionPool
Add tensor names to `extra_names` list in `SessionPool::new()`.

### 4. Extend set_extra_inputs()
Feed tensor initial/default values even when no override.
```rust
fn set_extra_inputs(pool: &[(String, MnnTensorSafe)], ...) {
    if let Some(t) = find(pool, "SaturationBlock/scale") {
        t.set_data(&INIT_VAL); // override default if controller provided
    }
}
```


## Fusion Considerations
- MNN's VulkanFuse backend should fuse the new input-fed ops
- Initializer-free blocks avoid ONNX converter issues
- Fusion rules need priority tuning (longest-chain-first)

## Testing Checklist
- [ ] Tensor shape validation
- [ ] Default behavior unchanged when no runtime params set
- [ ] Performance benchmark (GPU fused path)
- [ ] Controller-driven updating
- [ ] Temporal consistency (smoothing)
- [ ] Profile coverage: LITE/MED/HEAVY/UNIFIED