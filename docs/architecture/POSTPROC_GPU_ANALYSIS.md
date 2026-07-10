# Post-Processing Block GPU Analysis

## Summary

The following blocks are disabled in UNIFIED profile because they cause segfaults during MNN Vulkan optimization:

| Block | ONNX Ops Used | Issue | Fix Required |
|-------|---------------|-------|--------------|
| bilateral | GaussianBlur, Sobel, Mul | **Custom ops not in standard ONNX** | Rewrite with Conv |
| vignetting | Range, Mul | Tensor shape mismatch | Fix gain_map shape |
| colorspace | ReduceMax, ReduceMin, Sub, Div | Division by zero | Add epsilon |
| wavelet_denoise | AveragePool, Sub, Abs, Max, Div | Complex multi-pass | Simplify |
| auto_contrast | Add, Sub, Mul, Clip | Dynamic shapes | Fix initializers |

---

## Detailed Analysis

### 1. Bilateral Block (BLOCKED)

**Current Implementation:**
```rust
Proto::node("GaussianBlur", &[input], &[output], &[sigma, kernel_size]);
Proto::node("Sobel", &[input], &[output], &[]);
Proto::node("Mul", &[blurred, edges], &[output], &[]);
```

**Problem:**
- `GaussianBlur` is NOT a standard ONNX op (opset 21)
- `Sobel` is NOT a standard ONNX op
- MNN doesn't know how to handle these custom ops

**Solution:**
Replace with standard ONNX ops:
```rust
// Gaussian blur via Conv with pre-computed kernel
let kernel = gaussian_kernel(sigma, kernel_size);
Proto::node("Conv", &[input], &[blurred], &[kernel, bias]);

// Edge detection via Laplacian
let laplacian = [0, 1, 0, 1, -4, 1, 0, 1, 0];
Proto::node("Conv", &[input], &[edges], &[laplacian, bias]);
```

---

### 2. Vignetting Block (FIXABLE)

**Current Implementation:**
```rust
Proto::node("Range", &[], &[grid_x], &[start, limit, delta]);
Proto::node("Mul", &[input, gain_map], &[output], &[]);
```

**Problem:**
- `gain_map` initializer shape `[1, 1, H, W]` doesn't match input `[1, 3, H, W]`
- MNN can't broadcast automatically

**Solution:**
Broadcast gain_map to match input channels:
```rust
// Replicate gain_map across channels
let gain_3ch = Proto::node("Concat", &[gain_map, gain_map, gain_map], &[gain_3ch], &[axis=1]);
Proto::node("Mul", &[input, gain_3ch], &[output], &[]);
```

---

### 3. Colorspace Block (FIXABLE)

**Current Implementation:**
```rust
Proto::node("Div", &[s, max], &[output], &[]);
```

**Problem:**
- Division by zero when `max = 0` (black pixels)
- No epsilon guard

**Solution:**
Add epsilon to denominator:
```rust
let eps = Proto::tensor_float("eps", &[1], &[1e-6]);
let max_eps = Proto::node("Add", &[max, eps], &[max_eps], &[]);
Proto::node("Div", &[s, max_eps], &[output], &[]);
```

---

### 4. Wavelet Denoise Block (COMPLEX)

**Current Implementation:**
Multi-pass wavelet thresholding:
1. Horizontal pass: AvgPool → Sub → Abs → Threshold → Sign → Mul
2. Vertical pass: Same as horizontal
3. Combine: Add(input, -residual + thresholded)

**Problem:**
- Many intermediate tensors create complex computation graph
- MNN optimizer may not handle all passes correctly
- Shape mismatches between passes

**Solution:**
Simplify to single-pass box filter:
```rust
// Simple spatial denoising
let mean = Proto::node("AveragePool", &[input], &[mean], &[kernel=3]);
let residual = Proto::node("Sub", &[input, mean], &[residual], &[]);
let denoised = Proto::node("Sub", &[input, residual], &[output], &[]);
```

---

### 5. Auto Contrast Block (FIXABLE)

**Current Implementation:**
```rust
Proto::node("Add", &[input, lift], &[lifted], &[]);
Proto::node("Sub", &[lifted, half], &[centered], &[]);
Proto::node("Mul", &[centered, contrast], &[stretched], &[]);
Proto::node("Add", &[stretched, half], &[output], &[]);
```

**Problem:**
- `half` tensor not defined as initializer
- `contrast` tensor not defined as initializer
- Dynamic shapes not handled

**Solution:**
Add proper initializers:
```rust
fn initializers(&self) -> Vec<Vec<u8>> {
    vec![
        Proto::tensor_float("half", &[1], &[0.5]),
        Proto::tensor_float("contrast", &[1], &[self.contrast]),
        Proto::tensor_float("lift", &[1], &[self.shadow_lift]),
    ]
}
```

---

## Recommended Fixes (Priority Order)

### Priority 1: Enable Vignetting
```rust
// Fix: Broadcast gain_map to 3 channels
let gain_3ch = Proto::node("Tile", &[gain_map], &[gain_3ch], 
    &[Proto::attribute_ints("repeats", &[1, 3, 1, 1])]);
```

### Priority 2: Enable Auto Contrast
```rust
// Fix: Add missing initializers
fn initializers(&self) -> Vec<Vec<u8>> {
    vec![
        Proto::tensor_float("AutoContrast/half", &[1], &[0.5]),
        Proto::tensor_float("AutoContrast/contrast", &[1], &[self.contrast]),
    ]
}
```

### Priority 3: Enable Colorspace
```rust
// Fix: Add epsilon for division
let eps = Proto::tensor_float("ColorSpace/eps", &[1], &[1e-6]);
```

### Priority 4: Enable Wavelet Denoise
```rust
// Fix: Simplify to single-pass
let mean = Proto::node("AveragePool", &[input], &[mean], 
    &[Proto::attribute_ints("kernel_shape", &[3, 3])]);
```

### Priority 5: Enable Bilateral (Requires Rewrite)
```rust
// Fix: Replace GaussianBlur/Sobel with Conv
let kernel = gaussian_kernel(self.sigma_spatial, self.kernel_size);
Proto::node("Conv", &[input], &[blurred], &[kernel, bias]);
```

---

## Testing Strategy

1. **Enable one block at a time** - Don't enable all at once
2. **Test with small resolution** - Start with 64×64, not 4K
3. **Verify tensor shapes** - Log all input/output shapes
4. **Use CPU first** - Test with MNN CPU backend before Vulkan

---

## Current Status

| Block | CPU Working | GPU Working | Priority |
|-------|-------------|-------------|----------|
| bilateral | ❌ | ❌ | Low (complex) |
| vignetting | ✅ | ❌ | High (easy fix) |
| colorspace | ✅ | ❌ | Medium (epsilon) |
| wavelet_denoise | ✅ | ❌ | Medium (simplify) |
| auto_contrast | ✅ | ❌ | High (initializers) |

**Recommendation:** Fix vignetting and auto_contrast first (easy wins), then colorspace and wavelet_denoise, leave bilateral for last.
