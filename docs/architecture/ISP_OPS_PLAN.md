# Plan: Add Post-Processing ISP Ops to MNNConvertDeps

## Overview

Add 5 new ISP ops to MNN's VulkanFuse backend for GPU-accelerated post-processing:

1. `isp.vignetting` - Lens shading correction
2. `isp.auto_contrast` - Auto contrast adjustment
3. `isp.colorspace` - RGB/HSV conversion
4. `isp.wavelet_denoise` - Wavelet denoising
5. `isp.bilateral` - Edge-preserving denoising

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    MNN ISP Op Pipeline                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ONNX Graph                                                    │
│      │                                                          │
│      ▼                                                          │
│  IspOnnxOps.cpp (Op Registration)                              │
│      │                                                          │
│      ▼                                                          │
│  IspChainFusion.cpp (Fusion Rules)                             │
│      │                                                          │
│      ▼                                                          │
│  VulkanFuse.cpp (Execution)                                    │
│      │                                                          │
│      ▼                                                          │
│  SPIR-V Shaders (GPU Compute)                                  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## File Changes Required

### 1. GLSL Shaders (vulkan_isp/)

Create compute shaders for each op:

```glsl
// shader_vignetting.comp
#version 450
layout(local_size_x = 16, local_size_y = 16) in;
layout(set = 0, binding = 0) uniform sampler2D input_tex;
layout(set = 0, binding = 1) writeonly uniform image2D output_tex;
layout(set = 0, binding = 2) uniform sampler2D gain_map;

void main() {
    ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
    vec4 color = texelFetch(input_tex, pos, 0);
    vec4 gain = texelFetch(gain_map, pos, 0);
    imageStore(output_tex, pos, color * gain);
}
```

### 2. SPIR-V Embedding (isp_spirv_embedded.h)

Compile shaders and embed:
```bash
glslangValidator -V shader_vignetting.comp -o shader_vignetting.spv
python3 embed_spv.py shader_vignetting.spv > g_vignetting_spv[]
```

### 3. ONNX Op Registration (IspOnnxOps.cpp)

Add op classes:
```cpp
class IspVignetting : public OnnxExtraManager::Transform {
public:
    virtual EXPRP onExecute(EXPRP expr) const override {
        auto op = expr->get();
        auto newVar = createVulkanFuseOp(expr, op, "vignetting");
        return newVar->expr().first;
    }
};
```

### 4. Fusion Rules (IspChainFusion.cpp)

Add detection and fusion:
```cpp
// R13: Vignetting fusion
// Pattern: Mul(input, gain_map) → isp.vignetting
bool tryVignettingChain(...) {
    // Detect Mul with pre-computed gain_map
    // Replace with isp.vignetting Extra op
}
```

### 5. Vulkan Execution (VulkanFuse.cpp)

Add shader dispatch:
```cpp
if (extra->type == "isp.vignetting") {
    // Bind input, gain_map, output
    // Dispatch compute shader
}
```

---

## Detailed Implementation Plan

### Phase 1: Vignetting (Easy - Simple Multiply)

**GLSL Shader:**
```glsl
#version 450
layout(local_size_x = 16, local_size_y = 16) in;

layout(set = 0, binding = 0, rgba32f) readonly uniform image2D input_img;
layout(set = 0, binding = 1, rgba32f) writeonly uniform image2D output_img;
layout(set = 0, binding = 2, rgba32f) readonly uniform image2D gain_map;

void main() {
    ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
    if (pos.x >= inputWidth || pos.y >= inputHeight) return;
    
    vec4 color = imageLoad(input_img, pos);
    vec4 gain = imageLoad(gain_map, pos);
    
    imageStore(output_img, pos, color * gain);
}
```

**ONNX Pattern:**
```
Mul(input, gain_map) → output
```

**Fusion Rule:**
- Detect: Mul node with second input being a Constant (gain_map)
- Replace: isp.vignetting Extra op

---

### Phase 2: Auto Contrast (Medium - Simple Math)

**GLSL Shader:**
```glsl
#version 450
layout(local_size_x = 16, local_size_y = 16) in;

layout(set = 0, binding = 0, rgba32f) readonly uniform image2D input_img;
layout(set = 0, binding = 1, rgba32f) writeonly uniform image2D output_img;

uniform float shadow_lift;    // 0.0-0.2
uniform float contrast;       // 0.5-2.0
uniform float highlight_comp; // 0.0-0.5

void main() {
    ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
    vec3 color = imageLoad(input_img, pos).rgb;
    
    // Shadow lift
    color += shadow_lift;
    
    // Contrast stretch
    color = (color - 0.5) * contrast + 0.5;
    
    // Highlight compression
    if (highlight_comp > 0.0) {
        color = color * (1.0 - highlight_comp * max(0.0, color - 0.5));
    }
    
    imageStore(output_img, pos, vec4(clamp(color, 0.0, 1.0), 1.0));
}
```

**ONNX Pattern:**
```
Add(input, lift) → Sub(x, 0.5) → Mul(x, contrast) → Add(x, 0.5)
```

**Fusion Rule:**
- Detect: Add → Sub → Mul → Add chain
- Replace: isp.auto_contrast Extra op

---

### Phase 3: Colorspace (Medium - Per-pixel Math)

**GLSL Shader (RGB→HSV):**
```glsl
#version 450
layout(local_size_x = 16, local_size_y = 16) in;

layout(set = 0, binding = 0, rgba32f) readonly uniform image2D input_img;
layout(set = 0, binding = 1, rgba32f) writeonly uniform image2D output_img;

vec3 rgb2hsv(vec3 rgb) {
    float maxc = max(rgb.r, max(rgb.g, rgb.b));
    float minc = min(rgb.r, min(rgb.g, rgb.b));
    float d = maxc - minc;
    
    float h = 0.0;
    if (d > 0.001) {
        if (maxc == rgb.r) h = (rgb.g - rgb.b) / d;
        else if (maxc == rgb.g) h = 2.0 + (rgb.b - rgb.r) / d;
        else h = 4.0 + (rgb.r - rgb.g) / d;
        h *= 60.0;
        if (h < 0.0) h += 360.0;
    }
    
    float s = (maxc > 0.001) ? d / maxc : 0.0;
    float v = maxc;
    
    return vec3(h / 360.0, s, v);
}

void main() {
    ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
    vec3 rgb = imageLoad(input_img, pos).rgb;
    vec3 hsv = rgb2hsv(rgb);
    imageStore(output_img, pos, vec4(hsv, 1.0));
}
```

---

### Phase 4: Wavelet Denoise (Complex - Multi-pass)

**Approach:** Simplified single-pass box filter + threshold

**GLSL Shader:**
```glsl
#version 450
layout(local_size_x = 16, local_size_y = 16) in;

layout(set = 0, binding = 0, rgba32f) readonly uniform image2D input_img;
layout(set = 0, binding = 1, rgba32f) writeonly uniform image2D output_img;

uniform float sigma;

void main() {
    ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
    vec3 sum = vec3(0.0);
    float count = 0.0;
    
    // 3x3 box filter
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            ivec2 p = pos + ivec2(dx, dy);
            sum += imageLoad(input_img, p).rgb;
            count += 1.0;
        }
    }
    
    vec3 mean = sum / count;
    vec3 color = imageLoad(input_img, pos).rgb;
    vec3 residual = color - mean;
    
    // Soft threshold
    vec3 thresholded = sign(residual) * max(abs(residual) - sigma, 0.0);
    
    imageStore(output_img, pos, vec4(mean + thresholded, 1.0));
}
```

---

### Phase 5: Bilateral (Hard - Complex Algorithm)

**Approach:** Separable Gaussian + edge-aware blend

**GLSL Shader:**
```glsl
#version 450
layout(local_size_x = 16, local_size_y = 16) in;

layout(set = 0, binding = 0, rgba32f) readonly uniform image2D input_img;
layout(set = 0, binding = 1, rgba32f) writeonly uniform image2D output_img;

uniform float sigma_spatial;
uniform float sigma_range;

// Gaussian weight
float gaussian(float x, float sigma) {
    return exp(-x * x / (2.0 * sigma * sigma));
}

void main() {
    ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
    vec3 center = imageLoad(input_img, pos).rgb;
    vec3 sum = vec3(0.0);
    float weight_sum = 0.0;
    
    int radius = int(ceil(sigma_spatial * 2.0));
    
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            ivec2 p = pos + ivec2(dx, dy);
            vec3 sample = imageLoad(input_img, p).rgb;
            
            float spatial_w = gaussian(float(dx*dx + dy*dy), sigma_spatial);
            float range_w = gaussian(length(sample - center), sigma_range);
            float w = spatial_w * range_w;
            
            sum += sample * w;
            weight_sum += w;
        }
    }
    
    imageStore(output_img, pos, vec4(sum / weight_sum, 1.0));
}
```

---

## Implementation Order

| Priority | Op | Difficulty | GPU Benefit |
|----------|-----|------------|-------------|
| 1 | vignetting | Easy | High (simple multiply) |
| 2 | auto_contrast | Easy | High (simple math) |
| 3 | colorspace | Medium | Medium (per-pixel) |
| 4 | wavelet_denoise | Medium | Medium (box filter) |
| 5 | bilateral | Hard | High (expensive algo) |

## Testing Strategy

1. **Unit test each shader** - Small resolution (64×64)
2. **Compare CPU vs GPU** - Verify output matches
3. **Benchmark** - Measure speedup vs CPU
4. **Integration test** - Full pipeline with new ops

## Estimated Effort

- Phase 1 (vignetting): 2-3 hours
- Phase 2 (auto_contrast): 2-3 hours
- Phase 3 (colorspace): 3-4 hours
- Phase 4 (wavelet_denoise): 4-5 hours
- Phase 5 (bilateral): 6-8 hours

**Total: ~20-25 hours**
