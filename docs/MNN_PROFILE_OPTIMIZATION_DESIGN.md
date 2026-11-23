# MNN Profile Optimization Design

## Overview
This document describes the layered optimization pipeline for converting SoftISP pipeline profiles into optimized MNN models with custom `isp.*` opsets. The process is additive and preserves backward compatibility—existing workflows remain functional while new optimizations are layered on top.

## Optimization Pipeline

### 1. Pipeline Profile → ISP Block Function
- **Input**: A SoftISP pipeline profile (e.g., LITE, MED, HEAVY, PRO, UNIFIED) defining a sequence of ISP blocks.
- **Mapping**: Each block in the profile is mapped to its corresponding ISP block function (e.g., `UnpackCfaBlock` → `isp.unpack_packed`, `FcsBlock` → `isp.fcs`).
- **Output**: A logical ISP block sequence representing the pipeline's functional intent.

### 2. libmnnconvertdeps.so Recognition
- **Input**: The logical ISP block sequence from step 1.
- **Process**: During ONNX-to-MNN conversion via `libmnnconvertdeps.so`, the converter scans the ONNX graph for patterns matching predefined ISP opsets.
- **Recognition**: The converter uses fusion rules (see `PIPELINE_BLOCKS.md` → MNN Fusion Rules) to identify substitutable patterns (e.g., `Sub → ReLU6` → `isp.fcs`).
- **Output**: An intermediate MNN model where recognized patterns are tagged with `isp.*` opset placeholders.

### 3. Micro Optimization: Elementwise Op Fusion
- **Scope**: Within individual ISP blocks or adjacent pointwise operations.
- **Techniques**:
  - Fuse consecutive pointwise operations (e.g., `Mul → Add` → scaled addition).
  - Replace generic ops with specialized `isp.*` opsets when patterns match (e.g., `Sub → ReLU6 → Mul` → `isp.fcs` with scaled output).
  - Optimize data types (e.g., fold constants into weights where possible).
- **Output**: An MNN model with reduced operator count and improved arithmetic intensity.

### 4. Macro Optimization: Structural Block Fusion
- **Scope**: Across ISP block boundaries, leveraging semantic knowledge of the pipeline.
- **Techniques**:
  - Fuse adjacent blocks with compatible dataflow (e.g., `DemosaicBlock` + `CcmBlock` → `DemosaicCcmBlock` via `isp.demosaic_ccm`).
  - Eliminate intermediate tensors by merging Conv layers (e.g., fusing 1x1 demosaic and CCM matrices).
  - Reorder independent ops for better memory locality (guided by data dependency analysis).
- **Output**: A structurally optimized MNN model with fewer sessions and intermediate tensors.

### 5. Emit Optimized .mnn with isp.* Opsets
- **Final Output**: An `.mnn` file where:
  - Standard MNN ops are used for non-ISP patterns.
  - Recognized ISP patterns are replaced with custom `isp.*` opsets (e.g., `isp.unpack_packed`, `isp.fcs`, `isp.argb_convert`).
  - The model retains full functional equivalence to the original ONNX graph.
  - Custom opsets are implemented in MNN via plugins (e.g., `libMNN_Vulkan.so` for GPU acceleration).

## Backward Compatibility
- **Additive Design**: The optimization layer is optional and off by default. Existing workflows using standard MNN ops remain unchanged.
- **Fallback Path**: If an `isp.*` opset is unavailable (e.g., missing plugin), the converter falls back to the original unfused ONNX pattern.
- **Versioning**: Custom opsets are versioned; older MNN runtimes ignore unknown opsets and fall back to the base pattern.

## Implementation Notes
- The conversion logic resides in `libmnnconvertdeps.so` (built from `mnn_convert_api.cpp`).
- ISP opset definitions are registered via MNN's plugin mechanism (see `gResistor → MNNInsertExtraRuntimeCreator()`).
- Profiling-driven: Macro fusion decisions can be guided by hardware-specific cost models (e.g., favoring fusion on Vulkan for Conv-heavy blocks).

## Diagram
```
Pipeline Profile
        ↓
ISP Block Function Mapping
        ↓
[libmnnconvertdeps.so] → Pattern Recognition → Intermediate MNN (with isp.* placeholders)
        ↓
Micro Optimization (Elementwise Fusion)
        ↓
Macro Optimization (Structural Fusion)
        ↓
Optimized .mnn (isp.* opsets + standard MNN ops)
```

---
*This design enables progressive optimization: from basic ONNX conversion to hardware-specific ISP acceleration, without breaking existing integrations.*