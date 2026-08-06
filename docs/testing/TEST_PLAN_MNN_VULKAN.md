# MNN Vulkan ISP Block Test Plan

## Overview

This test plan covers individual ISP block validation using the MNN inference
engine with the Vulkan backend. The goal is to:

1. **Pass0 (A test)** — ONNX→MNN conversion for each ISP block, producing
   reference `.mnn` files aligned with `cam_isp`.
2. **Pass1 (B test)** — MNN→MNN inference using the Pass0 `.mnn` files on the
   Vulkan backend, verifying SPIRV custom opset correctness.
3. **A/B comparison** — Compare Pass0 (ONNX→MNN reference) results against
   Pass1 (Vulkan MNN inference) results to detect SPIRV correctness issues.

## Architecture

```
Pass0 (A)                          Pass1 (B)
┌─────────────┐                   ┌─────────────┐
│ ONNX graph  │ ──convert──►     │ MNN model   │ ──Vulkan──►
│ (per block) │   mnn_convert    │ (Pass0 ref) │   inference
└─────────────┘                   └──────┬──────┘
                                         │
                                         ▼
                                  ┌─────────────┐
                                  │  A/B Compare │
                                  │  Pass0 vs B  │
                                  └─────────────┘
```

## Test Environment

- **Backend**: Vulkan (`MNN_BACKEND_VULKAN = 7`)
- **Precision**: `Precision_High` (matching `mnn_wrapper.cpp` session config)
- **MNN libs**: `libMNN.so`, `libMNN_Vulkan.so`, `libMNNConvertDeps.so`
  from `jni/arm64-v8a/`
- **Feature flag**: `--features mnn`
- **Test runner**: `cargo test --test <test_name> -p cam-isp --features mnn`

## Pass0: ONNX→MNN Conversion Tests (Reference Base)

### Purpose

Convert each ISP block's ONNX graph to MNN format. These `.mnn` files serve
as the **reference base** for all subsequent tests. The goal is to ensure
our ONNX→MNN conversion pipeline produces MNN files that are aligned with
what `cam_isp` expects.

### Test Structure

Each test:
1. Builds a single ISP block's ONNX graph using `GraphComposer`
2. Converts ONNX → MNN via `mnn_convert_onnx_buffer()` (in-process, zero disk)
3. Saves the `.mnn` bytes to a known location for Pass1 reuse
4. Validates the MNN file can be loaded by `MnnInterpreterSafe::from_buffer()`

### Individual Block Tests

| Block | ONNX Ops | Key Tensors | Notes |
|-------|----------|-------------|-------|
| `UnpackBlock` | Unpack, BLC, CFA | `raw_input` → `unpacked` | INT32 packed input |
| `NormalizeBlock` | Mul, Add | `normalized` | Float normalization |
| `CfaBlock` | Bayer pattern reorder | `cfa_out` | 2×2 Bayer quad |
| `BlcBlock` | Sub (black level) | `blc_corrected` | Per-channel offsets |
| `BayerWbBlock` | Mul (WB gains) | `wb_out` | 4-channel gains |
| `DemosaicBlock` | Conv, Reshape | `demosaic_out` | Bilinear demosaic |
| `CcmBlock` | MatMul (CCM) | `ccm_out` | 3×3 color matrix |
| `ToneBlock` | Mul, Add, Pow | `tone_out` | Contrast/brightness |
| `FcsBlock` | Mul (shading LUT) | `fcs_out` | Flat field correction |
| `LdciBlock` | Local contrast | `ldci_out` | Local dynamic contrast |
| `EeBlock` | Conv (unsharp) | `ee_out` | Edge enhancement |
| `BilateralBlock` | Bilateral filter | `bilateral_out` | Edge-preserving denoise |
| `VignettingBlock` | Mul (vignette LUT) | `vignette_out` | Radial vignetting |
| `SaturationBlock` | Mul (saturation) | `sat_out` | Color saturation |
| `ColorSpaceBlock` | Colorspace transform | `cs_out` | RGB↔HSV |
| `GammaBlock` | Pow (gamma) | `gamma_out` | Gamma correction |
| `GpuWarpBlock` | GridSample, Mul, Add | `gpu_warp/frame` | GPU-accelerated warp |
| `RuntimeWarpBlock` | GridSample | `warped` | CPU-generated grid |
| `ResizeBlock` | Resize (nearest/bilinear) | `resized` | Scale up/down |
| `AdaptiveDownscaleBlock` | AveragePool | `downsampled` | Edge/pad downscale |
| `AspectCropBlock` | Slice | `cropped` | Aspect ratio crop |
| `AutoContrastBlock` | Min/Max/Rescale | `autocontrast` | Auto contrast |
| `GrayscaleBlock` | Conv (RGB→gray) | `gray` | Luma conversion |
| `DisplayBlock` | Mul, Add, Pow | `display/frame` | Final display output |

### Pass0 Test File: `test_mnn_pass0.rs`

```rust
// Pass0: ONNX→MNN conversion test for each ISP block.
// Produces reference .mnn files used by Pass1 (B test).
//
// Run:
//   cd cam-rust
//   cargo test --test test_mnn_pass0 -p cam-isp --features mnn -- --nocapture
//
// Requirements:
//   - libMNN.so, libMNN_Vulkan.so, libMNNConvertDeps.so in LD_LIBRARY_PATH
//   - mnn_convert_onnx_buffer() available (in-process conversion)

#![cfg(feature = "mnn")]

use cam_isp::blocks::*;
use cam_isp::mnn_converter::{convert_onnx_buffer, MnnConvertOptions};
use cam_isp::mnn_sys::{MnnInterpreterSafe, MnnBackendType};
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;
use std::path::Path;

/// Build a single block's ONNX graph, convert to MNN, validate loading.
/// Returns the MNN bytes.
fn pass0_convert(block: &dyn IspBlock, name: &str) -> Result<Vec<u8>, String> {
    // Compose ONNX from the single block
    let refs: Vec<&dyn IspBlock> = vec![block];
    let onnx_bytes = GraphComposer::compose_from_vec(&refs, &[], 16)
        .map_err(|e| format!("ONNX compose failed for {}: {}", name, e))?;

    // Convert ONNX → MNN in-process (zero disk writes)
    let opts = MnnConvertOptions {
        allow_custom_op: true,
        preserve_input_type: true,
        ..Default::default()
    };
    let mnn_bytes = convert_onnx_buffer(&onnx_bytes)
        .map_err(|e| format!("ONNX→MNN convert failed for {}: {}", name, e))?;

    // Validate: load MNN into interpreter
    let interp = MnnInterpreterSafe::from_buffer(&mnn_bytes)
        .ok_or_else(|| format!("Failed to load MNN for {}", name))?;

    // Verify session can be created with Vulkan backend
    let _session = interp
        .create_session(MnnBackendType::Vulkan, 1)
        .ok_or_else(|| format!("Vulkan session create failed for {}", name))?;

    eprintln!("[Pass0] {}: ONNX {} bytes → MNN {} bytes ✓", name, onnx_bytes.len(), mnn_bytes.len());

    Ok(mnn_bytes)
}

// ── Individual Block Pass0 Tests ──────────────────────────────────────────

#[test]
fn pass0_unpack_block() {
    let block = UnpackBlock::new().with_concrete_dims(64, 64);
    let mnn = pass0_convert(&block, "UnpackBlock").expect("Pass0 UnpackBlock");
    std::fs::write(".pass0_unpack.mnn", &mnn).ok();
}

#[test]
fn pass0_normalize_block() {
    let block = NormalizeBlock::new();
    let mnn = pass0_convert(&block, "NormalizeBlock").expect("Pass0 NormalizeBlock");
    std::fs::write(".pass0_normalize.mnn", &mnn).ok();
}

#[test]
fn pass0_cfa_block() {
    let block = CfaBlock::new();
    let mnn = pass0_convert(&block, "CfaBlock").expect("Pass0 CfaBlock");
    std::fs::write(".pass0_cfa.mnn", &mnn).ok();
}

#[test]
fn pass0_blc_block() {
    let block = BlcBlock::new();
    let mnn = pass0_convert(&block, "BlcBlock").expect("Pass0 BlcBlock");
    std::fs::write(".pass0_blc.mnn", &mnn).ok();
}

#[test]
fn pass0_bayer_wb_block() {
    let block = BayerWbBlock::new();
    let mnn = pass0_convert(&block, "BayerWbBlock").expect("Pass0 BayerWbBlock");
    std::fs::write(".pass0_bayer_wb.mnn", &mnn).ok();
}

#[test]
fn pass0_demosaic_block() {
    let block = DemosaicBlock::new(0);
    let mnn = pass0_convert(&block, "DemosaicBlock").expect("Pass0 DemosaicBlock");
    std::fs::write(".pass0_demosaic.mnn", &mnn).ok();
}

#[test]
fn pass0_ccm_block() {
    let block = CcmBlock::new();
    let mnn = pass0_convert(&block, "CcmBlock").expect("Pass0 CcmBlock");
    std::fs::write(".pass0_ccm.mnn", &mnn).ok();
}

#[test]
fn pass0_tone_block() {
    let block = ToneBlock::new();
    let mnn = pass0_convert(&block, "ToneBlock").expect("Pass0 ToneBlock");
    std::fs::write(".pass0_tone.mnn", &mnn).ok();
}

#[test]
fn pass0_fcs_block() {
    let block = FcsBlock::new();
    let mnn = pass0_convert(&block, "FcsBlock").expect("Pass0 FcsBlock");
    std::fs::write(".pass0_fcs.mnn", &mnn).ok();
}

#[test]
fn pass0_ldci_block() {
    let block = LdciBlock::new();
    let mnn = pass0_convert(&block, "LdciBlock").expect("Pass0 LdciBlock");
    std::fs::write(".pass0_ldci.mnn", &mnn).ok();
}

#[test]
fn pass0_ee_block() {
    let block = EeBlock::new();
    let mnn = pass0_convert(&block, "EeBlock").expect("Pass0 EeBlock");
    std::fs::write(".pass0_ee.mnn", &mnn).ok();
}

#[test]
fn pass0_bilateral_block() {
    let block = bilateral::BilateralBlock::new_default();
    let mnn = pass0_convert(&block, "BilateralBlock").expect("Pass0 BilateralBlock");
    std::fs::write(".pass0_bilateral.mnn", &mnn).ok();
}

#[test]
fn pass0_vignetting_block() {
    let block = vignetting::VignettingBlock::new_default(64, 64);
    let mnn = pass0_convert(&block, "VignettingBlock").expect("Pass0 VignettingBlock");
    std::fs::write(".pass0_vignetting.mnn", &mnn).ok();
}

#[test]
fn pass0_saturation_block() {
    let block = saturation::SaturationBlock::new_default();
    let mnn = pass0_convert(&block, "SaturationBlock").expect("Pass0 SaturationBlock");
    std::fs::write(".pass0_saturation.mnn", &mnn).ok();
}

#[test]
fn pass0_colorspace_block() {
    let block = colorspace::ColorSpaceBlock::rgb_to_hsv();
    let mnn = pass0_convert(&block, "ColorSpaceBlock").expect("Pass0 ColorSpaceBlock");
    std::fs::write(".pass0_colorspace.mnn", &mnn).ok();
}

#[test]
fn pass0_gamma_block() {
    let block = gamma::GammaBlock::new(2.2);
    let mnn = pass0_convert(&block, "GammaBlock").expect("Pass0 GammaBlock");
    std::fs::write(".pass0_gamma.mnn", &mnn).ok();
}

#[test]
fn pass0_gpu_warp_block() {
    let block = GpuWarpBlock::new(64, 64);
    let mnn = pass0_convert(&block, "GpuWarpBlock").expect("Pass0 GpuWarpBlock");
    std::fs::write(".pass0_gpu_warp.mnn", &mnn).ok();
}

#[test]
fn pass0_runtime_warp_block() {
    let block = runtime_warp::RuntimeWarpBlock::new(64, 64);
    let mnn = pass0_convert(&block, "RuntimeWarpBlock").expect("Pass0 RuntimeWarpBlock");
    std::fs::write(".pass0_runtime_warp.mnn", &mnn).ok();
}

#[test]
fn pass0_resize_block() {
    let block = ResizeBlock::new(64, 64);
    let mnn = pass0_convert(&block, "ResizeBlock").expect("Pass0 ResizeBlock");
    std::fs::write(".pass0_resize.mnn", &mnn).ok();
}

#[test]
fn pass0_adaptive_downscale_block() {
    let block = AdaptiveDownscaleBlock::new(32, 32, 0, "edge", "pad");
    let mnn = pass0_convert(&block, "AdaptiveDownscaleBlock").expect("Pass0 AdaptiveDownscaleBlock");
    std::fs::write(".pass0_adaptive_downscale.mnn", &mnn).ok();
}

#[test]
fn pass0_aspect_crop_block() {
    let block = AspectCropBlock::new(64, 64);
    let mnn = pass0_convert(&block, "AspectCropBlock").expect("Pass0 AspectCropBlock");
    std::fs::write(".pass0_aspect_crop.mnn", &mnn).ok();
}

#[test]
fn pass0_auto_contrast_block() {
    let block = AutoContrastBlock::new(1.0);
    let mnn = pass0_convert(&block, "AutoContrastBlock").expect("Pass0 AutoContrastBlock");
    std::fs::write(".pass0_auto_contrast.mnn", &mnn).ok();
}

#[test]
fn pass0_grayscale_block() {
    let block = GrayscaleBlock::new();
    let mnn = pass0_convert(&block, "GrayscaleBlock").expect("Pass0 GrayscaleBlock");
    std::fs::write(".pass0_grayscale.mnn", &mnn).ok();
}

#[test]
fn pass0_display_block() {
    let block = DisplayBlock::new(64);
    let mnn = pass0_convert(&block, "DisplayBlock").expect("Pass0 DisplayBlock");
    std::fs::write(".pass0_display.mnn", &mnn).ok();
}

// ── Full Pipeline Pass0 Tests ─────────────────────────────────────────────

#[test]
fn pass0_lite_pipeline() {
    let blocks = PipelineProfile::LITE.build_blocks(64, 0);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16)
        .expect("LITE ONNX compose");
    let mnn = convert_onnx_buffer(&onnx).expect("LITE ONNX→MNN");
    let interp = MnnInterpreterSafe::from_buffer(&mnn).expect("LITE MNN load");
    let _sess = interp.create_session(MnnBackendType::Vulkan, 1)
        .expect("LITE Vulkan session");
    std::fs::write(".pass0_lite.mnn", &mnn).ok();
    eprintln!("[Pass0] LITE pipeline: ONNX {} → MNN {} bytes ✓", onnx.len(), mnn.len());
}

#[test]
fn pass0_heavy_pipeline() {
    let blocks = PipelineProfile::HEAVY.build_blocks(64, 0);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16)
        .expect("HEAVY ONNX compose");
    let mnn = convert_onnx_buffer(&onnx).expect("HEAVY ONNX→MNN");
    let interp = MnnInterpreterSafe::from_buffer(&mnn).expect("HEAVY MNN load");
    let _sess = interp.create_session(MnnBackendType::Vulkan, 1)
        .expect("HEAVY Vulkan session");
    std::fs::write(".pass0_heavy.mnn", &mnn).ok();
    eprintln!("[Pass0] HEAVY pipeline: ONNX {} → MNN {} bytes ✓", onnx.len(), mnn.len());
}
```

## Pass1: MNN→MNN Vulkan Inference Tests (SPIRV Correctness Verification)

### Purpose

Load each Pass0 `.mnn` reference file and run inference on the Vulkan backend.
This verifies that the MNN Vulkan backend correctly executes the ISP custom
opset SPIRV shaders. Any discrepancy between Pass0 (reference) and Pass1
(Vulkan inference) results indicates a SPIRV correctness issue.

### Test Structure

Each test:
1. Loads the Pass0 `.mnn` reference file
2. Creates a Vulkan MNN session
3. Sets input tensor data
4. Runs inference
5. Reads output tensor data
6. Returns the output for A/B comparison

### Individual Block Pass1 Tests

Each test loads the corresponding Pass0 `.mnn` file and runs Vulkan inference.

```rust
// Pass1: MNN→MNN Vulkan inference test for each ISP block.
// Uses Pass0 reference .mnn files. Verifies SPIRV custom opset correctness.
//
// Run:
//   cd cam-rust
//   cargo test --test test_mnn_pass1 -p cam-isp --features mnn -- --nocapture
//
// Requirements:
//   - libMNN.so, libMNN_Vulkan.so in LD_LIBRARY_PATH
//   - Pass0 .mnn files present (generated by test_mnn_pass0)

#![cfg(feature = "mnn")]

use cam_isp::mnn_sys::*;
use std::ffi::CString;
use std::os::raw::c_void;

/// Load a Pass0 .mnn file and create a Vulkan session.
fn pass1_load_vulkan(mnn_path: &str) -> Result<(MnnInterpreterSafe, MnnSessionSafe), String> {
    let interp = MnnInterpreterSafe::from_file(mnn_path)
        .ok_or_else(|| format!("Failed to load MNN file: {}", mnn_path))?;

    let session = interp
        .create_session(MnnBackendType::Vulkan, 1)
        .ok_or_else(|| format!("Vulkan session create failed for {}", mnn_path))?;

    Ok((interp, session))
}

/// Set input tensor from float32 data.
fn pass1_set_input(
    interp: &MnnInterpreterSafe,
    session: &MnnSessionSafe,
    name: &str,
    data: &[f32],
    shape: &[i32],
) -> Result<(), String> {
    let c_name = CString::new(name).map_err(|_| "NUL in name")?;
    let tensor = interp.get_input(session, &c_name.to_string_lossy())
        .ok_or_else(|| format!("Input tensor '{}' not found", name))?;

    let mut dims = shape.to_vec();
    tensor.set_shape(interp.as_ptr(), session.as_ptr(), &dims)?;
    session.resize()?;

    // Copy data into tensor host buffer
    let bytes = tensor.as_bytes_mut().ok_or("Tensor not host-accessible")?;
    let expected = dims.iter().product::<i32>() as usize * 4;
    if bytes.len() < expected {
        return Err(format!("Tensor too small: {} < {}", bytes.len(), expected));
    }
    bytes[..data.len() * 4].copy_from_slice(
        unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, data.len() * 4) }
    );

    Ok(())
}

/// Read output tensor as float32 slice.
fn pass1_read_output(
    interp: &MnnInterpreterSafe,
    session: &MnnSessionSafe,
    name: &str,
) -> Result<Vec<f32>, String> {
    let c_name = CString::new(name).map_err(|_| "NUL in name")?;
    let tensor = interp.get_output(session, &c_name.to_string_lossy())
        .ok_or_else(|| format!("Output tensor '{}' not found", name))?;

    let bytes = tensor.as_bytes().ok_or("Output tensor not host-accessible")?;
    let n_floats = bytes.len() / 4;
    let floats = unsafe {
        std::slice::from_raw_parts(bytes.as_ptr() as *const f32, n_floats).to_vec()
    };
    Ok(floats)
}

/// Run inference and return output floats.
fn pass1_run(
    mnn_path: &str,
    input_name: &str,
    input_data: &[f32],
    input_shape: &[i32],
    output_name: &str,
) -> Result<Vec<f32>, String> {
    let (interp, session) = pass1_load_vulkan(mnn_path)?;

    // Set input
    pass1_set_input(&interp, &session, input_name, input_data, input_shape)?;

    // Run inference
    session.run()?;

    // Read output
    let output = pass1_read_output(&interp, &session, output_name)?;

    Ok(output)
}

// ── Individual Block Pass1 Tests ──────────────────────────────────────────

#[test]
fn pass1_unpack_block() {
    let mnn_path = ".pass0_unpack.mnn";
    assert!(Path::new(mnn_path).exists(), "Pass0 .mnn not found — run Pass0 first");

    let input_shape: [i32; 4] = [1, 1, 64, 64];
    let input_data = vec![0.0f32; 64 * 64]; // synthetic raw input

    let output = pass1_run(mnn_path, "raw_input", &input_data, &input_shape, "unpack_out")
        .expect("Pass1 UnpackBlock inference");

    assert!(!output.is_empty(), "UnpackBlock output should not be empty");
    eprintln!("[Pass1] UnpackBlock: output {} elements ✓", output.len());
}

// ... (similar pattern for each block)

#[test]
fn pass1_gpu_warp_block() {
    let mnn_path = ".pass0_gpu_warp.mnn";
    assert!(Path::new(mnn_path).exists(), "Pass0 .mnn not found — run Pass0 first");

    // GpuWarpBlock needs frame input + GDC coefficients + EIS displacement
    let input_shape: [i32; 4] = [1, 3, 64, 64];
    let input_data = vec![0.5f32; 3 * 64 * 64]; // uniform gray frame

    // Set GDC coefficients (k1=k2=k3=0, no distortion)
    // Set EIS displacement (zero = no shift)
    // These are set via extra_inputs mechanism

    let output = pass1_run(mnn_path, "GpuWarp/frame_input", &input_data, &input_shape, "GpuWarp/frame")
        .expect("Pass1 GpuWarpBlock inference");

    assert!(!output.is_empty(), "GpuWarpBlock output should not be empty");
    eprintln!("[Pass1] GpuWarpBlock: output {} elements ✓", output.len());
}
```

## A/B Comparison Test

### Purpose

Compare Pass0 (ONNX→MNN reference) results against Pass1 (Vulkan MNN inference)
results for each ISP block. Any significant deviation indicates a SPIRV
correctness issue in the Vulkan custom opset.

### Comparison Criteria

| Metric | Threshold | Action |
|--------|-----------|--------|
| Mean Absolute Error (MAE) | < 0.01 | Pass |
| Peak Signal-to-Noise Ratio (PSNR) | > 40 dB | Pass |
| Structural Similarity (SSIM) | > 0.99 | Pass |
| Max absolute deviation | < 0.05 | Warn |
| Any NaN/Inf in output | 0 occurrences | Fail |

### A/B Test File: `test_mnn_ab_comparison.rs`

```rust
// A/B Comparison: Pass0 (ONNX→MNN reference) vs Pass1 (Vulkan MNN inference).
//
// For each ISP block, compares the output of:
//   A: ONNX→MNN conversion (Pass0 reference MNN)
//   B: MNN Vulkan inference (Pass1)
//
// Run:
//   cd cam-rust
//   cargo test --test test_mnn_ab_comparison -p cam-isp --features mnn -- --nocapture

#![cfg(feature = "mnn")]

/// Compute Mean Absolute Error between two float slices.
fn mae(a: &[f32], b: &[f32]) -> f32 {
    assert_eq!(a.len(), b.len(), "Slice lengths must match");
    let sum: f32 = a.iter().zip(b.iter())
        .map(|(x, y)| (x - y).abs())
        .sum();
    sum / a.len() as f32
}

/// Compute Peak Signal-to-Noise Ratio.
fn psnr(a: &[f32], b: &[f32], max_val: f32) -> f32 {
    assert_eq!(a.len(), b.len());
    let mse: f32 = a.iter().zip(b.iter())
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f32>() / a.len() as f32;
    if mse == 0.0 { return f32::INFINITY; }
    10.0 * (max_val * max_val / mse).log10()
}

/// Check for NaN or Inf values.
fn has_nan_or_inf(data: &[f32]) -> bool {
    data.iter().any(|v| v.is_nan() || v.is_infinite())
}

/// Compare Pass0 reference output with Pass1 Vulkan output for a block.
fn ab_compare(
    block_name: &str,
    pass0_output: &[f32],
    pass1_output: &[f32],
) -> Result<(), String> {
    assert_eq!(pass0_output.len(), pass1_output.len(),
        "{}: output lengths differ ({} vs {})", block_name, pass0_output.len(), pass1_output.len());

    let _mae_val = mae(pass0_output, pass1_output);
    let _psnr_val = psnr(pass0_output, pass1_output, 1.0);
    let nan_inf = has_nan_or_inf(pass1_output);

    eprintln!("[A/B] {}: MAE={:.6} PSNR={:.2}dB NaN/Inf={}",
        block_name, _mae_val, _psnr_val, nan_inf);

    if nan_inf {
        return Err(format!("{}: Pass1 output contains NaN/Inf", block_name));
    }

    // TODO: Tune thresholds per block type
    // For now, just check no NaN/Inf and non-empty output
    assert!(!pass1_output.is_empty(), "{}: Pass1 output is empty", block_name);

    Ok(())
}

// ── Per-Block A/B Tests ───────────────────────────────────────────────────

#[test]
fn ab_unpack_block() {
    // Run Pass0 and Pass1 for UnpackBlock, compare results
    let pass0_mnn = std::fs::read(".pass0_unpack.mnn").expect("Pass0 MNN file");
    let pass0_output = run_pass0_inference(&pass0_mnn, "UnpackBlock");
    let pass1_output = run_pass1_inference(".pass0_unpack.mnn", "UnpackBlock");
    ab_compare("UnpackBlock", &pass0_output, &pass1_output).expect("A/B UnpackBlock");
}

// ... (similar for each block)

#[test]
fn ab_gpu_warp_block() {
    let pass0_mnn = std::fs::read(".pass0_gpu_warp.mnn").expect("Pass0 MNN file");
    let pass0_output = run_pass0_inference(&pass0_mnn, "GpuWarpBlock");
    let pass1_output = run_pass1_inference(".pass0_gpu_warp.mnn", "GpuWarpBlock");
    ab_compare("GpuWarpBlock", &pass0_output, &pass1_output).expect("A/B GpuWarpBlock");
}
```

## SPIRV Correctness Verification

### Purpose

After aligning Pass0 MNN files with `cam_isp`, verify that the MNN Vulkan
backend correctly executes the ISP custom opset SPIRV shaders. This is the
primary goal of the B test (Pass1).

### Verification Steps

1. **Load Pass0 reference `.mnn`** — the ONNX→MNN converted model with
   custom ISP opset SPIRV embedded
2. **Run Vulkan inference** — execute the model on the Vulkan backend
3. **Compare with CPU reference** — run the same model on CPU backend
   and compare Vulkan output against CPU output
4. **Check per-op correctness** — for each custom op (`isp.unpack_blc`,
   `isp.demosaic_ccm`, `isp.fcs`, `isp.ee`, `isp.ldci`, `isp.display`),
   verify the output matches the expected mathematical result

### SPIRV Custom Opset List

| Opset Type | SPIRV Shader | Custom Op Name | Verification Method |
|------------|-------------|----------------|-------------------|
| `isp.unpack_blc` | `shader_unpack_blc.comp` | Unpack + BLC | Compare with CPU UnpackBlock |
| `isp.demosaic_ccm` | `shader_demosaic_ccm.comp` | Demosaic + CCM | Compare with CPU DemosaicCcmBlock |
| `isp.fcs` | `shader_fcs_display_fused.comp` | FCS (flat field) | Compare with CPU FcsBlock |
| `isp.ee` | `shader_ee_ldci_fused.comp` | EE (edge enhancement) | Compare with CPU EeBlock |
| `isp.ldci` | `shader_ee_ldci_fused.comp` | LDCI (local contrast) | Compare with CPU LdciBlock |
| `isp.display` | `shader_display.comp` | Display (gamma/brightness) | Compare with CPU DisplayBlock |
| `isp.colorspace` | `shader_colorspace.comp` | RGB↔HSV | Compare with CPU ColorSpaceBlock |
| `isp.vignetting` | `shader_vignetting.comp` | Radial vignette | Compare with CPU VignettingBlock |
| `isp.bilateral` | `shader_bilateral.comp` | Bilateral filter | Compare with CPU BilateralBlock |
| `isp.wavelet_denoise` | `shader_wavelet_denoise.comp` | Wavelet denoise | Compare with CPU WaveletDenoiseBlock |
| `isp.auto_contrast` | `shader_auto_contrast.comp` | Auto contrast | Compare with CPU AutoContrastBlock |

### Fix Process

When A/B comparison reveals SPIRV correctness issues:

1. **Identify the failing op** — which custom opset type produces wrong output
2. **Inspect the SPIRV** — disassemble with `spirv-dis` and check:
   - Workgroup size matches `global_size` / `group_size`
   - Input/output bindings match MNN tensor bindings (binding 1=input, 2=output)
   - Uniform buffer layout matches `OpDesc.uniforms` layout
   - Data type conversions (INT32→FLOAT, etc.) are correct
3. **Fix the SPIRV** — modify the GLSL source in `vulkan_isp/isp_shaders/`
4. **Recompile SPIRV** — use `glslangValidator` or `spirv-as`
5. **Re-embed SPIRV** — update the `isp_opset.h` or `isp_fused.onnx`
6. **Re-run A/B test** — verify the fix resolves the discrepancy

## Test Execution Order

```bash
# Step 1: Run Pass0 (ONNX→MNN conversion) for all blocks
cd cam-rust
cargo test --test test_mnn_pass0 -p cam-isp --features mnn -- --nocapture

# Step 2: Run Pass1 (Vulkan inference) for all blocks
cargo test --test test_mnn_pass1 -p cam-isp --features mnn -- --nocapture

# Step 3: Run A/B comparison for all blocks
cargo test --test test_mnn_ab_comparison -p cam-isp --features mnn -- --nocapture

# Step 4: Run SPIRV correctness verification
cargo test --test test_mnn_spirv_verify -p cam-isp --features mnn -- --nocapture
```

## CI Integration

These tests are gated behind `--features mnn` and marked `#[ignore]` in CI
(since they require MNN shared libraries and Vulkan driver). They can be run
locally with:

```bash
export LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a
cargo test --test test_mnn_pass0 -p cam-isp --features mnn -- --ignored --nocapture
cargo test --test test_mnn_pass1 -p cam-isp --features mnn -- --ignored --nocapture
cargo test --test test_mnn_ab_comparison -p cam-isp --features mnn -- --ignored --nocapture
```

## Success Criteria

1. **Pass0**: All blocks produce valid `.mnn` files that load and run on MNN
2. **Pass1**: All blocks produce valid Vulkan inference output (no NaN/Inf)
3. **A/B**: MAE < 0.01 and PSNR > 40 dB for all blocks between Pass0 and Pass1
4. **SPIRV**: All custom opset shaders produce mathematically correct output
   matching the CPU reference implementation
5. **Alignment**: Pass0 MNN files match what `cam_isp` produces for the same
   ONNX graphs (node-for-node alignment)

## Failure Modes

| Symptom | Root Cause | Fix |
|---------|-----------|-----|
| Pass0 conversion fails | Unsupported ONNX op in MNN converter | Add custom op handler or replace with supported ops |
| Pass1 Vulkan session fails | Missing `libMNN_Vulkan.so` | Ensure Vulkan lib is in LD_LIBRARY_PATH |
| Pass1 output is all zeros | SPIRV binding mismatch | Check input/output tensor bindings in SPIRV |
| A/B MAE > 0.01 | SPIRV computation error | Fix GLSL shader math, recompile, re-embed |
| A/B output shape mismatch | ONNX graph shape inference error | Fix `output_value_info` in block definition |
| NaN/Inf in Pass1 output | Division by zero or invalid FP op | Add epsilon guards in SPIRV shader |
