# ISP Pipeline Metadata & Parameter Reference Analysis

## Overview

This document provides a comprehensive analysis of the SoftISP pipeline's metadata handling, parameter references, and control flow. It covers:

- **Input Metadata**: What metadata the pipeline accepts (CCM, 3A parameters, etc.)
- **Output Metadata**: What metadata the pipeline produces (statistics, auxiliary outputs, etc.)
- **Parameter Reference System**: How parameters flow through the pipeline
- **Pipeline Architecture**: Block-based processing with metadata flow
- **Neural Controller Integration**: How the ISP Rectifier model handles parameters
- **Profile-Based Processing**: How different pipeline profiles affect metadata handling

---

## Table of Contents

1. [Pipeline Architecture](#pipeline-architecture)
2. [Parameter Reference System](#parameter-reference-system)
   - [Core Parameter Structures](#core-parameter-structures)
   - [Parameter Flow Architecture](#parameter-flow-architecture)
   - [Controller API](#controller-api)
3. [Input Metadata](#input-metadata)
   - [Core Image Parameters](#core-image-parameters)
   - [3A Parameters](#3a-parameters)
   - [CCM (Color Correction Matrix)](#ccm-color-correction-matrix)
   - [Lens Correction Parameters](#lens-correction-parameters)
   - [Display Parameters](#display-parameters)
   - [Scene and Context Metadata](#scene-and-context-metadata)
4. [Neural Controller Parameter Reference](#neural-controller-parameter-reference)
   - [Model Input Specification](#model-input-specification)
   - [Model Output Specification](#model-output-specification)
   - [Parameter Conversion](#parameter-conversion)
   - [Register Mapping](#register-mapping)
5. [Output Metadata](#output-metadata)
   - [Auxiliary Outputs (IspAuxOutput)](#auxiliary-outputs-ispauxoutput)
   - [Frame Statistics](#frame-statistics)
   - [Block-Specific Outputs](#block-specific-outputs)
6. [Pipeline Profiles and Metadata Flow](#pipeline-profiles-and-metadata-flow)
   - [Profile Comparison](#profile-comparison)
   - [Metadata Flow by Profile](#metadata-flow-by-profile)
7. [Parameter Reference Tables](#parameter-reference-tables)
8. [API Reference](#api-reference)
9. [Usage Examples](#usage-examples)
10. [Summary Tables](#summary-tables)

---

## Pipeline Architecture

The SoftISP pipeline is a modular, block-based image signal processing system that follows this general structure:

```
Raw Input → Preprocessing → Processing → Postprocessing → Output
                     ↓
                 Statistics Extraction
                     ↓
                 Metadata Generation
                     ↓
                 Parameter Update (Controller)
```

### Block Chain Structure

The pipeline consists of a series of `IspBlock` implementations that form a linked chain. Each block:
- Receives input tensors from the previous block
- Receives parameters from the controller
- Performs its processing operation
- Outputs tensors to the next block
- Optionally produces auxiliary metadata

### Parameter Flow Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     PARAMETER REFERENCE SYSTEM                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────┐ │
│  │   ProcessParams │────▶│   IspEngine     │────▶│   IspFrame  │ │
│  │   (Input)       │     │   (Processing)  │     │   (Output)  │ │
│  └─────────────────┘     └─────────────────┘     └─────────────┘ │
│           │                        │                        │       │
│           │                        ▼                        │       │
│           │              ┌─────────────────┐                  │       │
│           │              │   Controller    │◀─────────────────┘       │
│           │              │   (Rule/Neural)  │                          │
│           │              └─────────────────┘                          │
│           │                        │                                    │
│           ▼                        ▼                                    │
│  ┌─────────────────┐     ┌─────────────────┐                        │
│  │   IspParams     │     │   FrameStats     │                        │
│  │   (Parameters)  │     │   (Statistics)   │                        │
│  └─────────────────┘     └─────────────────┘                        │
│           │                        │                                    │
│           ▼                        ▼                                    │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │                    Block Parameterization                    │  │
│  │  Each block receives:                                        │  │
│  │  - Input tensors from previous block                         │  │
│  │  - Parameters from IspParams (WB, CCM, Tone, etc.)           │  │
│  │  - Context from ProcessParams (resolution, format, etc.)     │  │
│  └─────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Parameter Reference System

### Core Parameter Structures

#### IspParams (Main Parameter Container)

The `IspParams` struct is the primary container for all ISP processing parameters. It contains:

```rust
pub struct IspParams {
    pub blc: BlcParams,          // Black level correction
    pub wb: WbParams,            // White balance gains
    pub ccm: CcmParams,          // Color correction matrix
    pub tone: ToneParams,        // Tone mapping curve
    pub saturation: SaturationParams,  // Saturation control
    pub sharpen: SharpenParams,  // Sharpening parameters
    pub denoise: DenoiseParams,  // Denoising parameters
    pub lens: LensParams,        // Lens correction parameters
    pub display: DisplayParams,  // Display/output settings
    pub custom: HashMap<String, f32>,  // Custom parameters
}
```

**Key Features:**
- All parameters are **deterministic** - blocks are pure functions without internal state
- Parameters flow from controller → engine → blocks
- Supports **interpolation** (`lerp`) between parameter sets for smooth transitions
- Can create **identity parameters** (no processing effect)

#### ProcessParams (Processing Context)

The `ProcessParams` struct provides the complete context for processing a single frame:

```rust
pub struct ProcessParams<'a> {
    // Frame dimensions
    pub width: u32,
    pub height: u32,
    pub stride_width: u32,
    
    // Input data
    pub buf: &'a [u8],           // Raw pixel buffer
    pub sensor_max: f32,         // Sensor maximum value
    
    // Output dimensions
    pub target_width: u32,
    pub target_height: u32,
    
    // Parameter overrides (optional)
    pub ccm_matrix: Option<[f32; 9]>,    // Direct CCM override
    pub tone_params: ToneParams,          // Tone parameters
    pub bayer_gains: Option<[f32; 4]>,
    pub awb_gains: Option<[f32; 3]>,
    pub bayer_pattern: i32,
    pub analog_gain: f32,
    pub scene_change: f32,
    pub lsc_gains: Option<&'a [f32]>,
    pub blc_values: Option<[f32; 4]>,
    pub warp_grid: Option<&'a [f32]>,
    
    // Output settings
    pub output_format: OutputFormat,
    pub timestamp_ns: u64,
    
    // Full parameter set (from controller)
    pub isp_params: Option<IspParams>,
}
```

**Parameter Priority:**
1. Direct overrides (e.g., `ccm_matrix`, `awb_gains`) - highest priority
2. `isp_params` from controller - medium priority
3. Default values - lowest priority

### Parameter Flow Architecture

The parameter flow follows this sequence:

1. **Controller Analysis**: `IspController` or `NeuralController` analyzes the previous frame
2. **Parameter Generation**: Controller produces `IspParams` with optimal settings
3. **Engine Processing**: `IspEngine` receives `ProcessParams` and applies parameters to blocks
4. **Block Execution**: Each block uses parameters from `IspParams` to process its input
5. **Metadata Extraction**: Blocks produce auxiliary outputs and statistics
6. **Feedback Loop**: Output metadata is fed back to controller for next frame

```
Controller —(IspParams)—▶ Engine —(ProcessParams)—▶ Pipeline
         ▲                                                  ↓
         └———(FrameStats, IspAuxOutput)———————┘
```

### Controller API

The `ControllerApi` trait provides a unified interface for parameter control:

```rust
pub trait ControllerApi {
    /// Main entry point: analyze frame and produce parameters
    fn analyze_and_update(&mut self, frame: &IspFrame) -> IspParams;
    
    /// Check if neural model is loaded
    fn has_model(&self) -> bool;
    
    /// Load neural model
    fn load_model(&mut self, model_path: &str) -> bool;
    
    /// Get last computed parameters
    fn last_params(&self) -> Option<&IspParams>;
}
```

**Implementations:**
- `IspController`: Rule-based controller (always available)
- `NeuralController`: Neural network controller with fallback
- `Controller`: Enum wrapper for both types

---

## Input Metadata

### Core Image Parameters

#### Raw Input Metadata
- **Width/Height**: Sensor resolution (pixels)
- **Bayer Pattern**: Sensor color filter array pattern (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR)
- **Bit Depth**: Raw pixel bit depth (typically 10-16 bits)
- **Sensor Max**: Maximum sensor value (e.g., 1023.0 for 10-bit)
- **Data Format**: Input data format (RawSensor, Raw10, Raw12, etc.)
- **Timestamp**: Capture timestamp (nanoseconds since epoch)
- **Stride Width**: Stride for packed formats

#### Pixel Data
- **Raw Bayer Data**: Raw sensor data in Bayer pattern
- **Packed INT32**: Zero-copy packed format (2 pixels per INT32)
- **Float Data**: Normalized float data (legacy path)

### 3A Parameters

The pipeline supports comprehensive 3A (Auto-Exposure, Auto-White-Balance, Auto-Focus) metadata:

#### Auto-Exposure (AE) Parameters

From `IspParams.tone` and `ProcessParams`:

```rust
pub struct ToneParams {
    pub contrast: f32,        // Contrast (1.0 = neutral)
    pub brightness: f32,      // Brightness offset
    pub gamma: f32,           // Gamma (1.0 = linear)
    pub black_crush: f32,     // Black crush (0.0-1.0)
    pub white_clip: f32,      // White clip (0.0-1.0)
}
```

Additional AE metadata in `ProcessParams`:
- `analog_gain`: Sensor analog gain
- `scene_change`: Scene change detection flag

#### Auto-White-Balance (AWB) Parameters

From `IspParams.wb`:

```rust
pub struct WbParams {
    pub r: f32,    // Red gain
    pub g: f32,    // Green gain
    pub b: f32,    // Blue gain
}
```

Additional AWB metadata in `ProcessParams`:
- `awb_gains`: Optional direct WB gains override `[f32; 3]`
- `bayer_gains`: Bayer-specific gains `[f32; 4]` (R, Gr, Gb, B)

#### Auto-Focus (AF) Parameters

From `ProcessParams`:
- `scene_change`: Scene change flag (triggers AF reset)

From auxiliary outputs:
- `focus_metric`: Sharpness metric (higher = sharper)
- `calibration_stats`: Quad-level Bayer analysis `[f32; 24]` for AF
- `vcm_position`: Voice coil motor lens position (i32)
- `af_phase`: AF phase display string

### CCM (Color Correction Matrix)

The pipeline accepts CCM metadata in multiple forms:

#### CCM Matrix Structure
- **Format**: 3×3 matrix (row-major order)
- **Type**: `[f32; 9]`
- **Row Sum**: Each row typically sums to 1.0 (neutral preservation)
- **Constraints**: Off-diagonals are negative (no purple cross-contamination)

#### CCM Input Sources

1. **Static CCM**: Direct matrix input via `CcmParams`
   ```rust
   pub struct CcmParams {
       pub matrix: [f32; 9],
   }
   ```

2. **CCT-Based CCM**: Computed from Correlated Color Temperature
   ```rust
   // From ccm_engine.rs
   pub fn quadratic_ccm(cct: i32) -> [f32; 9]
   pub fn select_ccm(cct: i32) -> [f32; 9]
   ```

3. **Identity CCM**: `[1,0,0, 0,1,0, 0,0,1]` (no color correction)

4. **Default Sensor CCM**: Predefined matrix for typical mobile sensors
   ```rust
   pub fn default_sensor_ccm() -> [f32; 9] {
       [1.40, -0.30, -0.10,
        -0.25, 1.45, -0.20,
        -0.05, -0.30, 1.35]
   }
   ```

5. **Sanitized CCM**: Validated and clamped via `CcmEngine::sanitize_ccm()`

#### CCM Invariants

The CCM engine enforces these invariants:
- Each row sums to 1.0 (neutral preservation)
- R-B < 0 and B-R < 0 (anti-purple)
- All off-diagonals ∈ [-1.0, 0] (no positive cross-talk)
- Main diagonal ∈ [0.5, 3.0] (reasonable gain range)
- AWB feasibility: sensor gray produces valid gains

### Lens Correction Parameters

From `IspParams.lens`:

```rust
pub struct LensParams {
    pub vignetting_strength: f32,      // Vignetting correction strength
    pub vignetting_center_x: f32,      // Vignetting center X (0.0-1.0)
    pub vignetting_center_y: f32,      // Vignetting center Y (0.0-1.0)
    pub distortion_k1: f32,            // Radial distortion coefficient 1
    pub distortion_k2: f32,            // Radial distortion coefficient 2
    pub chromatic_aberration: f32,     // Chromatic aberration correction
}
```

From `ProcessParams`:
- `lsc_gains`: Lens shading correction gains (optional slice)

### Display Parameters

From `IspParams.display`:

```rust
pub struct DisplayParams {
    pub format: OutputFormat,    // Output pixel format
    pub width: u32,             // Output width
    pub height: u32,            // Output height
}
```

From `ProcessParams`:
- `output_format`: Output format (can override display.format)
- `target_width/height`: Target output dimensions

### Scene and Context Metadata

From `FrameStats` (used by controller):

```rust
pub struct FrameStats {
    pub avg_luminance: f32,      // Average luminance (0.0-1.0)
    pub histogram: [u32; 256],   // Luminance histogram (256 bins)
    pub color_temp: f32,         // Color temperature estimate (K)
    pub noise_level: f32,        // Noise level estimate (0.0-0.3)
    pub sharpness: f32,          // Sharpness score (0.0-1.0)
    pub faces: Vec<FaceInfo>,    // Face detection results
    pub scene: SceneType,        // Scene classification
}
```

From `ProcessParams`:
- `scene_change`: Scene change detection flag
- `analog_gain`: Sensor gain

---

## Neural Controller Parameter Reference

### Model Input Specification

The ISP Rectifier neural controller (`isp_rectifier`) accepts input as a **267-dimensional feature vector**:

#### Input Layout (267 dimensions)

| Component | Dimensions | Type | Range | Description |
|-----------|------------|------|-------|-------------|
| **Histogram** | 256 | u32 → f32 | [0, ∞) | 256-bin luminance histogram |
| **CCT** | 1 | f32 | [2000, 12000] K | Correlated Color Temperature |
| **WB Gains** | 3 | f32 | [0.2, 5.0] | White balance gains [R, G, B] |
| **Exposure Time** | 1 | f32 | [0.001, 0.1] s | Camera exposure time |
| **ISO Gain** | 1 | f32 | [50, 12800] | ISO gain value |
| **Focus Position** | 1 | f32 | [0.0, 1.0] | AF actuator position |
| **Sharpness** | 1 | f32 | [0.0, 1.0] | Focus sharpness metric |
| **Brightness** | 1 | f32 | [0.0, 1.0] | Luma mean |
| **Contrast** | 1 | f32 | [0.0, 1.0] | Luma std dev |
| **Noise Level** | 1 | f32 | [0.0, 0.3] | Noise estimate |

**Total: 256 + 1 + 3 + 1 + 1 + 1 + 1 + 1 + 1 + 1 = 267 dimensions**

#### Input Normalization

| Field | Normalization | Formula |
|-------|---------------|---------|
| Histogram | Sum normalization | `hist[i] = hist[i] / sum * 10000.0` |
| CCT | Scale | `cct / 10000.0` → [0.2, 1.2] |
| WB Gains | Identity | Raw values [0.2, 5.0] |
| Exposure Time | Clamp & Scale | Clamped to [0.001, 0.1] |
| ISO Gain | Log scale | `log2(iso) / 14` → [0.0, 1.0] |
| Focus Position | Identity | Raw value [0.0, 1.0] |
| Sharpness | Identity | Raw value [0.0, 1.0] |
| Brightness | Identity | Raw value [0.0, 1.0] |
| Contrast | Identity | Raw value [0.0, 1.0] |
| Noise Level | Scale | `noise / 0.3` → [0.0, 1.0] |

#### FrameMetadata Structure

```rust
// From isp-rectifier/src/types.rs
pub struct FrameMetadata {
    pub histogram: Vec<u32>,      // 256-bin luminance histogram
    pub cct: f32,                 // Correlated Color Temperature (K)
    pub wb_gains: [f32; 3],       // Current WB gains [R, G, B]
    pub ae: AutoExposure,         // Auto-exposure metadata
    pub af: AutoFocus,            // Auto-focus metadata
    pub awb: AutoWhiteBalance,    // Auto-white-balance metadata
    pub brightness: f32,          // Frame brightness [0.0, 1.0]
    pub contrast: f32,            // Frame contrast [0.0, 1.0]
    pub noise_level: f32,         // Noise level [0.0, 0.3]
    pub timestamp: u64,           // Frame timestamp
}

pub struct AutoExposure {
    pub exposure_time: f32,
    pub iso_gain: f32,
    pub target_brightness: f32,
}

pub struct AutoFocus {
    pub position: f32,
    pub sharpness: f32,
}

pub struct AutoWhiteBalance {
    pub gains: [f32; 3],
    pub confidence: f32,
}
```

### Model Output Specification

The neural controller produces a **20-dimensional output vector**:

#### Output Layout (20 dimensions)

| Component | Dimensions | Type | Range | Activation | Target Block |
|-----------|------------|------|-------|------------|--------------|
| **WB Gains** | 3 | f32 | [0.2, 5.0] | `exp(TanhClamp)` | BayerWbBlock |
| **CCM Matrix** | 9 | f32 | [-3.0, 3.0] | `TanhClamp` | CcmBlock |
| **Tone Curve** | 7 | f32 | [0.0, 1.0] | `Sigmoid` | ToneBlock |
| **Zoom Factor** | 1 | f32 | [1.0, 4.0] | `ReLU+1` | ScaleBlocks |

**Total: 3 + 9 + 7 + 1 = 20 dimensions**

#### ISPOptimizedParams Structure

```rust
pub struct ISPOptimizedParams {
    // White balance
    pub wb_r_gain: f32,    // Red gain [0.2, 5.0]
    pub wb_g_gain: f32,    // Green gain [0.2, 5.0]
    pub wb_b_gain: f32,    // Blue gain [0.2, 5.0]
    
    // Color correction matrix (3x3, row-major)
    pub ccm: [[f32; 3]; 3],
    
    // Tone mapping curve (7 control points, monotonic)
    pub tone_curve_lut: Vec<f32>,
    
    // Digital zoom
    pub zoom_factor: f32,   // [1.0, 4.0]
}
```

### Parameter Conversion

#### From Neural Output to IspParams

```rust
// From neural_controller.rs
fn params_from_optimized(&self, optimized: &ISPOptimizedParams) -> IspParams {
    IspParams {
        blc: BlcParams::default(),
        wb: WbParams {
            r: optimized.wb_r_gain,
            g: optimized.wb_g_gain,
            b: optimized.wb_b_gain,
        },
        ccm: CcmParams {
            matrix: {
                let mut m = [0.0f32; 9];
                for i in 0..3 {
                    for j in 0..3 {
                        m[i * 3 + j] = optimized.ccm[i][j];
                    }
                }
                m
            },
        },
        tone: ToneParams {
            contrast: 1.0,
            brightness: 0.0,
            gamma: 1.0,
            curve_lut: optimized.tone_curve_lut.clone(),
            ..Default::default()
        },
        saturation: SaturationParams::default(),
        sharpen: SharpenParams::default(),
        denoise: DenoiseParams::default(),
        lens: LensParams::default(),
        display: DisplayParams::default(),
        custom: HashMap::new(),
    }
}
```

#### From IspParams to Hardware Registers

```rust
// From isp-rectifier/src/types.rs
pub struct ISPRegisters {
    // White Balance (Q4.12 fixed-point)
    pub wb_r_gain: u16,
    pub wb_g_gain: u16,
    pub wb_b_gain: u16,
    
    // CCM (Q4.12 fixed-point, signed)
    pub ccm_00: i16, pub ccm_01: i16, pub ccm_02: i16,
    pub ccm_10: i16, pub ccm_11: i16, pub ccm_12: i16,
    pub ccm_20: i16, pub ccm_21: i16, pub ccm_22: i16,
    
    // Tone curve (Q0.16 fixed-point)
    pub tone_lut: [u16; 7],
    
    // Zoom (Q4.12 fixed-point)
    pub zoom_scale: u16,
    pub crop_x: u16, pub crop_y: u16,
    pub crop_w: u16, pub crop_h: u16,
}

impl ISPRegisters {
    pub fn from_params(params: &ISPOptimizedParams) -> Self {
        let mut regs = Self::default();
        
        // WB: float → Q4.12 (16-bit fixed point, 4 integer bits)
        regs.wb_r_gain = (params.wb_r_gain * 4096.0) as u16;
        regs.wb_g_gain = (params.wb_g_gain * 4096.0) as u16;
        regs.wb_b_gain = (params.wb_b_gain * 4096.0) as u16;
        
        // CCM: float → Q4.12 (signed)
        regs.ccm_00 = (params.ccm[0][0] * 4096.0) as i16;
        regs.ccm_01 = (params.ccm[0][1] * 4096.0) as i16;
        // ... (all 9 elements)
        
        // Tone curve: float → Q0.16 (16-bit unsigned, 0 integer bits)
        for (i, &val) in params.tone_curve_lut.iter().enumerate() {
            if i < 7 {
                regs.tone_lut[i] = (val.clamp(0.0, 1.0) * 65535.0) as u16;
            }
        }
        
        // Zoom: float → Q4.12
        regs.zoom_scale = (params.zoom_factor * 4096.0) as u16;
        
        regs
    }
}
```

### Register Mapping

#### Fixed-Point Formats

| Parameter | Format | Range | Precision |
|-----------|--------|-------|-----------|
| WB Gains | Q4.12 | [-8.0, 7.999] | 1/4096 ≈ 0.000244 |
| CCM | Q4.12 (signed) | [-8.0, 7.999] | 1/4096 ≈ 0.000244 |
| Tone LUT | Q0.16 | [0.0, 0.99998] | 1/65535 ≈ 0.000015 |
| Zoom | Q4.12 | [0.0, 7.999] | 1/4096 ≈ 0.000244 |

#### Register Addresses

The ISP hardware register map includes:
- **WB Registers**: Addresses for R, G, B gains
- **CCM Registers**: 9 registers for the 3×3 matrix (row-major)
- **Tone LUT Registers**: 7 registers for tone curve control points
- **Zoom/Crop Registers**: Scale and crop parameters

---

## Output Metadata

### Auxiliary Outputs (IspAuxOutput)

The primary output metadata structure is `IspAuxOutput`, which contains statistics and metadata extracted during pipeline processing:

```rust
#[derive(Debug, Clone, Default)]
pub struct IspAuxOutput {
    // AWB Statistics
    pub channel_means: Option<[f32; 3]>,      // RGB channel means
    pub tone_stats: Option<[f32; 3]>,        // Tone curve statistics
    pub wb_gains: Option<[f32; 3]>,          // Computed white balance gains
    
    // AE Statistics  
    pub histogram: Option<Vec<f32>>,          // RGB histogram (256 bins per channel)
    pub zone_stats: Option<Vec<f32>>,         // Zone statistics for AE metering
    
    // AF Statistics
    pub focus_metric: Option<f32>,           // Focus metric (higher = sharper)
    pub calibration_stats: Option<[f32; 24]>, // Quad-level Bayer analysis
    
    // Color Temperature
    pub cct: Option<f32>,                    // Correlated color temperature (Kelvin)
    
    // Exposure
    pub ae_gain: Option<f32>,                // Auto-exposure gain
    
    // Scene Classification
    pub scene_category: Option<String>,       // Scene classification
    pub af_phase: Option<String>,             // AF phase display
    
    // Motion
    pub vcm_position: Option<i32>,           // Current VCM lens position
    pub eis_compensation: Option<[f32; 3]>,   // EIS compensation [dx_px, dy_px, roll_deg]
}
```

### Frame Statistics

Each processed frame (`IspFrame`) includes:

```rust
pub struct IspFrame {
    pub data: Vec<u8>,                     // Processed pixel data
    pub float_data: Option<Vec<f32>>,      // Float data (if applicable)
    pub width: u32,                        // Output width
    pub height: u32,                       // Output height
    pub format: FrameFormat,               // Pixel format
    pub aux: Option<IspAuxOutput>,          // Auxiliary outputs
    
    // Performance Timing
    pub timestamp_ns: u64,                 // Capture timestamp
    pub prep_duration_ns: u64,             // Frame preparation duration
    pub inference_duration_ns: u64,        // Inference/processing duration
    pub total_duration_ns: u64,            // Total pipeline duration
}
```

### Block-Specific Outputs

Each pipeline block can produce additional metadata:

#### Statistics Blocks
- **ZoneStatsBlock**: Per-zone RGB means for multi-illuminant AWB
- **ChannelMeansBlock**: Global RGB channel means for grey-world AWB
- **ToneStatsBlock**: Luma mean, min, max, clipped/shadows for AE
- **CoarseHistogramBlock**: 16-bin luminance histogram
- **CalibrationBlock**: Quad-level means/vars/mins/maxs for AF

#### Processing Blocks
- **BayerWbBlock**: White balance gains applied
- **DemosaicBlock**: Demosaicing quality metrics
- **CcmBlock**: Color correction matrix application results
- **ToneBlock**: Tone curve application statistics
- **EeBlock**: Edge enhancement metrics
- **FcsBlock**: False color suppression statistics
- **LdciBlock**: Local dynamic contrast improvement metrics
- **BilateralBlock**: Denoising statistics

---

## Pipeline Profiles and Metadata Flow

### Profile Comparison

| Profile | Level | Main Blocks | Aux Blocks | Stats | CCM | AWB | AE | AF | Lens | Features |
|---------|-------|-------------|------------|-------|-----|-----|----|----|------|----------|
| LITE | 0 | 16 | 3 | Basic | Basic | Basic | Basic | Basic | None | Minimal |
| MED | 1 | 16 | 4 | Full | Full | Full | Full | Full | None | Balanced |
| HEAVY | 2 | 16 | 5 | Full | CCT | Multi | Full | Full | Full | Complete |
| PRO | 3 | 16+ | 5+ | Full | Full | Full | Full | Full | Full | All |
| UNIFIED | 3 | 16+ | 5+ | Full | Full | Full | Full | Full | Full | Complete |
| TEST | 0 | 16 | 2 | Basic | Identity | Basic | None | Basic | None | Testing |

### Metadata Flow by Profile

#### LITE Profile
- **Input**: Basic CCM, simple WB, basic tone
- **Stats**: Channel means, zone stats, calibration stats
- **Blocks**: 16 main + 3 aux = 19 total
- **Use Case**: Real-time preview, minimal latency

#### MED Profile  
- **Input**: Full CCM, advanced WB, tone curve
- **Stats**: Channel means, zone stats, tone stats, calibration stats
- **Blocks**: 16 main + 4 aux = 20 total
- **Use Case**: Standard photography, balanced quality/performance

#### HEAVY Profile
- **Input**: CCT-based CCM, multi-illuminant AWB, edge demosaicing
- **Stats**: All statistics at full resolution
- **Features**: FCS, LDCI, EE, bilateral filter, vignetting, saturation
- **Blocks**: 16 main + 5 aux = 21 total
- **Use Case**: High-quality photography
- **Optimization**: Stats downscale to 540p for performance

#### PRO Profile
- **Input**: Everything enabled
- **Features**: Warp (EIS/deshake), HDR merge, all lens corrections
- **Stats**: All statistics at full resolution
- **Blocks**: 16+ main + 5+ aux
- **Use Case**: Professional photography, maximum quality

#### UNIFIED Profile
- **Input**: Complete ISP pipeline
- **Features**: All available blocks and corrections
- **Stats**: All available metadata
- **Use Case**: Complete ISP solution

#### TEST Profile
- **Input**: Minimal blocks for testing
- **Stats**: Basic zone stats, calibration stats
- **Blocks**: 16 main + 2 aux = 18 total
- **Use Case**: Unit testing, debugging

---

## Parameter Reference Tables

### Input Parameter Reference

| Parameter | Source | Type | Range | Default | Description |
|-----------|--------|------|-------|---------|-------------|
| width | ProcessParams | u32 | [1, ∞) | Required | Sensor width |
| height | ProcessParams | u32 | [1, ∞) | Required | Sensor height |
| bayer_pattern | ProcessParams | i32 | 0-3 | 0 | CFA pattern |
| sensor_max | ProcessParams | f32 | [255, 65535] | 1023.0 | Max sensor value |
| buf | ProcessParams | &[u8] | - | Required | Raw pixel data |
| timestamp_ns | ProcessParams | u64 | - | 0 | Capture timestamp |
| target_width | ProcessParams | u32 | [1, ∞) | width | Target width |
| target_height | ProcessParams | u32 | [1, ∞) | height | Target height |
| output_format | ProcessParams | OutputFormat | - | Bgra | Output format |
| ccm_matrix | ProcessParams | Option<[f32;9]> | - | None | CCM override |
| tone_params | ProcessParams | ToneParams | - | default() | Tone parameters |
| awb_gains | ProcessParams | Option<[f32;3]> | - | None | AWB gains override |
| bayer_gains | ProcessParams | Option<[f32;4]> | - | None | Bayer gains |
| analog_gain | ProcessParams | f32 | [0.0, ∞) | 1.0 | Analog gain |
| scene_change | ProcessParams | f32 | [0.0, 1.0] | 0.0 | Scene change flag |
| lsc_gains | ProcessParams | Option<&[f32]> | - | None | Lens shading gains |
| blc_values | ProcessParams | Option<[f32;4]> | - | None | Black level correction |
| warp_grid | ProcessParams | Option<&[f32]> | - | None | Warp grid |
| isp_params | ProcessParams | Option<IspParams> | - | None | Full parameters |

### IspParams Structure Reference

| Field | Type | Range | Default | Description |
|-------|------|-------|---------|-------------|
| blc.r | f32 | [-∞, ∞) | 64.0 | Black level R |
| blc.gr | f32 | [-∞, ∞) | 64.0 | Black level Gr |
| blc.gb | f32 | [-∞, ∞) | 64.0 | Black level Gb |
| blc.b | f32 | [-∞, ∞) | 64.0 | Black level B |
| wb.r | f32 | [0.0, ∞) | 1.0 | White balance R gain |
| wb.g | f32 | [0.0, ∞) | 1.0 | White balance G gain |
| wb.b | f32 | [0.0, ∞) | 1.0 | White balance B gain |
| ccm.matrix | [f32;9] | [-∞, ∞) | identity | Color correction matrix |
| tone.contrast | f32 | [-∞, ∞) | 1.0 | Contrast |
| tone.brightness | f32 | [-∞, ∞) | 0.0 | Brightness offset |
| tone.gamma | f32 | [0.0, ∞) | 1.0 | Gamma |
| tone.black_crush | f32 | [0.0, 1.0] | 0.0 | Black crush |
| tone.white_clip | f32 | [0.0, 1.0] | 1.0 | White clip |
| saturation.factor | f32 | [0.0, ∞) | 1.0 | Saturation factor |
| saturation.vibrance | f32 | [0.0, ∞) | 0.0 | Vibrance |
| sharpen.amount | f32 | [0.0, ∞) | 0.0 | Sharpening amount |
| sharpen.radius | f32 | [0.0, ∞) | 1.0 | Sharpening radius |
| sharpen.threshold | f32 | [0.0, ∞) | 0.0 | Edge threshold |
| denoise.spatial_strength | f32 | [0.0, ∞) | 0.0 | Spatial denoise |
| denoise.temporal_strength | f32 | [0.0, ∞) | 0.0 | Temporal denoise |
| denoise.edge_preserve | f32 | [0.0, 1.0] | 0.5 | Edge preservation |
| denoise.bilateral_sigma | f32 | [0.0, ∞) | 3.0 | Bilateral sigma |
| lens.vignetting_strength | f32 | [0.0, ∞) | 0.0 | Vignetting correction |
| lens.vignetting_center_x | f32 | [0.0, 1.0] | 0.5 | Vignetting center X |
| lens.vignetting_center_y | f32 | f32 | [0.0, 1.0] | 0.5 | Vignetting center Y |
| lens.distortion_k1 | f32 | [-∞, ∞) | 0.0 | Distortion k1 |
| lens.distortion_k2 | f32 | [-∞, ∞) | 0.0 | Distortion k2 |
| lens.chromatic_aberration | f32 | [0.0, ∞) | 0.0 | CA correction |
| display.format | OutputFormat | - | Rgb | Output format |
| display.width | u32 | [1, ∞) | 1920 | Output width |
| display.height | u32 | [1, ∞) | 1080 | Output height |

### Neural Controller Parameter Reference

#### Input Parameters (FrameMetadata → Model)

| Field | Dimensions | Type | Range | Normalization | Model Input Index |
|-------|------------|------|-------|---------------|------------------|
| histogram | 256 | u32 → f32 | [0, ∞) | sum=10000 | 0-255 |
| cct | 1 | f32 | [2000, 12000] | /10000 | 256 |
| wb_gains[0] | 1 | f32 | [0.2, 5.0] | identity | 257 |
| wb_gains[1] | 1 | f32 | [0.2, 5.0] | identity | 258 |
| wb_gains[2] | 1 | f32 | [0.2, 5.0] | identity | 259 |
| ae.exposure_time | 1 | f32 | [0.001, 0.1] | clamp | 260 |
| ae.iso_gain | 1 | f32 | [50, 12800] | log2/14 | 261 |
| af.position | 1 | f32 | [0.0, 1.0] | identity | 262 |
| af.sharpness | 1 | f32 | [0.0, 1.0] | identity | 263 |
| brightness | 1 | f32 | [0.0, 1.0] | identity | 264 |
| contrast | 1 | f32 | [0.0, 1.0] | identity | 265 |
| noise_level | 1 | f32 | [0.0, 0.3] | /0.3 | 266 |

#### Output Parameters (Model → ISPOptimizedParams)

| Field | Dimensions | Type | Range | Activation | Model Output Index |
|-------|------------|------|-------|------------|-------------------|
| wb_r_gain | 1 | f32 | [0.2, 5.0] | exp(TanhClamp) | 0 |
| wb_g_gain | 1 | f32 | [0.2, 5.0] | exp(TanhClamp) | 1 |
| wb_b_gain | 1 | f32 | [0.2, 5.0] | exp(TanhClamp) | 2 |
| ccm[0][0] | 1 | f32 | [-3.0, 3.0] | TanhClamp | 3 |
| ccm[0][1] | 1 | f32 | [-3.0, 3.0] | TanhClamp | 4 |
| ccm[0][2] | 1 | f32 | [-3.0, 3.0] | TanhClamp | 5 |
| ccm[1][0] | 1 | f32 | [-3.0, 3.0] | TanhClamp | 6 |
| ccm[1][1] | 1 | f32 | [-3.0, 3.0] | TanhClamp | 7 |
| ccm[1][2] | 1 | f32 | [-3.0, -3.0] | TanhClamp | 8 |
| ccm[2][0] | 1 | f32 | [-3.0, 3.0] | TanhClamp | 9 |
| ccm[2][1] | 1 | f32 | [-3.0, 3.0] | TanhClamp | 10 |
| ccm[2][2] | 1 | f32 | [-3.0, 3.0] | TanhClamp | 11 |
| tone_curve_lut[0] | 1 | f32 | [0.0, 1.0] | Sigmoid | 12 |
| tone_curve_lut[1] | 1 | f32 | [0.0, 1.0] | Sigmoid | 13 |
| tone_curve_lut[2] | 1 | f32 | [0.0, 1.0] | Sigmoid | 14 |
| tone_curve_lut[3] | 1 | f32 | [0.0, 1.0] | Sigmoid | 15 |
| tone_curve_lut[4] | 1 | f32 | [0.0, 1.0] | Sigmoid | 16 |
| tone_curve_lut[5] | 1 | f32 | [0.0, 1.0] | Sigmoid | 17 |
| tone_curve_lut[6] | 1 | f32 | [0.0, 1.0] | Sigmoid | 18 |
| zoom_factor | 1 | f32 | [1.0, 4.0] | ReLU+1 | 19 |

#### Register Mapping (ISPOptimizedParams → ISPRegisters)

| Parameter | Format | Register | Conversion |
|-----------|--------|----------|------------|
| wb_r_gain | Q4.12 | wb_r_gain | `value * 4096.0` |
| wb_g_gain | Q4.12 | wb_g_gain | `value * 4096.0` |
| wb_b_gain | Q4.12 | wb_b_gain | `value * 4096.0` |
| ccm[0][0] | Q4.12 (signed) | ccm_00 | `value * 4096.0` |
| ccm[0][1] | Q4.12 (signed) | ccm_01 | `value * 4096.0` |
| ccm[0][2] | Q4.12 (signed) | ccm_02 | `value * 4096.0` |
| ccm[1][0] | Q4.12 (signed) | ccm_10 | `value * 4096.0` |
| ccm[1][1] | Q4.12 (signed) | ccm_11 | `value * 4096.0` |
| ccm[1][2] | Q4.12 (signed) | ccm_12 | `value * 4096.0` |
| ccm[2][0] | Q4.12 (signed) | ccm_20 | `value * 4096.0` |
| ccm[2][1] | Q4.12 (signed) | ccm_21 | `value * 4096.0` |
| ccm[2][2] | Q4.12 (signed) | ccm_22 | `value * 4096.0` |
| tone_curve_lut[0] | Q0.16 | tone_lut[0] | `value * 65535.0` |
| tone_curve_lut[1] | Q0.16 | tone_lut[1] | `value * 65535.0` |
| tone_curve_lut[2] | Q0.16 | tone_lut[2] | `value * 65535.0` |
| tone_curve_lut[3] | Q0.16 | tone_lut[3] | `value * 65535.0` |
| tone_curve_lut[4] | Q0.16 | tone_lut[4] | `value * 65535.0` |
| tone_curve_lut[5] | Q0.16 | tone_lut[5] | `value * 65535.0` |
| tone_curve_lut[6] | Q0.16 | tone_lut[6] | `value * 65535.0` |
| zoom_factor | Q4.12 | zoom_scale | `value * 4096.0` |

---

## API Reference

### ProcessParams

Primary input structure for pipeline processing:

```rust
pub struct ProcessParams<'a> {
    pub width: u32,
    pub height: u32,
    pub stride_width: u32,
    pub buf: &'a [u8],
    pub sensor_max: f32,
    pub target_width: u32,
    pub target_height: u32,
    pub ccm_matrix: Option<[f32; 9]>,
    pub tone_params: ToneParams,
    pub bayer_gains: Option<[f32; 4]>,
    pub awb_gains: Option<[f32; 3]>,
    pub bayer_pattern: i32,
    pub analog_gain: f32,
    pub scene_change: f32,
    pub lsc_gains: Option<&'a [f32]>,
    pub blc_values: Option<[f32; 4]>,
    pub warp_grid: Option<&'a [f32]>,
    pub output_format: OutputFormat,
    pub timestamp_ns: u64,
    pub isp_params: Option<IspParams>,
}
```

### IspFrame

Primary output structure from pipeline processing:

```rust
pub struct IspFrame {
    pub data: Vec<u8>,
    pub float_data: Option<Vec<f32>>,
    pub width: u32,
    pub height: u32,
    pub format: FrameFormat,
    pub aux: Option<IspAuxOutput>,
    pub timestamp_ns: u64,
    pub prep_duration_ns: u64,
    pub inference_duration_ns: u64,
    pub total_duration_ns: u64,
}
```

### IspAuxOutput

See [Auxiliary Outputs](#auxiliary-outputs-ispauxoutput) section.

### ControllerApi

```rust
pub trait ControllerApi {
    fn analyze_and_update(&mut self, frame: &IspFrame) -> IspParams;
    fn has_model(&self) -> bool;
    fn load_model(&mut self, model_path: &str) -> bool;
    fn last_params(&self) -> Option<&IspParams>;
}
```

---

## Usage Examples

### Basic Pipeline with Parameter Control

```rust
use cam_isp::pipeline::{PipelineBuilder, IspFrame};
use cam_isp::engine::{IspEngine, ProcessParams, OutputFormat};
use cam_isp::isp_params::{IspParams, WbParams, CcmParams, ToneParams};

// Build pipeline
let mut pipeline = PipelineBuilder::new(1920, 1080)
    .unpack()
    .demosaic(2)  // GBRG pattern
    .ccm()
    .tone()
    .display()
    .compose()?;

// Create engine
let mut engine = cam_isp::engine::select_engine()?;
engine.build_from_onnx(&pipeline)?;

// Create custom parameters
let mut params = IspParams::default();
params.wb = WbParams { r: 1.8, g: 1.0, b: 1.5 };
params.ccm = CcmParams::new([
    1.40, -0.30, -0.10,
    -0.25, 1.45, -0.20,
    -0.05, -0.30, 1.35,
]);
params.tone = ToneParams {
    contrast: 1.2,
    brightness: 0.05,
    gamma: 2.2,
    ..Default::default()
};

// Prepare input
let raw_data = vec![128u8; 1920 * 1080 * 2];
let mut process_params = ProcessParams::new(1920, 1080, &raw_data);
process_params.bayer_pattern = 2;
process_params.isp_params = Some(params);

// Process frame
let frame: IspFrame = engine.process(&process_params)?;

// Access output metadata
if let Some(aux) = frame.aux {
    println!("WB Gains: {:?}", aux.wb_gains);
    println!("CCT: {:?}K", aux.cct);
    println!("Focus Metric: {:?}", aux.focus_metric);
}
```

### Neural Controller with Parameter Feedback

```rust
use cam_isp::controller_api::{Controller, ControllerApi};
use cam_isp::pipeline::IspFrame;
use cam_isp::isp_params::IspParams;

// Create neural controller (falls back to rule-based if no model)
let mut controller = Controller::neural();

// Load model if available
if !controller.has_model() {
    controller.load_model("models/fusedispcontroller.onnx");
}

// Process frames in a loop
let mut previous_frame: Option<IspFrame> = None;

for raw_frame in camera_capture_iter() {
    // Build process params
    let mut process_params = ProcessParams::from_raw(&raw_frame);
    
    // Get parameters from controller
    if let Some(ref prev) = previous_frame {
        let params = controller.analyze_and_update(prev);
        process_params.isp_params = Some(params);
    }
    
    // Process frame
    let frame = engine.process(&process_params)?;
    
    // Store for next iteration
    previous_frame = Some(frame);
    
    // Access updated parameters
    if let Some(params) = controller.last_params() {
        println!("Current WB: R={:.2}, G={:.2}, B={:.2}", 
                 params.wb.r, params.wb.g, params.wb.b);
        println!("Current CCM[0][0]: {:.4}", params.ccm.matrix[0]);
    }
}
```

### Direct CCM Control with CCT

```rust
use cam_isp::ccm_engine;

// Compute CCM from color temperature
let cct = 5500;  // Kelvin (daylight)
let ccm_matrix = ccm_engine::select_ccm(cct);

// Or use quadratic interpolation
let ccm_quad = ccm_engine::quadratic_ccm(cct);

// Validate and sanitize
let mut ccm_safe = ccm_matrix;
let mut flags = 0i32;
ccm_engine::sanitize_ccm(&mut ccm_safe, cct, "example", Some(&mut flags), None);

if flags != 0 {
    println!("CCM was clamped: flags=0x{:x}", flags);
}

// Use in process params
let mut process_params = ProcessParams::new(1920, 1080, &raw_data);
process_params.ccm_matrix = Some(ccm_safe);
```

### Neural Controller with FrameMetadata

```rust
use isp_rectifier::types::{FrameMetadata, AutoExposure, AutoFocus, AutoWhiteBalance};

// Extract metadata from frame
let metadata = FrameMetadata {
    histogram: compute_histogram(&frame.data, frame.width, frame.height),
    cct: frame.aux.as_ref().and_then(|a| a.cct).unwrap_or(5500.0),
    wb_gains: frame.aux.as_ref().and_then(|a| a.wb_gains).unwrap_or([1.0, 1.0, 1.0]),
    ae: AutoExposure {
        exposure_time: 0.033,  // From camera
        iso_gain: 100.0,       // From camera
        target_brightness: 0.5,
    },
    af: AutoFocus {
        position: 0.5,        // From AF system
        sharpness: frame.aux.as_ref().and_then(|a| a.focus_metric).unwrap_or(0.5),
    },
    awb: AutoWhiteBalance {
        gains: [1.0, 1.0, 1.0],
        confidence: 0.8,
    },
    brightness: frame.aux.as_ref().and_then(|a| a.channel_means)
        .map(|m| (m[0] + m[1] + m[2]) / 3.0)
        .unwrap_or(0.5),
    contrast: 0.5,
    noise_level: frame.aux.as_ref().and_then(|a| a.noise_level).unwrap_or(0.1),
    timestamp: frame.timestamp_ns,
};

// Convert to feature vector
let (hist_features, meta_features) = metadata.to_feature_vector();
assert_eq!(hist_features.len(), 256);
assert_eq!(meta_features.len(), 11);

// Full feature vector
let mut input_vector = hist_features;
input_vector.extend(meta_features);
assert_eq!(input_vector.len(), 267);
```

---

## Summary Tables

### Input Metadata Summary

| Category | Parameter | Type | Range | Source | Required |
|----------|-----------|------|-------|--------|----------|
| **Raw Input** | width | u32 | [1, ∞) | ProcessParams | Yes |
| | height | u32 | [1, ∞) | ProcessParams | Yes |
| | bayer_pattern | i32 | 0-3 | ProcessParams | Yes |
| | raw_data | &[u8] | - | ProcessParams | Yes |
| | timestamp_ns | u64 | - | ProcessParams | Yes |
| **CCM** | matrix | [f32;9] | [-∞, ∞) | CcmParams/ProcessParams | Optional |
| | cct | i32 | [2000, 12000] | Neural input | Optional |
| **AWB** | wb_gains | [f32;3] | [0.2, 5.0] | WbParams/ProcessParams | Optional |
| | channel_means | [f32;3] | [0.0, 1.0] | Computed | No |
| **AE** | ae_gain | f32 | [0.0, ∞) | ProcessParams | Optional |
| | tone_stats | [f32;3] | [-∞, ∞) | Computed | No |
| **AF** | focus_metric | f32 | [0.0, 1.0] | Computed | No |
| | calibration_stats | [f32;24] | [-∞, ∞) | Computed | No |
| **Lens** | vignetting | f32 | [0.0, ∞) | LensParams | Optional |
| | distortion | f32 | [-∞, ∞) | LensParams | Optional |
| **Display** | format | OutputFormat | - | ProcessParams | Yes |

### Output Metadata Summary

| Category | Field | Type | Range | Profile | Description |
|----------|-------|------|-------|---------|-------------|
| **AWB** | channel_means | Option<[f32;3]> | [0.0, 1.0] | All | RGB channel means |
| | wb_gains | Option<[f32;3]> | [0.2, 5.0] | All | Computed WB gains |
| **AE** | tone_stats | Option<[f32;3]> | [-∞, ∞) | MED+ | Tone curve stats |
| | histogram | Option<Vec<f32>> | - | HEAVY+ | RGB histogram |
| | zone_stats | Option<Vec<f32>> | - | All | Per-zone stats |
| **AF** | focus_metric | Option<f32> | [0.0, 1.0] | All | Sharpness metric |
| | calibration_stats | Option<[f32;24]> | - | All | Quad-level stats |
| **Color** | cct | Option<f32> | [2000, 12000] | All | Color temperature |
| **Exposure** | ae_gain | Option<f32> | [0.0, ∞) | All | AE gain |
| **Scene** | scene_category | Option<String> | - | All | Scene classification |
| **Motion** | vcm_position | Option<i32> | - | All | Lens position |
| | eis_compensation | Option<[f32;3]> | - | PRO | EIS compensation |

### Neural Controller Parameter Mapping

| Stage | Input | Output | Conversion |
|-------|-------|--------|------------|
| Frame → Metadata | IspFrame | FrameMetadata | Extract stats |
| Metadata → Vector | FrameMetadata | [f32; 267] | to_feature_vector() |
| Model Input | [f32; 267] | [f32; 20] | Neural inference |
| Vector → Params | [f32; 20] | ISPOptimizedParams | From output |
| Params → IspParams | ISPOptimizedParams | IspParams | params_from_optimized() |
| IspParams → Registers | IspParams | ISPRegisters | from_params() |

### Fixed-Point Register Formats

| Parameter | Format | Range | Precision | Register Type |
|-----------|--------|-------|-----------|---------------|
| WB Gains | Q4.12 | [-8.0, 7.999] | 1/4096 | u16 |
| CCM | Q4.12 | [-8.0, 7.999] | 1/4096 | i16 (signed) |
| Tone LUT | Q0.16 | [0.0, 0.99998] | 1/65535 | u16 |
| Zoom | Q4.12 | [0.0, 7.999] | 1/4096 | u16 |

---

## Conclusion

The SoftISP pipeline provides a comprehensive parameter reference system with:

1. **Flexible Input**: Supports direct parameter overrides, controller-based parameters, and neural network predictions
2. **Rich Metadata**: Produces extensive output metadata for 3A control and diagnostics
3. **Neural Integration**: ISP Rectifier model accepts 267-dim input and produces 20-dim output with optimized parameters
4. **Hardware Mapping**: Parameters can be converted to hardware register values for direct ISP control
5. **Profile-Based**: Different profiles enable trading off quality vs. performance
6. **Extensible**: Custom parameters and blocks can be added as needed

The parameter flow is designed to be:
- **Deterministic**: Blocks are pure functions of their inputs and parameters
- **Feedback-Driven**: Output metadata feeds back to controllers for adaptive processing
- **Hardware-Agnostic**: Works with CPU, GPU, and neural acceleration
- **Production-Ready**: Includes validation, clamping, and safety checks

---

## References

- ISP Rectifier Model Specification: `isp-rectifier/MODEL_SPECIFICATION.md`
- Pipeline Types: `cam-rust/cam-isp/src/pipeline/types.rs`
- ISP Parameters: `cam-rust/cam-isp/src/isp_params.rs`
- Engine: `cam-rust/cam-isp/src/engine.rs`
- Controller API: `cam-rust/cam-isp/src/controller_api.rs`
- ISP Controller: `cam-rust/cam-isp/src/isp_controller.rs`
- Neural Controller: `cam-rust/cam-isp/src/neural_controller.rs`
- CCM Engine: `cam-rust/cam-isp/src/ccm_engine.rs`

---

## Performance Benchmarks (MNN Vulkan on Snapdragon)

Benchmarks run on MNN Vulkan backend (aarch64-linux-android / Termux).

### 4K→FHD ISP Pipeline (7 blocks, raw INT16 Bayer input)

| Metric | Value |
|--------|-------|
| Resolution | 3840×2160 → 960×540 |
| Average latency | 25.2 ms/frame |
| FPS | 39.7 (steady state) |
| First frame (incl. compile) | 43.6 ms |
| MNN inference only | ~27-31 ms |

### Profile × Format × Resolution Comparison

#### LITE Profile

| Format | HD (1280×720) | FHD (1920×1080) | 4K (3840×2160) |
|--------|:-------------:|:---------------:|:--------------:|
| PackedRgb | 11 ms / 90 FPS | 12 ms / 83 FPS | 88 ms / 11 FPS |
| Argb | 11 ms / 90 FPS | 14 ms / 71 FPS | 88 ms / 11 FPS |
| FloatRgb | 18 ms / 55 FPS | 14 ms / 71 FPS | 28 ms / 35 FPS |

#### HEAVY Profile (fused operations)

> The HEAVY profile consistently outperforms LITE on Vulkan because fused operations create larger compute kernels that the GPU parallelizes more efficiently.

| Format | HD (1280×720) | FHD (1920×1080) | 4K (3840×2160) |
|--------|:-------------:|:---------------:|:--------------:|
| PackedRgb | 8 ms / 125 FPS | 9 ms / 111 FPS | 16 ms / 62 FPS |
| Argb | 8 ms / 125 FPS | 10 ms / 100 FPS | 14 ms / 71 FPS |
| FloatRgb | 12 ms / 83 FPS | 17 ms / 58 FPS | 20 ms / 50 FPS |

### Key Observations

1. **HEAVY profile is faster than LITE on Vulkan** — Fused operations benefit GPU parallelism
2. **HD resolutions sustain 90-125 FPS** across both profiles
3. **4K real-time achievable** — HEAVY profile does 4K at 50-71 FPS (well above 30 FPS target)
4. **FloatRgb has lowest overhead** for downstream processing (no format conversion needed)
5. **First frame includes ONNX compilation** — subsequent frames benefit from pipeline caching

### Engine Registration Order (Priority)

| Engine | Priority |
|--------|:--------:|
| mnn_vulkan | 99 (default) |
| mnn_neon | 75 |
| CPU (SIMD) | 70 |
| mnn_cpu | 65 |
| mnn_opencl | 55 |
| mnn_opengl | 50 |

---

### TODO: ONNX Opset Optimization for MNN 2-Pass Converter

The LITE and MEDIUM pipeline profiles use individual (non-fused) ISP blocks:
- `UnpackBlock` + `BlcBlock` (vs fused `UnpackCfaBlock`)
- `CcmBlock` (separate, vs fused in `DemosaicCcmBlock`)
- `EeBlock` + `LdciBlock` separately (vs fused `EeLdciBlock`)

MNN's 2-pass converter (`IspChainFusion`) has limited support for these
individual ops when the block ordering/combination doesn't match expected
patterns. The `--allowCustomOp` flag is required for custom domain ops
(`isp.fcs`, `isp.demosaic_ccm`, `isp.ee_ldci`, `isp.ldci`).

**Action**: Profile the LITE/MEDIUM ONNX opset against MNN's 2-pass
converter and either:
1. Adjust block ordering to match expected fusion patterns
2. Use fused block variants (UnpackCfaBlock, DemosaicCcmBlock, EeLdciBlock)
3. Implement missing IspChainFusion passes in MNN's converter plugin

See: `cam-rust/cam-isp/src/auto_profile.rs` for the AutoProfile block selection.

---

*Generated on: 2026-07-12*
*Version: SoftISP Pipeline Parameter Reference v2.0*
*Source: cam-rust/cam-isp & isp-rectifier codebase*
