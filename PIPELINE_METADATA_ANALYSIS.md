# ISP Pipeline Metadata Analysis

## Overview

This document provides a comprehensive analysis of the SoftISP pipeline's metadata handling, including:
- **Input Metadata**: What metadata the pipeline accepts (CCM, 3A parameters, etc.)
- **Output Metadata**: What metadata the pipeline produces (statistics, auxiliary outputs, etc.)
- **Pipeline Architecture**: How metadata flows through the system
- **Profile-Based Processing**: How different pipeline profiles affect metadata handling

---

## Table of Contents

1. [Pipeline Architecture](#pipeline-architecture)
2. [Input Metadata](#input-metadata)
   - [Core Image Parameters](#core-image-parameters)
   - [3A Parameters](#3a-parameters)
   - [CCM (Color Correction Matrix)](#ccm-color-correction-matrix)
   - [Lens Correction Parameters](#lens-correction-parameters)
   - [Display Parameters](#display-parameters)
   - [Scene and Context Metadata](#scene-and-context-metadata)
3. [Output Metadata](#output-metadata)
   - [Auxiliary Outputs (IspAuxOutput)](#auxiliary-outputs-ispauxoutput)
   - [Frame Statistics](#frame-statistics)
   - [Block-Specific Outputs](#block-specific-outputs)
4. [Pipeline Profiles and Metadata Flow](#pipeline-profiles-and-metadata-flow)
   - [LITE Profile](#lite-profile)
   - [MED Profile](#med-profile)
   - [HEAVY Profile](#heavy-profile)
   - [PRO Profile](#pro-profile)
   - [UNIFIED Profile](#unified-profile)
   - [TEST Profile](#test-profile)
5. [Metadata Flow Diagram](#metadata-flow-diagram)
6. [API Reference](#api-reference)
   - [ProcessParams](#processparams)
   - [IspFrame](#ispframe)
   - [IspAuxOutput](#ispauxoutput)
7. [Usage Examples](#usage-examples)
8. [Summary Tables](#summary-tables)

---

## Pipeline Architecture

The SoftISP pipeline is a modular, block-based image signal processing system that follows this general structure:

```
Raw Input → Preprocessing → Processing → Postprocessing → Output
                     ↓
                 Statistics Extraction
                     ↓
                 Metadata Generation
```

### Block Chain Structure

The pipeline consists of a series of `IspBlock` implementations that form a linked chain. Each block:
- Receives input tensors from the previous block
- Performs its processing operation
- Outputs tensors to the next block
- Optionally produces auxiliary metadata

### Metadata Flow

Metadata flows through the pipeline in two ways:

1. **Input Parameters**: Passed to blocks via `ProcessParams` before processing
2. **Auxiliary Outputs**: Extracted during processing and returned in `IspFrame.aux`

---

## Input Metadata

### Core Image Parameters

The pipeline accepts the following core image metadata via the `ProcessParams` structure:

#### Raw Input Metadata
- **Width/Height**: Sensor resolution (pixels)
- **Bayer Pattern**: Sensor color filter array pattern (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR)
- **Bit Depth**: Raw pixel bit depth (typically 10-16 bits)
- **Data Format**: Input data format (INT32 packed, FLOAT, etc.)
- **Timestamp**: Capture timestamp (nanoseconds)

#### Pixel Data
- **Raw Bayer Data**: Raw sensor data in Bayer pattern
- **Packed INT32**: Zero-copy packed format (2 pixels per INT32)
- **Float Data**: Normalized float data (legacy path)

### 3A Parameters

The pipeline supports comprehensive 3A (Auto-Exposure, Auto-White-Balance, Auto-Focus) metadata:

#### Auto-Exposure (AE) Parameters
From `IspParams.tone` and auxiliary inputs:
- `ae_gain`: Auto-exposure gain multiplier
- `tone_stats`: Luma mean, min, max for AE metering
- `histogram`: Luminance histogram (16 or 256 bins)
- `zone_stats`: Per-zone statistics for metering

#### Auto-White-Balance (AWB) Parameters
From `IspParams.wb` and `IspParams.ccm`:
- `wb_gains`: White balance gains [R, G, B]
- `cct`: Correlated Color Temperature (Kelvin)
- `channel_means`: RGB channel means for grey-world AWB
- `zone_stats`: Per-zone RGB means for multi-illuminant AWB

#### Auto-Focus (AF) Parameters
From auxiliary blocks:
- `focus_metric`: Sharpness metric (higher = sharper)
- `calibration_stats`: Quad-level Bayer analysis [24] for AF
- `vcm_position`: Voice coil motor lens position
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
2. **CCT-Based CCM**: Computed from Correlated Color Temperature using `CcmEngine::quadratic_ccm()`
3. **Identity CCM**: `[1,0,0, 0,1,0, 0,0,1]` (no color correction)
4. **Default Sensor CCM**: Predefined matrix for typical mobile sensors
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
- `vignetting_strength`: Vignetting correction strength
- `vignetting_center_x/y`: Vignetting center coordinates (0.0-1.0)
- `distortion_k1/k2`: Radial distortion coefficients
- `chromatic_aberration`: Chromatic aberration correction strength

### Display Parameters

From `IspParams.display`:
- `format`: Output pixel format (Rgb, Rgba, Argb, Yuv420, Float16)
- `width`: Output width (pixels)
- `height`: Output height (pixels)
- `rotate_mode`: Orientation transform (0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip)

### Scene and Context Metadata

From `FrameStats`:
- `scene`: Scene classification (General, Portrait, Landscape, LowLight, Hdr)
- `faces`: Face detection results with bounding boxes and confidence
- `avg_luminance`: Average scene luminance
- `noise_level`: Estimated noise level
- `sharpness`: Sharpness score

---

## Output Metadata

### Auxiliary Outputs (IspAuxOutput)

The primary output metadata structure is `IspAuxOutput`, which contains statistics and metadata extracted during pipeline processing:

```rust
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

#### Performance Timing
- `timestamp_ns`: Sensor capture timestamp (nanoseconds)
- `prep_duration_ns`: Frame preparation duration
- `inference_duration_ns`: Inference/processing duration
- `total_duration_ns`: Total pipeline duration

#### Output Data
- `data`: Processed pixel data (Vec<u8>)
- `float_data`: Raw float data (for FloatRgb/FloatBgra formats)
- `width/height`: Output dimensions
- `format`: Output pixel format

### Block-Specific Outputs

Each pipeline block can produce additional metadata:

#### Statistics Blocks
- **ZoneStatsBlock**: Per-zone RGB means for multi-illuminant AWB
- **ChannelMeansBlock**: Global RGB channel means for grey-world AWB
- **ToneStatsBlock**: Luma mean, min, max, clipped/shadows for AE
- **CoarseHistogramBlock**: 16-bin luminance histogram
- **CalibrationBlock**: Quad-level means/vars/mins/maxs for AF

#### Processing Blocks
- **BayerWbBlock**: White balance gains
- **DemosaicBlock**: Demosaicing quality metrics
- **CcmBlock**: Color correction matrix application results
- **ToneBlock**: Tone curve application statistics
- **EeBlock**: Edge enhancement metrics
- **FcsBlock**: False color suppression statistics
- **LdciBlock**: Local dynamic contrast improvement metrics
- **BilateralBlock**: Denoising statistics

---

## Pipeline Profiles and Metadata Flow

The pipeline supports multiple profiles that determine which blocks are enabled and what metadata is produced.

### Profile Hierarchy

```
LITE (Minimal) → MED (Medium) → HEAVY (Full) → PRO (Everything) → UNIFIED (Complete)
```

### LITE Profile

**Metadata Input:**
- Basic CCM (identity or default)
- Simple WB gains
- Basic tone parameters
- No advanced 3A

**Metadata Output:**
- Channel means (AWB)
- Zone stats (basic AE)
- Calibration stats (AF)
- No tone stats
- No histogram

**Blocks:** 16 main + 3 aux = 19 total

### MED Profile

**Metadata Input:**
- Full CCM support
- Advanced WB gains
- Tone curve parameters
- Basic 3A statistics

**Metadata Output:**
- Channel means
- Zone stats
- Tone stats
- Calibration stats
- No histogram

**Blocks:** 16 main + 4 aux = 20 total

### HEAVY Profile

**Metadata Input:**
- Full CCM with CCT-based interpolation
- Advanced AWB with multi-illuminant support
- Full tone curve
- Edge-aware demosaicing
- FCS, LDCI, EE

**Metadata Output:**
- All statistics (zone, channel, tone, histogram)
- Full 3A metadata
- Calibration stats
- Stats downscale to 540p for performance

**Blocks:** 16 main + 5 aux = 21 total

### PRO Profile

**Metadata Input:**
- Everything enabled
- Warp (EIS/deshake)
- HDR merge
- All lens corrections

**Metadata Output:**
- All statistics at full resolution
- EIS compensation metadata
- HDR metadata
- Full scene analysis

**Blocks:** 16+ main + 5+ aux

### UNIFIED Profile

**Metadata Input:**
- Complete ISP pipeline
- All 3A features
- All lens corrections
- All cosmetic features

**Metadata Output:**
- All available metadata
- Full statistics
- Complete scene analysis

**Blocks:** 16+ main + 5+ aux

### TEST Profile

**Metadata Input:**
- Minimal blocks for testing
- Identity placeholders for disabled features

**Metadata Output:**
- Basic zone stats
- Calibration stats

**Blocks:** 16 main + 2 aux = 18 total

---

## Metadata Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      INPUT METADATA                               │
├─────────────────────────────────────────────────────────────────┤
│  Raw Input              3A Parameters            CCM              │
│  ├─ Width/Height        ├─ AE Gain               ├─ Matrix [9]    │
│  ├─ Bayer Pattern       ├─ WB Gains [3]          ├─ CCT (K)        │
│  ├─ Bit Depth           ├─ Tone Stats            └─ Identity       │
│  ├─ Timestamp           └─ Focus Metric                          │
│  └─ Pixel Data                                                     │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    PIPELINE PROCESSING                            │
├─────────────────────────────────────────────────────────────────┤
│  Unpack → Norm → CFA → BLC → WB → Demosaic → CCM → Tone → ...   │
│       ↓        ↓        ↓        ↓        ↓        ↓            │
│  Stats Hooks for Auxiliary Blocks                                  │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                     OUTPUT METADATA                               │
├─────────────────────────────────────────────────────────────────┤
│  IspFrame                        IspAuxOutput                     │
│  ├─ Pixel Data (u8)              ├─ Channel Means [3]             │
│  ├─ Float Data (f32)             ├─ Tone Stats [3]                │
│  ├─ Width/Height                 ├─ WB Gains [3]                  │
│  ├─ Format                        ├─ Histogram (Vec<f32>)          │
│  ├─ Timestamps (4)               ├─ Zone Stats (Vec<f32>)         │
│  └─ Duration Metrics             ├─ Focus Metric (f32)            │
│                                  ├─ Calibration Stats [24]        │
│                                  ├─ CCT (f32)                     │
│                                  ├─ AE Gain (f32)                 │
│                                  ├─ Scene Category (String)       │
│                                  └─ EIS Compensation [3]          │
└─────────────────────────────────────────────────────────────────┘
```

---

## API Reference

### ProcessParams

Primary input structure for pipeline processing:

```rust
pub struct ProcessParams {
    pub width: u32,              // Input width
    pub height: u32,             // Input height
    pub bayer_pattern: i32,      // Bayer pattern (0-3)
    pub raw_data: Vec<u8>,       // Raw pixel data
    pub target_width: u32,       // Target output width
    pub target_height: u32,      // Target output height
    pub output_format: OutputFormat,  // Output format
    pub isp_params: IspParams,   // All ISP parameters
    pub timestamp_ns: u64,       // Frame timestamp
    pub frame_id: u64,           // Frame identifier
}
```

### IspFrame

Primary output structure from pipeline processing:

```rust
pub struct IspFrame {
    pub data: Vec<u8>,           // Processed pixel data
    pub float_data: Option<Vec<f32>>,  // Float data (if applicable)
    pub width: u32,              // Output width
    pub height: u32,             // Output height
    pub format: FrameFormat,     // Pixel format
    pub aux: Option<IspAuxOutput>,  // Auxiliary outputs
    pub timestamp_ns: u64,       // Capture timestamp
    pub prep_duration_ns: u64,   // Preparation duration
    pub inference_duration_ns: u64,  // Inference duration
    pub total_duration_ns: u64,  // Total duration
}
```

### IspAuxOutput

See [Auxiliary Outputs](#auxiliary-outputs-ispauxoutput) section.

### IspParams

Complete ISP parameter set:

```rust
pub struct IspParams {
    pub blc: BlcParams,          // Black level correction
    pub wb: WbParams,            // White balance
    pub ccm: CcmParams,          // Color correction matrix
    pub tone: ToneParams,        // Tone mapping
    pub saturation: SaturationParams,  // Saturation
    pub sharpen: SharpenParams,  // Sharpening
    pub denoise: DenoiseParams,  // Denoising
    pub lens: LensParams,        // Lens corrections
    pub display: DisplayParams,  // Display settings
    pub custom: HashMap<String, f32>,  // Custom parameters
}
```

---

## Usage Examples

### Basic Pipeline with Metadata

```rust
use cam_isp::pipeline::{PipelineBuilder, IspFrame, IspAuxOutput};
use cam_isp::engine::{IspEngine, ProcessParams, OutputFormat};

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

// Prepare input with metadata
let raw_data = vec![128u8; 1920 * 1080 * 2];  // 10-bit packed
let mut params = ProcessParams::new(1920, 1080, &raw_data);
params.bayer_pattern = 2;  // GBRG
params.isp_params.wb.r = 1.8;  // Custom WB gains
params.isp_params.wb.b = 1.5;
params.isp_params.ccm = CcmParams::from_matrix([
    1.4, -0.3, -0.1,
    -0.25, 1.45, -0.2,
    -0.05, -0.3, 1.35,
]);

// Process frame
let frame: IspFrame = engine.process(&params)?;

// Access output metadata
if let Some(aux) = frame.aux {
    println!("WB Gains: {:?}", aux.wb_gains);
    println!("CCT: {:?}K", aux.cct);
    println!("AE Gain: {:?}", aux.ae_gain);
    println!("Focus Metric: {:?}", aux.focus_metric);
    
    if let Some(means) = aux.channel_means {
        println!("Channel Means: R={:.2}, G={:.2}, B={:.2}", 
                 means[0], means[1], means[2]);
    }
}

// Access performance metadata
println!("Processing time: {}ms", frame.total_duration_ns as f64 / 1_000_000.0);
```

### Profile-Based Pipeline

```rust
use cam_isp::profile::PipelineProfile;
use cam_isp::pipeline::GraphComposer;

// Select profile based on requirements
let profile = PipelineProfile::HEAVY;

// Build blocks according to profile
let mut blocks = profile.build_blocks(1920, 2);  // 1920 width, GBRG pattern

// Build auxiliary stats blocks
let aux_blocks = profile.build_aux_blocks(1080, 1920);

// Compose ONNX model
GraphComposer::wire_blocks(&mut blocks);
let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
let onnx = GraphComposer::compose_from_vec(&block_refs, &aux_blocks, 21)?;

// Create engine and process
let mut engine = cam_isp::engine::select_engine()?;
let head = blocks.remove(0);
engine.build(head, blocks, Some(aux_blocks), 21)?;

// Process with full metadata
let params = ProcessParams::new(1920, 1080, &raw_data);
let frame = engine.process(&params)?;

// HEAVY profile produces all metadata
def print_metadata(aux: &IspAuxOutput) {
    println!("=== HEAVY Profile Metadata ===");
    println!("Channel Means: {:?}", aux.channel_means);
    println!("Tone Stats: {:?}", aux.tone_stats);
    println!("WB Gains: {:?}", aux.wb_gains);
    println!("Histogram: {:?} bins", aux.histogram.as_ref().map_or(0, |h| h.len()));
    println!("Zone Stats: {:?} zones", aux.zone_stats.as_ref().map_or(0, |z| z.len()));
    println!("Focus Metric: {:?}", aux.focus_metric);
    println!("Calibration Stats: {:?}", aux.calibration_stats);
    println!("CCT: {:?}K", aux.cct);
    println!("AE Gain: {:?}", aux.ae_gain);
}
```

### CCT-Based CCM

```rust
use cam_isp::ccm_engine;

// Compute CCM from color temperature
let cct = 5500;  // Kelvin
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

// Use in pipeline
params.isp_params.ccm = CcmParams::new(ccm_safe);
```

---

## Summary Tables

### Input Metadata Summary

| Category | Parameter | Type | Description | Required |
|----------|-----------|------|-------------|----------|
| **Raw Input** | width | u32 | Sensor width | Yes |
| | height | u32 | Sensor height | Yes |
| | bayer_pattern | i32 | CFA pattern (0-3) | Yes |
| | raw_data | Vec<u8> | Raw pixel data | Yes |
| | timestamp_ns | u64 | Capture timestamp | Yes |
| **CCM** | matrix | [f32;9] | Color correction matrix | Yes |
| | cct | i32 | Correlated color temperature | Optional |
| **AWB** | wb_gains | [f32;3] | White balance gains | Optional |
| | channel_means | [f32;3] | RGB channel means | Computed |
| **AE** | ae_gain | f32 | Auto-exposure gain | Optional |
| | tone_stats | [f32;3] | Tone curve stats | Computed |
| **AF** | focus_metric | f32 | Sharpness metric | Computed |
| | calibration_stats | [f32;24] | Quad-level stats | Computed |
| **Lens** | vignetting_strength | f32 | Vignetting correction | Optional |
| | distortion_k1/k2 | f32 | Distortion coefficients | Optional |
| | chromatic_aberration | f32 | CA correction | Optional |
| **Display** | format | OutputFormat | Output format | Yes |
| | rotate_mode | i32 | Orientation | Optional |

### Output Metadata Summary

| Category | Field | Type | Description | Profile |
|----------|-------|------|-------------|---------|
| **AWB** | channel_means | Option<[f32;3]> | RGB channel means | All |
| | wb_gains | Option<[f32;3]> | Computed WB gains | All |
| **AE** | tone_stats | Option<[f32;3]> | Tone curve stats | MED+ |
| | histogram | Option<Vec<f32>> | RGB histogram | HEAVY+ |
| | zone_stats | Option<Vec<f32>> | Per-zone stats | All |
| **AF** | focus_metric | Option<f32> | Sharpness metric | All |
| | calibration_stats | Option<[f32;24]> | Quad-level stats | All |
| **Color** | cct | Option<f32> | Color temperature | All |
| **Exposure** | ae_gain | Option<f32> | AE gain | All |
| **Scene** | scene_category | Option<String> | Scene classification | All |
| | af_phase | Option<String> | AF phase | All |
| **Motion** | vcm_position | Option<i32> | Lens position | All |
| | eis_compensation | Option<[f32;3]> | EIS compensation | PRO |

### Profile Comparison

| Profile | Level | Blocks | Stats Blocks | CCM | AWB | AE | AF | Lens | Features |
|---------|-------|--------|-------------|-----|-----|----|----|------|----------|
| LITE | 0 | 16 | 3 | Basic | Basic | Basic | Basic | None | Minimal |
| MED | 1 | 16 | 4 | Full | Full | Full | Full | None | Balanced |
| HEAVY | 2 | 16 | 5 | CCT-based | Multi-illuminant | Full | Full | Full | Complete |
| PRO | 3 | 16+ | 5+ | Everything | Everything | Everything | Everything | Everything | All |
| UNIFIED | 3 | 16+ | 5+ | Everything | Everything | Everything | Everything | Everything | Complete |
| TEST | 0 | 16 | 2 | Identity | Basic | None | Basic | None | Testing |

### Block Metadata Production

| Block | Primary Output | Metadata Output |
|-------|----------------|-----------------|
| RawInputBlock | Raw tensor | None |
| UnpackBlock | Unpacked float | None |
| NormalizeBlock | Normalized | None |
| CfaBlock | CFA-processed | None |
| BlcBlock | Black-level corrected | None |
| BayerWbBlock | White-balanced | wb_gains |
| DemosaicBlock | RGB image | None |
| DemosaicCcmBlock | Demosaiced + CCM | None |
| CcmBlock | Color-corrected | None |
| ToneBlock | Tone-mapped | tone_stats |
| DisplayBlock | Output format | None |
| ZoneStatsBlock | None | zone_stats |
| ChannelMeansBlock | None | channel_means |
| ToneStatsBlock | None | tone_stats |
| CoarseHistogramBlock | None | histogram |
| CalibrationBlock | None | calibration_stats |
| FcsBlock | FCS-processed | fcs_stats |
| LdciBlock | LDCI-processed | ldci_stats |
| EeBlock | Edge-enhanced | ee_stats |
| BilateralBlock | Denoised | denoise_stats |

---

## Conclusion

The SoftISP pipeline provides comprehensive metadata support for:

1. **Input**: Full 3A parameters (AE, AWB, AF), CCM with CCT-based interpolation, lens corrections, and scene context
2. **Output**: Rich auxiliary metadata including statistics, histograms, focus metrics, and scene classification
3. **Flexibility**: Profile-based configuration allows trading off quality vs. performance
4. **Extensibility**: Custom parameters and blocks can be added as needed

The pipeline is designed to support both simple use cases (LITE profile with minimal metadata) and advanced scenarios (PRO/UNIFIED profiles with full metadata extraction and processing).

---

*Generated on: 2026-07-09*
*Version: SoftISP Pipeline Analysis v1.0*
*Source: cam-rust/cam-isp pipeline codebase*
