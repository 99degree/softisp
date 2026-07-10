# Controller API — Unified ISP Parameter Prediction

## Overview

The Controller API provides a unified interface for ISP parameter prediction. It supports both rule-based and neural network controllers, allowing runtime switching and fallback mechanisms.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    ControllerApi Trait                           │
├─────────────────────────────────────────────────────────────────┤
│  fn analyze_and_update(&mut self, frame: &IspFrame) -> IspParams│
│  fn has_model(&self) -> bool                                    │
│  fn load_model(&mut self, path: &str) -> bool                  │
│  fn last_params(&self) -> Option<&IspParams>                   │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
                    ┌───────────────┐
                    │   Controller  │ (enum)
                    └───────┬───────┘
                            │
            ┌───────────────┼───────────────┐
            ▼                               ▼
    ┌───────────────┐               ┌───────────────┐
    │  RuleBased    │               │    Neural     │
    │(IspController)│               │(NeuralCtrl)   │
    ├───────────────┤               ├───────────────┤
    │ • Rule-based  │               │ • ONNX model  │
    │ • 3A heuristics│              │ • 267→20 MLP  │
    │ • Always works│               │ • Fallback    │
    └───────────────┘               └───────────────┘
```

## Usage Examples

### Basic Usage

```rust
use cam_isp::controller_api::{Controller, ControllerApi};

// Create controller (default: neural with fallback)
let mut controller = Controller::neural();

// Process frame
let frame = IspFrame { ... };
let params = controller.analyze_and_update(&frame);

// Apply to pipeline blocks
bayer_wb_block.set_gains([params.wb.r, params.wb.g, params.wb.b]);
ccm_block.set_matrix(&params.ccm.matrix);
tone_block.set_contrast(params.tone.contrast);
```

### Load Neural Model

```rust
// Load INT8 quantized model
if controller.load_model("model_int8.onnx") {
    println!("Neural model loaded successfully");
} else {
    println!("Using rule-based fallback");
}
```

### Switch Controllers

```rust
// Switch to rule-based
controller = Controller::rule_based();

// Switch to neural
controller = Controller::neural();

// Check which is active
if controller.has_model() {
    println!("Neural model active");
} else {
    println!("Rule-based active");
}
```

### Generic Code with Trait

```rust
fn process_frame<C: ControllerApi>(controller: &mut C, frame: &IspFrame) -> IspParams {
    controller.analyze_and_update(frame)
}

// Works with any controller type
let mut rule_based = Controller::rule_based();
let mut neural = Controller::neural();

let params1 = process_frame(&mut rule_based, &frame);
let params2 = process_frame(&mut neural, &frame);
```

## Controller Types

### RuleBased (IspController)

Rule-based controller using traditional 3A algorithms:
- Auto-exposure based on luminance histogram
- Auto-white-balance using gray world assumption
- Auto-focus using contrast measurement

**Pros:**
- Always works (no dependencies)
- Fast (< 0.1ms)
- Deterministic

**Cons:**
- Heuristic-based
- May not optimal for all scenes

### Neural (NeuralController)

Neural network controller using distilled model:
- Input: 267 dims (256 histogram + 11 metadata)
- Output: 20 dims (3 WB + 9 CCM + 7 tone + 1 zoom)
- Architecture: MLP (256→128→64→32) + 4 heads

**Pros:**
- Learned from data
- Optimized for quality
- Handles complex scenes

**Cons:**
- Requires model file
- Falls back to rule-based on failure

## Model Specification

| Aspect | Value |
|--------|-------|
| Input | 267 dims (256 histogram + 11 metadata) |
| Output | 20 dims (3 WB + 9 CCM + 7 tone + 1 zoom) |
| Size | 0.6 MB (INT8), 2.1 MB (FP32) |
| Latency | 1.2 ms (INT8), 3.2 ms (FP32) |

See [MODEL_SPECIFICATION.md](../../isp-rectifier/MODEL_SPECIFICATION.md) for details.

## Integration with UnifiedPipeline

```rust
use cam_isp::unified_pipeline::UnifiedPipeline;

// Create pipeline
let mut pipeline = UnifiedPipeline::new(config)?;

// Load neural model at runtime
pipeline.load_model("model_int8.onnx");

// Process frame (uses neural if available, else rule-based)
let output = pipeline.process(&raw_data, width, height)?;

// Switch controller
pipeline.use_rule_based_controller();  // disable neural
pipeline.use_neural_controller();      // re-enable neural
```

## Fallback Chain

When neural model fails, the system falls back:

1. **Neural inference** → Try ONNX model
2. **Last good params** → Use previous successful parameters
3. **Rule-based** → Use IspController heuristics
4. **Defaults** → Use factory default parameters

## Temporal Smoothing

Both controllers apply temporal smoothing to prevent flickering:

```rust
// Smoothing formula
params_t = 0.7 × params_{t-1} + 0.3 × params_t
```

This blends 70% of the previous frame's parameters with 30% of the new computation.

## Performance

| Metric | RuleBased | Neural (INT8) | Neural (FP32) |
|--------|-----------|---------------|---------------|
| Latency | < 0.1 ms | 1.2 ms | 3.2 ms |
| Memory | 0 MB | 0.6 MB | 2.1 MB |
| Accuracy | Baseline | +15% | +20% |

## Testing

Run controller tests:

```bash
cargo test -p cam-isp --lib controller_api
```

Tests cover:
- Rule-based controller
- Neural controller with fallback
- Model loading
- Parameter access
- Generic trait usage
