# SoftISP

A complete camera ISP pipeline in Rust with Vulkan GPU acceleration.

## What It Does

Converts raw Bayer sensor data into display-ready images in real-time:

```
Raw Sensor → BLC → WB → Demosaic → CCM → Tone → Display
```

## Key Features

- **40+ ISP blocks** for full camera processing
- **Vulkan GPU acceleration** (1ms at HD, 2ms at FHD)
- **Neural controller** (93 FPS ONNX inference)
- **Multiple profiles** (LITE/MED/HEAVY/PRO/UNIFIED)
- **AOSP HAL3** binderized interface

## Quick Start

```rust
// Create GPU engine
let mut engine = select_engine_by_name("mnn_vulkan")?;

// Process frame
let params = ProcessParams::new(3840, 2160, &raw_bayer);
let frame = engine.process(&params)?;
```

## Performance

| Resolution | HEAVY | LITE |
|------------|-------|------|
| HD | 1.0ms | 4.9ms |
| FHD | 2.0ms | 8.0ms |
| 4K | 8.0ms | 24ms |

## Documentation

- **[Architecture](docs/architecture/ARCHITECTURE.md)** - System design
- **[Summary](docs/SUMMARY.md)** - Detailed overview
- **[Pipeline Blocks](docs/architecture/PIPELINE_BLOCKS.md)** - Block reference
- **[MNN/Vulkan Guide](docs/guides/MNN_VULKAN_GUIDE.md)** - GPU setup

## Testing

```bash
cargo test -p cam-isp              # 706+ tests
cargo test -p cam-isp --features mnn  # With GPU
```

## License

- **Code:** MIT OR Apache-2.0
- **Models:** CC BY-NC 4.0 (non-commercial)
