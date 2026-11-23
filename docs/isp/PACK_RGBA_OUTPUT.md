# Output Format — pack_rgba default

| Mode | ONNX output | `output_channels` | Use |
|------|------------|-------------------|-----|
| **pack_rgba** (default) | `[1,1,H,W]` INT32 | `1` | Production — `R*65536+G*256+B` per i32, unpacked to BGRA |
| bg4a (verify) | `[1,4,H,W]` FLOAT | `4` | Benchmark/verification — Conv(1×1) BGRA |
| legacy float | `[1,3,H,W]` FLOAT | `3` | Post-proc chaining |

Set via `PipelineManager::output_channels` or `ProcessParams::output_channels`.
