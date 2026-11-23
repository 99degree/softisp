# Stats Integration — Triple-Buffered Feedback Loop

```
Controller → engine.process(frame, ccm, tone, bayer, awb)
              ├─ set_extra_inputs()     ← write params to ONNX initializers
              ├─ inference              ← ONNX graph (display + stats outputs)
              ├─ read stats tensors     ← channel means, tone, hist, zones
              └─ write_stats()          → RollingStats write-slot
            controller.rotate_stats()   → process previous, ready next
```

- **3-slot rotate** (`write_idx` → `process_idx` → `ready_idx`): engine writes frame N, controller processes frame N-1, next frame reads frame N-2 params. No lock contention.
- **ONNX dual-registration**: extra inputs (CCM, tone, WB) registered as BOTH initializer AND graph input for runtime override.
- **Bayer pattern fusion**: `set_extra_inputs(bayer_pattern)` selects RGGB/BGGR/GRBG/GBRG demosaic weights fused with CCM matrix.
- **Adaptive stats downscale**: `stats_downscale_max` auto-inserts ResizeBlock before stats for 4K+ sensors.
