# FHD Benchmark — Block Costs

```
Block            CPU 9.0fps  Vulkan 12.3fps  Δ
unpack_cfa       37.7ms      42.4ms         +4.7ms
demosaic_ccm     19.2ms      ~0ms (fused)   🏆
ldci             28.8ms       2.3ms         -26.5ms ← Conv-heavy
  ee             12.0ms       4.0ms          -8.0ms
display          17.4ms      23.4ms          +6.0ms ← mem xfer
Total:~111ms                Total:~81ms      -27%
```

Vulkan wins on Conv-heavy blocks (ldci -92%, ee -67%) but adds kernel-launch overhead on elementwise ops.
- **4K→4K MNN stress test** (post-DCE fix, real ISP graph executed):
  - HEAVY profile: **79.8 FPS** avg (2394 frames / 30s, P99 129.1ms)
  - UNIFIED profile (tiled 2×2): **141.2 FPS** avg (4236 frames / 30s, P99 8.7ms)
- Older per-block 4K→FHD: ~~≈ 4.7 FPS~~ before TypeProto/Extra-input fixes (prevented MNN DCE).
