# SIMD Backend

```
CpuEngine::process() → simd: &'static dyn SimdEngine (auto-selected at init)
  ├─ NeonDotprod (ARMv8.4) ← dotprod+fp16+neon
  ├─ NeonFp16    (ARMv8.2) ← fp16+neon
  ├─ Neon        (ARMv8.0) ← 128-bit NEON
  └─ Scalar                ← always available

Op              Speedup  Method
normalize_u16    ~4×     8 u16→f32/iter
apply_ccm        ~3.5×   4 pixels/iter (3×3 FMA)
apply_ae_gain    ~4×     4 f32/iter
display_output   ~3×     batch 4 BGRA
```
