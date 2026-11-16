docs(isp): record what the shader review found and what it did not cover

Two questions came up that the earlier audit did not answer, so both are written
down here with the evidence rather than left implicit.

## Is the review complete?

The *rendering* half is. For all 80 real `isp_*.spv` the binding map is read out
of the bytecode (`scanSpvBindings`), not assumed:

    InBuf@3 OutBuf@4   67 shaders   <- the universal map
    InBuf@1 OutBuf@2    8 shaders   <- legacy sources (crop, raw_blc, lsc, ...)
    no frame tensor     4 shaders   <- algo_ae/awb/cct/gamma_scalar

77 resolve to exactly what they declare; the 4 exceptions bind named buffers
(`StatsBuf`, `MeansBuf`, …) and never enter the frame path. `isp_crop_debug` is
inconsistent with itself (`InBuf@1`, `OutBuf@4`) but is a debug op.

What it does *not* cover, and should not be read as covered:

  * **21 of 81 shaders are `// placeholder compute shader`.** They copy their
    input. `probe_direct_opset` shows the consequence: MED in MnnIspMin mode
    renders a black frame (mean 0.0000 vs 0.9864 primitive) because `isp.ee`
    is one of those stubs and `isp.display` never receives its `gamma`.
  * **Sixteen implemented shaders read a coefficient buffer at binding 5+**
    (`isp_blc` BlcBuf@5, `isp_ccm` ParamBuf@5, `isp_demosaic_g2` two buffers at
    5/6, `isp_algo_gamma_scalar` up to binding 10, …). The map now routes
    metadata there, but **no softisp block populates those buffers** — the
    blocks still emit their coefficients as attributes. Those ops would run with
    zeroed coefficients.
  * **No `isp.*` shader handles fp16 tensor storage** except
    `isp.display_rgba_f16`, which writes packed halves. The engine defaults to
    `Precision_Normal`, which on this backend sets `mUseFP16 = true`
    (`VulkanBackend.cpp:176`). A fused float shader reading a half-stored tensor
    would misinterpret the data. That combination has never been exercised
    because fusion is not enabled.

So: the binding mismatch is fixed and verified; the shader *implementations* are
only partly usable, and the three gaps above are what stands between "fusion is
wired up" and "fusion can be turned on".
