# P3 — Runtime tensor-name convergence

Date: 2026-11-26

## Symptom

The same runtime decision is spelled three ways:

- `ProcessParams::bayer_gains` is the caller-facing RBGG gain vector.
- `BayerWbBlock/gains` is the logical `[1,4,1,1]` WB tensor when the WB block
  survives conversion.
- `DemosaicCcmBlock/w` is the fused `[3,4,1,1]` matrix that consumes those gains
  when WB is folded into demosaic+CCM.
- `CcmBlock_ccm/matrix` is the last-resort per-colour CCM sink.

`docs/reference/TensorNames.java` already defines canonical names for the first
three tensor sinks, but the Rust blocks emitted only their legacy names.

## Canonical registry entries used

| Meaning | Canonical name | Legacy Rust name |
|---|---|---|
| Bayer-plane WB gains `[4]` | `BAYERWBBLOCK/BAYER_GAINS` | `BayerWbBlock/gains` |
| Fused demosaic+CCM weights `[3,4,1,1]` | `DEMOSAICCCMBLOCK/W` | `DemosaicCcmBlock/w` |
| Fused demosaic+CCM bias `[3]` | `DEMOSAICCCMBLOCK/B` | `DemosaicCcmBlock/b` |
| CCM matrix `[3,3]` | `CCMBLOCK/CCM_MATRIX` | `CcmBlock_ccm/matrix` |
| CCM bias `[3]` | `CCMBLOCK/CCM_BIAS` | `CcmBlock_ccm/bias` |

`DEMOSAICCCMBLOCK/W` is intentionally a generic fused weight matrix, not a WB
gain vector. `DemosaicBlock/w` and `DemosaicBlock/b` have no registry entry, so
they stay on their primitive names rather than inventing a canonical name that
would collide with the fused sink detection.

## Fix

- Add `cam_isp::tensor_names` as the single Rust mirror of the registry subset.
- Emit canonical names from `BayerWbBlock`, `DemosaicCcmBlock`, and the
  production `CcmBlock` instance while leaving frame/intermediate tensor names
  (`BayerWbBlock/frame`, `CcmBlock_ccm/frame`, `/zero`, `/one`) unchanged.
- In `MnnEngine`, detect every sink by canonical-or-legacy alias and write each
  changed runtime tensor under both names. Already-loaded MNN models and probes
  therefore keep working; the flush path already skips absent session inputs.
- Preserve the existing sink priority (WB block, fused demosaic+CCM, primitive
  demosaic, CCM matrix) so folding cannot double-apply WB.

## Explicit deferrals

- `DemosaicBlock/w` / `DemosaicBlock/b`: no canonical registry entry.
- `bayer_proc/wb_gains` and `models/old/bayer_model.onnx`: legacy standalone
  namespace with no production profile emitter.
- Python `awb.gains*` controller graphs: controller-model namespace
  (`ALGO_AWB/AWB_GAINS`), not the ISP runtime sink.
- GLSL/C++ uniforms such as `wb_gains` and `ccm`: positional shader bindings,
  not MNN tensor names.
- `ProcessParams::{bayer_gains, awb_gains, ccm_matrix}`: public Rust API; 184
  assignment sites. Document the mapping instead of breaking callers.
- A repo-wide suffix normalization across unrelated blocks: separate roadmap
  item; it would invalidate many generated models without improving WB/CCM.

## Verification

Run before landing:

```bash
cargo fmt --all -- --check
git diff --check
cargo clippy -p cam-isp --lib --features rectifier -- -D warnings
cargo clippy -p cam-isp --lib --features mnn -- -D warnings
cargo test -p cam-isp --lib --features rectifier --no-fail-fast
cargo test -p cam-isp --lib --features "rectifier mnn" --no-fail-fast
cargo run -p cam-isp --example gen_api_docs
```

Results: fmt and `git diff --check` clean; clippy clean for `cam-isp` with
`rectifier` and `rectifier mnn`; 851 rectifier lib tests and 886 real-MNN lib
tests passed (2 ignored in each). The ignored MNN probes
`probe_isp_ccm_bindings`, `probe_wb_fused`, and `probe_isp_contract` all pass,
confirming legacy names still load. `gen_api_docs` was run but only the
WB/CCM naming notes were kept; unrelated stale generator output was not taken.
