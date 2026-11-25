# ToneLutBlock build break — diagnosis and fix (2026-11-25)

## Symptom

After syncing to `9bb1d799`, `cargo check --workspace --all-targets` fails in
`cam-isp`:

- `E0433`: `ToneLutBlock` not in scope at `src/integration.rs:194`
- `E0609`: `cam_types::ToneParams` has no `gamma` field at `src/mnnengine.rs:1202`
- `E0609`: `cam_types::ToneParams` has no `curve_lut` field at `src/mnnengine.rs:1208`

`cargo fmt --all -- --check` also fails on the newly added benchmark example.

The first full-workspace pass exposed a second cluster of stale call sites from
the same upstream refresh:

- `E0063` in `ProcessParams` literals: the struct gained `input_buffer`,
  `output_buffer`, `input_ahb` and `output_ahb`; most probe fixtures still used
  exhaustive literals without them.
- `E0061` on `mnn_session_create`: the FFI gained a `precision: c_int` argument.
- `E0599`: `MnnBackendType::Cpu` was removed; only `Vulkan` remains.
- `E0061` on `mnn_run_true_zero_copy`: the FFI now takes explicit
  `input_name`/`output_name` C strings.
- `E0599`: `RawInputBlock::with_elem_type` was removed; raw element types are
  selected by choosing `RawInputBlock`, `RawInput16Block` or
  `RawFloatInputBlock`.
- `E0599`: `ProcessParams` is no longer `Clone`; the neural end-to-end
  benchmark must update the single params value in place.
- `E0603`: `GpuWarpParams` is re-exported at `cam_isp::GpuWarpParams`, not from
  `unified_pipeline`.
- `E0277`: `ProcessParams` is now `!Sync` because it holds raw buffer handles;
  `bench_parallel` must build one params value per worker instead of sharing one
  across `thread::scope`.
- `E0433`: `probe_convert_eq` called `serde_json`, which is not a dependency of
  `cam-isp` and must not be added just for a diagnostic example.
- `E0061` on `PipelineProfile::custom`: the constructor gained `input_type`,
  `post_ccm_downscale`, `normalize_at_display`, `opset_mode` and `fuse_stages`.

## Root cause

Commits `583c3054`, `479da0c2` and `2ab3d26a` added `ToneLutBlock` in
successive, partially overlapping patches:

1. `integration.rs` wired the new block into the CPU fallback pipeline without
   extending its explicit `use crate::blocks::{...}` import list.
2. `mnnengine.rs::set_extra_inputs()` was left at the intermediate revision
   from `479da0c2`: it writes `tone.gamma` and `tone.curve_lut`, but the local
   `tone` is a `cam_types::ToneParams` (fields `gamma_recip`, no `curve_lut`).
   The 7-point LUT actually lives on `crate::isp_params::IspParams`, which
   `set_extra_inputs()` already receives.
3. The same stale block writes LUT values into `ToneLutBlock/t0..t6`. `t_i`
   are the fixed segment positions `i/6` in the block's graph; the LUT values
   belong in `y0..y6`, and the slopes must be derived from them.
4. The block writes parametric `contrast`, `brightness` and `inv_gamma` into
   `ToneLutBlock` even though `ToneBlock` runs immediately before it with those
   same values. The documented hybrid design is parametric first, neural LUT
   second; applying parametric tone twice changes the picture.
5. `ToneLutBlock::extra_inputs()` / `extra_input_defaults()` push the single
   `seg_width` tensor six times. MNN gets duplicate graph inputs, and the
   dedicated defaults are overwritten by later entries. `let y = ...` in
   `nodes()` is unused.

## Fix

- Import `ToneLutBlock` in `integration.rs`.
- Have `mnnengine.rs` write only `ToneLutBlock/y0..y6` from
  `p.isp_params.tone.curve_lut` when it contains exactly 7 points. Leave `t_i`,
  `seg_width`, `zero`, `one`, `contrast`, `brightness` and `inv_gamma` at the
  block's identity defaults: tone stays a single parametric pass, the neural
  curve refines it.
- Derive segment slopes inside the ONNX graph (`(y_{i+1} - y_i) / seg_width`)
  so every LUT point has a real consumer and no extra per-frame uploads are
  needed.
- Emit each input/default tensor name exactly once and update the unit tests
  accordingly.
- Replace exhaustive `ProcessParams` literals in probes with
  `..ProcessParams::new(...)` (or explicit default fields) so future buffer
  fields cannot break every fixture at once.
- Update stale MNN example/test call sites to the Vulkan-only session API and
  pass the named input/output tensors to `mnn_run_true_zero_copy`.
- Select raw input blocks by concrete type instead of the removed
  `with_elem_type` builder.
- Drop the unused `GpuWarpParams`/`UnifiedConfig` import from
  `bench_unified_full`; the benchmark only needs `IspBlock` and `OutputFormat`.
- Compare converter JSON dumps directly in `probe_convert_eq` instead of adding
  a JSON parser dependency for an example.
- Give each worker in `bench_parallel` its own `ProcessParams` value while
  sharing only the immutable raw byte slice.
- Extend the `PipelineProfile::custom` call in `bench_blocks` with the new
  profile arguments, preserving the old feature selection.

## Verification

Run from the workspace root after `cargo fetch --locked`:

- `cargo fmt --all -- --check` — pass.
- `cargo check --workspace --all-targets` — pass.
- Per-crate clippy loop (`-D warnings`), with `cam-isp` on `rectifier` — all
  pass; `cargo clippy -p softisp-anw --all-features --no-deps -D warnings`
  passes.
- Hermetic lib tests — `cam-types`, `cam-hal`, `cam-core`, `cam-onnx`,
  `cam-motion` all pass; `cam-isp --features rectifier` passes 829 tests with 2
  ignored.
- Real MNN lib tests — `cam-isp --features "rectifier mnn"` passes 864 tests
  with 2 ignored.
- `git diff --check` — clean; no `build_vk` references remain in tracked files.

`cargo test --workspace --doc --all-features` cannot complete on the Termux
Android host: `--all-features` activates `rscam` (Linux/FreeBSD-only;
`compile_error!` on Android) and `ort-sys` (build script cannot determine a
cache directory here). Those are pre-existing platform constraints outside this
fix; the CI gate runs on Linux with network access. All doc-testable crates
that do not pull those native dependencies were otherwise checked by the lib
and workspace test runs above.
