# Cache output metadata and CString names on the visible path (2026-11-26)

## Symptom

`perf_critical_path` splits the visible frame into setup, tensor upload,
`runSession`, output metadata and output copy. The output metadata phase still
reconstructs work that is fixed for a prepared frame: it resolves the output
tensor again, reads its shape/type, recomputes channel packing flags, allocates
two `CString`s for tensor names, and repeats the channel-count check on every
frame. This is small per call but sits between `runSession` and presentation.

## Root cause

`MnnEngine::prepare()` already owns a session slot, but only stores raw geometry
and the output tensor name in `PreparedMeta`. `run_prepared()` therefore rebuilds
the derived output layout and the C names after inference, inside the visible
window. The frame input name is also converted from `String` to `CString` on
every frame even though it is fixed at `build()` time.

## Fix

- Add a pure `derive_output_layout()` helper that turns the output tensor shape,
  data type, `OutputFormat` and `has_fused_display` into one small `OutputLayout`
  value: height, width, channels, pixel count, element width and the packed-U32 /
  packed-display flags.
- Compute the layout in `prepare()` after any session resize, while that work is
  overlapped with camera fill, and store it in `PreparedMeta`.
- Store the output tensor name as a `CString` in `PreparedMeta` and the frame
  input name as an `Option<CString>` on `MnnEngine`, both built once instead of
  once per frame.
- `run_prepared()` consumes the cached layout and pointers; the bytes copied to
  the caller and all log/timing fields stay unchanged.

## Verification

Hermetic tests cover the pure layout derivation for packed-U32, u8 tail, raw
float and fallback-shape cases. The canonical gate is fmt, the per-crate clippy
loop, `cargo test -p cam-isp --lib --features rectifier`, and
`cargo check -p cam-isp --tests --features "rectifier mnn"`. The on-device
`perf_critical_path` probe should show a smaller `output_meta` column; absolute
numbers drift with device temperature, so compare rows from the same run.

## Roadmap position

Backlog item 4 in `docs/overview/ROADMAP.md`. It does not touch upload dedup,
AHB lifetime or the 3A cadence, and can be validated independently.
