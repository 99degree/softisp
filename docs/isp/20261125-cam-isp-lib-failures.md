# cam-isp lib failures — findings

status: resolved

## Symptom

Two hermetic `cam-isp --lib` tests failed with `--features "rectifier mnn"`:

1. `unified_pipeline::tests::test_unified_force_input16` — MNN `runSession`
   returned `COMPUTE_SIZE_ERROR` (`n=-3`).
2. `manager::tests::test_manager_build_and_process` — the returned BGRA frame
   was 230400 bytes where 160x90x4 = 57600 bytes was expected.

## Reproduction

```bash
LD_LIBRARY_PATH=cam-rust/lib/arm64-v8a:cam-rust/lib/aarch64-v8a:$LD_LIBRARY_PATH \
cargo test -p cam-isp --lib --features "rectifier mnn" --no-fail-fast \
  unified_pipeline::tests::test_unified_force_input16 -- --nocapture

LD_LIBRARY_PATH=cam-rust/lib/arm64-v8a:cam-rust/lib/aarch64-v8a:$LD_LIBRARY_PATH \
cargo test -p cam-isp --lib --features "rectifier mnn" --no-fail-fast \
  manager::tests::test_manager_build_and_process -- --nocapture
```

## Root cause

### Forced 16-bit input

The `force_input16` profile selected `RawInput16Block`, declaring an INT16
`[1,1,H,W]` graph input and an in-graph `Cast(INT16→FLOAT32)` before
`UnpackCfaBlock`. MNN's converter upcasts that graph input to FLOAT32 but the
Vulkan backend cannot execute the INT16→FLOAT Cast, so `runSession` failed with
`COMPUTE_SIZE_ERROR`. The engine already has a `RawFloat` host path that widens
the u16 sensor buffer before upload; the graph was still asking MNN to do the
same conversion a second time.

### BGRA output size

Output-decoding inferred a packed display word from
`has_fused_display && is_u8_format` without checking the output tensor's shape.
The manager test's graph returns a four-channel FLOAT Conv (`[1,4,H,W]`), which
was narrowed as if it were one packed UINT32 word per pixel. Each float was
copied as four bytes, producing 16 bytes per pixel instead of four.

## Fix applied

- Added `RawFloatInputBlock` with a FLOAT32 `[1,1,H,W]` graph input and
  `UnpackMode::NativeFloat`. `UnpackCfaBlock` feeds its stride-2 Conv directly
  from the graph input, omitting the Vulkan-hostile Cast. The engine's existing
  `RawFloat` mode performs the u16→f32 widening on the host.
- Switched both `force_input16` profile branches to the float head. Updated the
  profile/docs and added a unit test for the Cast-free native-float chain.
- Added a `graph_channels == 1` precondition before treating the tail as a
  packed display word. Both `packed_display_by_type` and the fused-display
  fallback now honour it; `isp.display_uint32_bgra` / `OutputFormat::PackedRgb`
  handling is unchanged.
- Removed all temporary diagnostic `eprintln!` probes from `manager.rs` and
  `mnnengine.rs`, and restored the force-input16 test's original 1280x720 input.

## Findings

- The graph input type must describe what the engine uploads, not merely the
  sensor buffer's storage type; MNN Vulkan decides supported ops from the
  converted graph.
- A fused-display flag alone is not sufficient to identify packed output. The
  tensor's channel count is the authoritative discriminator between a one-word
  BGRA tail and a four-channel float Conv.
- No production APK/UI change was made. The final packed op remains
  `isp.display_uint32_bgra`, and all temporary diagnostics are gone.

## Verification

```text
cargo fmt --all -- --check                                      clean
cargo clippy -p cam-isp --lib --features rectifier              clean
cargo clippy -p cam-isp --lib --features "rectifier mnn"        clean
cargo clippy -p cam-camera-app --all-features                   clean
cargo test -p cam-isp --lib --features "rectifier mnn" \
  --no-fail-fast                                851 passed; 0 failed; 2 ignored
```

Both previously failing tests now pass individually as well.
