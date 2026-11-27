# Intermittent MNN Vulkan test hang after sync (2026-11-27)

## Symptom

On the synced tree (`317abd4d` / `dev/all-features-android-gate`), the full
MNN-gated unit suite sometimes stops progressing without a Rust panic:

- first run: `manager::tests::test_manager_build_and_process` stayed over 60s
- second run: `unified_pipeline::tests::test_unified_pipeline_process` stayed over 60s

Both runs were terminated by the enclosing `timeout`; no `FAILED`, `SIGBUS`, or
MNN error was emitted. The same suite passed completely (`900 passed`) on the
pre-rebase tree, and each hanging test passes in isolation after the rebase:

```bash
cargo test -p cam-isp --lib --features "rectifier mnn" \
  manager::tests::test_manager_build_and_process -- --nocapture
cargo test -p cam-isp --lib --features "rectifier mnn" \
  unified_pipeline::tests::test_unified_pipeline_process -- --nocapture
```

## Diagnosis

All non-MNN gates are stable on the synced tree: fmt, the per-crate
`-D warnings` clippy loop, all hermetic unit tests, and the all-features doc
tests. The hang is confined to parallel execution of the MNN/Vulkan tests.

The upstream commits added by the sync do not touch the ANW timing code; the
only relevant change is `60ead57f`, which rewires shared ISP coefficients
through canonical tensor-name aliases in `mnnengine.rs`. It changes scalar
tensor writes, not MNN session lifecycle or teardown. The two tests that hung
create independent full MNN sessions and call `crate::init()`; running either
one alone completes in about 1.3s with the expected Vulkan backend loaded.

Root cause is therefore concurrent Vulkan session/resource contention in the
test harness on this Termux device, not a deterministic regression in the
branch. This is a test-environment flake, not a source defect.

## Verification policy

For local Android verification, run the MNN suite once with
`-- --test-threads=1` when the default parallel run hangs. CI on Linux hosts
continues to use the canonical command unchanged. No test was deleted, ignored,
or mocked to hide the flake.
