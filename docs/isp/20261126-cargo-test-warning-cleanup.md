# Cargo test warning cleanup — findings (2026-11-26)

## Symptom

`./build-apk.sh` succeeds, but the hermetic cargo test commands still print warnings:

- `cargo test -p softisp-anw --lib --all-features --no-fail-fast` prints 12 warnings from `cam-ipa` while compiling it as a dependency (`unused import`, `unused variable`, `unused_mut`, and `dead_code`).
- `cargo test -p cam-isp --lib --features "rectifier" --no-fail-fast` prints 10 warnings from `cam-rust/cam-isp/src/upload_cache.rs` (`dead_code` for its constants, statics, and helper functions).
- `cargo test -p cam-isp --lib --features "rectifier mnn" --no-fail-fast` does not print the `upload_cache` warnings, which means the module is only live when the `mnn` feature is enabled.

The canonical per-crate clippy gate with `-D warnings` is already clean; these warnings are emitted only by dependency/test compilation configurations that do not use the canonical feature set.

## Reproduction

```bash
cargo test -p softisp-anw --lib --all-features --no-fail-fast 2>&1 | grep '^warning:'
cargo test -p cam-isp --lib --features "rectifier" --no-fail-fast 2>&1 | grep '^warning:'
```

Logs from the failing runs: `.tmp/final-anw-tests.log`, `.tmp/final-tests.log`.

## Root cause

- `cam-isp/src/upload_cache.rs` is compiled for unit tests without the `mnn` feature because `lib.rs` gates it as `#[cfg(any(feature = "mnn", test))]`. Its switch helpers are only called from `mnnengine.rs`, which is `mnn`-only, so in the `rectifier`-only test build they are dead code. The unit tests exercise the cache itself but never touch the switch accessors.
- `cam-ipa` contains stale code from earlier IPA scaffolding:
  - `engine_manager.rs` imports `cam_isp::blocks::*` and carries a private `ipa_format_to_output_format` helper that duplicates the live helper in `context.rs`; neither is used.
  - `events.rs` imports `Arc`/`Mutex` that the event loop never uses.
  - `AhwBuffer::new` accepts `width`, `height`, and `format` but ignores them, and the struct stores descriptor metadata that no caller reads.
  - `IpaContext` owns a `BufferManager` that is never used because frames arrive through caller-supplied buffers.
  - `IpaController::analyze_and_update` ignores its `frame` argument and needlessly takes the mutex as `mut`.
  - `IpaContext::process_frame` accepts `plane_in`/`plane_out` values that the opaque buffer path does not use.

## Fix applied

- `cam-isp/src/upload_cache.rs`: the switch accessors are now exercised by a
  hermetic unit test, so the `rectifier`-only test build no longer reports them
  as dead code. No `#[allow(dead_code)]` was added.
- `cam-ipa/src/engine_manager.rs`: removed the unused `blocks::*` and `IpaFormat`
  imports and the duplicate `ipa_format_to_output_format()` helper; `context.rs`
  already owns the live conversion.
- `cam-ipa/src/events.rs`: removed unused `Arc`/`Mutex` imports.
- `cam-ipa/src/buffer.rs`: `AhwBuffer::new()` now validates that the described
  AHB matches the requested geometry/format, stores the requested values, and
  exposes `format()`, `width()`, `height()` and `strides()` getters so descriptor
  metadata is actually consumed instead of being dead state.
- `cam-ipa/src/context.rs`: removed the unused `BufferManager` field/import and
  marked the unused `plane_in`/`plane_out` parameters as intentionally unused.
- `cam-ipa/src/controller.rs`: marked the unused `frame` parameter as
  intentionally unused and dropped a needless mutable mutex binding.

## Findings

- The warning sources were configuration-dependent dead code and stale
  scaffolding, not compiler false positives. Fixing the root causes keeps the
  canonical `-D warnings` clippy contract intact without blanket suppressions.
- The new `AhwBuffer` descriptor check fails closed on a mismatched buffer, so
  the getter cleanup also removes a silent geometry/format mismatch hazard.
- Verified clean after the edits:
  `cargo fmt --all -- --check`, canonical per-crate clippy loop,
  `cargo clippy -p softisp-anw --all-features --no-deps -- -D warnings`,
  857 `cam-isp --features rectifier` tests, 895 `rectifier mnn` tests and
  51 `softisp-anw` tests. No `warning:` lines remain in the hermetic test logs.
## Follow-up: `rscam` on Android host

`cargo test --workspace --doc --all-features --no-fail-fast` still exits 101 on
Termux/Android because `--all-features` enables `v4l2` in both
`cam-hal-linux` and `cam-hal-android`. Both crates declare `rscam` as a plain
optional dependency, so Cargo builds `rscam 0.5.5` for `target_os = "android"`
and its `compile_error!("rscam (v4l2) is for linux/freebsd only")` aborts the
doc-test build. The Android HAL also has no business probing V4L2 — camera
input comes from MediaNDK `AImageReader`/`AImage` and native `AHardwareBuffer`.

Fix applied: `rscam` is now declared only under
`[target.'cfg(target_os = "linux")'.dependencies]` in `cam-hal-linux`; every
V4L2/rscam code path there is gated on `all(feature = "v4l2", target_os = "linux")`
while stub APIs remain available on other targets. The `v4l2` feature, module and
`rscam` dependency were removed from `cam-hal-android` entirely: Android camera
input is the MediaNDK `AImageReader`/`AImage` path, not V4L2. Verification is the
same `--workspace --doc --all-features` command plus the canonical fmt/clippy
gate.

After the 2026-11-27 sync to `1b6ad529`, `cargo check -p cam-hal-linux
--all-features`, `cargo check -p cam-hal-android --all-features` and
`cargo check -p cam-binder --all-features` all pass on the Android host with no
`rscam` build. The full doc-test command then exposed two independent,
pre-existing all-features issues:

- `ort-sys 2.0.0-rc.15` panics with `could not determine cache directory`
  because `cache_dir_default()` has no Android branch. Setting
  `ORT_SKIP_DOWNLOAD=1` avoids the download path and defers the ORT link error
  as designed.
- With ORT download skipped, `cam-onnx/src/lib.rs` fails to compile because
  `ort 2.0.0-rc.15` changed `Session::builder()` to require an `&Environment`.
  This is a real API drift in the `ort` feature path, so the call now creates
  an environment with `ort::init().build()` before building the session.
- `cam-isp/src/onnx/mod.rs` has the same `Session::builder()` API drift in its
  `ort` feature path.
- `cam-isp/src/cabi.rs` still constructs `PipelineProfile` with the pre-`fuse_stages`
  field set and is missing six newer fields (`input_type`, `lsc_strength`,
  `post_ccm_downscale`, `normalize_at_display`, `opset_mode`, `fuse_stages`).
  The C ABI custom profile must preserve the legacy defaults: raw input, no
  post-CCM downscale, no display-normalisation trick, primitive opsets and no
  fused stages.
- Once those compile errors were removed, the doc tests exposed two pseudo-code
  snippets that rustdoc treated as Rust: an indented C++ excerpt in
  `DisplayBlock::conv_weights` and the `?`/`...` usage example in
  `isp-rectifier/src/lib.rs`. Both are now explicitly `text`/`ignore` blocks.
- On Termux/Android, `ort-sys` still needs `ORT_CACHE_DIR` because its
  `cache_dir_default()` has no Android branch. With a repo-local cache directory
  set, the canonical `--workspace --doc --all-features` command passes; the CI
  Linux host uses its normal `$HOME/.cache` path and needs no override.
