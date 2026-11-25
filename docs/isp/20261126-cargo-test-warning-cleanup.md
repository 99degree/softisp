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
- `cargo test --workspace --doc --all-features --no-fail-fast` still exits 101
  on the pre-existing `rscam (v4l2) is for linux/freebsd only` error; this is
  unrelated to these edits and is not part of the canonical CI gate.
