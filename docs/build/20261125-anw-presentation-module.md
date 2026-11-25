---
title: "Restore the ANativeWindow presentation module"
created_at: 2026-11-25T00:00:00+00:00
status: implemented (rebased onto the upstream presentation module)
---

# Restore the ANativeWindow presentation module

## Symptom

`cargo fmt --all -- --check` and `cargo check -p softisp-anw` both fail. The
formatter reports:

```
Error writing files: failed to resolve mod `presentation`:
anw/src/presentation.rs does not exist
```

The compiler adds `E0583` for the missing module and two "method call on a
diverging inference variable" errors at the `window_target.present(...)` calls.

## Reproduction

```bash
cargo fmt --all -- --check
cargo check -p softisp-anw
```

## Root cause

The upstream force-update replaced the old tree with a combined IPA/CLI history.
Its `anw/src/lib.rs` declares `mod presentation;` and calls
`WindowTarget::new(...)` / `WindowTarget::present(...)`, but no
`anw/src/presentation.rs` was ever committed on any branch. The crate therefore
never compiles, and rustfmt cannot traverse the workspace to apply or check
formatting. `git log --all -- anw/src/presentation.rs` is empty.

The implementation the commit says was "split from android-app preview.rs"
still exists verbatim in `android-app/src/preview.rs`: `WindowTarget`,
`Viewport::fit`, the BGRA blit, and `paint_bars`. A later upstream commit added
`anw/src/presentation.rs` independently; while resynchronising, that version was
kept and extended with the five `Viewport` tests from the app preview.

## Fix applied

The two raw-pointer FFI entry points now acquire a refcounted `NativeWindow`
with `clone_from_ptr` before constructing `WindowTarget`, and `Drop` releases
that reference. `softisp_anw_present` wraps the caller's `*const u8` in a
length-checked slice before calling `present()`. Unused imports and the unused
context binding were removed, and `presentation` is public so its API is
reachable from the cdylib.

No behaviour is invented: the presentation geometry, letterboxing, and BGRA→RGBA
channel order stay identical to the existing camera app preview.

## Verification

- `cargo check -p softisp-anw`: clean (only the pre-existing manifest warning for
  the unused android-target `android_logger` dependency).
- `cargo clippy -p softisp-anw --lib --no-deps -- -D warnings`: clean. The
  `-D warnings` run without `--no-deps` still reports pre-existing `cam-ipa`
  lints from the upstream force-update; those are outside this fix.
- `cargo test -p softisp-anw --lib`: 7 passed, 0 failed.
- `cargo fmt --all -- --check`: clean after the separate formatting-only commit.
