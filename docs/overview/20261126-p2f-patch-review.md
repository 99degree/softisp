# P2f local-fixes patch review (2026-11-26)

## Scope

The strict three-day branch audit (`20261125-local-branch-audit.md`) found five
commits on `local-fixes` with no upstream-equivalent patch. Each was reviewed
against the current renamed workspace before any change was made. No patch was
blindly cherry-picked: the app crate was renamed to `anw`, `android-app/` is now
resource-only, and several files moved since those commits were written.

## Decisions

| Commit | Patch | Decision | Reason |
| --- | --- | --- | --- |
| `04e87a9e` | Build `MNNConvert` CLI in `scripts/build_mnn_libs.sh` | Skip | Runtime conversion uses the in-process `libMNNConvertDeps.so` buffer API. The fallback's binary search does not include the patch's `build_isp/MNNConvert` path, so the added `make` step would lengthen every APK build without fixing a production path. `build_isp` remains the only MNN build directory. |
| `ea139531` | MNN-gated cleanup in `mnnengine.rs` | Superseded | The affected session/output-buffer code has been replaced by `SessionPool` and explicit zero-copy metadata; the old `mut`/cast lines no longer exist. |
| `a796111e` | `runtime_libs = "../cam-rust/lib"` | Already applied | P2g moved the manifest to `anw/Cargo.toml`, which already carries the correct one-level-up path. |
| `72ad0292` | `cam-app` → `cam-disp` / `android-app` docs | Apply where still stale | Root `README.md`, `CLAUDE.md` and the APK docs are already current. `cam-rust/README.md` still lists `cam-app`, and `docs/isp/PROFILE_BLOCK_COMPARISON.md` still has a `cam-app` invocation section. Both are corrected here with the current `anw` entry point. |
| `2b796345` | Platform theme and version-qualified adaptive icons | Skip | `themes.xml` already uses a platform (non-AppCompat) theme. The current per-density adaptive-icon XML is part of the APK that the user installed and launched successfully; moving it would be unrelated churn with no demonstrated benefit. |

## Changes made

- `cam-rust/README.md`: replace the stale `cam-app` crate row with `cam-disp`.
- `docs/isp/PROFILE_BLOCK_COMPARISON.md`: mark the `cam-app` command as
  historical and name `anw` as the current application owner.
- `docs/overview/ROADMAP.md`: close P2f with these review decisions.

## Verification

Documentation-only changes; no Rust code, manifest, or build script changed.
`git diff --check` is clean. The preceding tree-fix commit `ccffb4b6` passed the
full local verification set (`--locked` fmt, workspace check, clippy loop, lib
tests, and real-MNN lib tests).
