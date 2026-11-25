# APK build failure — missing `jni` workspace member

status: resolved

## Symptom

`./build-apk.sh` built MNN successfully, then `cargo apk2 build -p cam-camera-app`
failed before compiling the app:

```text
error: failed to load manifest for workspace member `/.../softisp/jni`
Caused by:
  failed to read `/.../softisp/jni/Cargo.toml`
  No such file or directory (os error 2)
```

A second, non-fatal warning appeared after the manifest was repaired:

```text
warning: profiles for the non root package will be ignored, specify profiles at
the workspace root: package: cli/Cargo.toml
```

## Reproduction

```bash
./build-apk.sh
```

## Root cause

`Cargo.toml` listed `jni` as a workspace member, but no `jni/` directory or
`jni/Cargo.toml` exists in the repository. The member was added by the cam-IPA
multi-frontend commit even though the path is covered by the `jni/` Android
build-artifact ignore rule and never became a tracked crate. Cargo validates
every workspace member before building a single package, so the absent
manifest blocked all `cargo apk2 build` invocations regardless of the selected
`-p cam-camera-app`.

`cli/Cargo.toml` also redeclared `[profile.dev]` and `[profile.release]`. Cargo
honours profiles only at the workspace root, where the same settings already
exist, so those sections were inert and produced a warning on every build.

## Fix applied

- Removed the stale `jni` workspace member. Cargo pruned the orphaned
  `softisp_jni` / `jni 0.21.1` chain from `Cargo.lock`; no source referenced it.
- Removed the ignored profile sections from `cli/Cargo.toml`.
- No APK source, UI, or MNN build-script change was made. MNN was still built
  only through `scripts/build_mnn_libs.sh arm64-v8a` / `build_isp`.

## Findings

The absent member blocked the build before any Rust package was compiled; the
profile warning came from the same recently added CLI/IPA workspace changes.
Both were workspace-manifest issues, not ISP or camera code defects.

## Verification

```text
./build-apk.sh                                  success, no cargo warnings
cargo metadata --no-deps --format-version 1     clean
APK: target/debug/apk/softisp-camera.apk        202 MB
SHA-256: c8e4f930082bfaaf47eb0b9faa0c69c84f2747f430323c38cfb4ad98450140a0
```

APK verification and debug signing both succeeded. The APK was **not**
installed; the user handles installation and launch manually.
