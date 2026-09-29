//! Link the camera2 NDK library.
//!
//! `ndk-sys` emits `#[link]` attributes for `libandroid`, `libmediandk` and
//! friends, but the camera2 entry points (`ACameraManager_*`, `ACaptureRequest_*`)
//! live in a separate `libcamera2ndk.so` that it does not declare. Declaring it
//! here keeps the camera FFI in pure Rust — no C shim, no `cc` crate.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os != "android" {
        return;
    }
    println!("cargo:rustc-link-lib=dylib=camera2ndk");
}
