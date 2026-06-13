// Build script for Android Camera HAL
//
// This sets up the necessary NDK linking and ensures we're building for Android.

use std::env;
use std::path::PathBuf;

fn main() {
    // Check if we're cross-compiling for Android
    let target = env::var("TARGET").unwrap_or_else(|_| "".to_string());

    if target.contains("android") {
        println!("cargo:rustc-link-lib=log");
        println!("cargo:rustc-link-lib=android");
        println!("cargo:rustc-link-lib=cutils");
        println!("cargo:rustc-link-lib=utils");
        println!("cargo:rustc-link-lib=camera_metadata");
        println!("cargo:rustc-link-lib=camera3");

        // Additional libs needed for Hardware Buffer and GraphicBuffer
        println!("cargo:rustc-link-lib=hardware");
        println!("cargo:rustc-link-lib=hidl");
        println!("cargo:rustc-link-lib=base");
        println!("cargo:rustc-link-lib=gralloc");

        // Tell cargo to link the NDK sysroot
        if let Ok(ndk_home) = env::var("ANDROID_NDK_HOME") {
            let sysroot = PathBuf::from(ndk_home).join("toolchains").join("llvm").join("prebuilt").join("linux-x86_64").join("sysroot");
            if sysroot.exists() {
                println!("cargo:rustc-link-search-native={}", sysroot.display());
            }
        }
    } else {
        // Native build for testing on host - link stubs
        // We'll need to provide mock implementations for the NDK types
        println!("cargo:warning=Building for non-Android target - using stubs");
    }
}
