//! Build script for isp-rectifier.
//!
//! Links the MNN native libraries (libMNNConvertDeps.so, libmnn_wrapper.a)
//! that are copied into cam-rust/lib/<abi>/ by cam-isp's build.rs.
//!
//! This is needed when the `mnn` feature is enabled so that isp-rectifier's
//! own mnn_sys FFI declarations can resolve libMNNConvertDeps.so symbols
//! (mnn_convert_onnx_to_mnn_buffer) and libmnn_wrapper symbols at link time.

use std::path::{Path, PathBuf};

fn main() {
    #[cfg(feature = "mnn")]
    {
        link_mnn_libs();
    }
}

#[cfg(feature = "mnn")]
fn link_mnn_libs() {
    // The ABI directory where cam-isp's build.rs copied the MNN .so files
    // and static libs.
    let abi_dir = abi_dir();

    // Link search path
    println!("cargo:rustc-link-search=native={}", abi_dir.display());

    // Link libMNNConvertDeps.so (for ONNX→MNN conversion)
    println!("cargo:rustc-link-lib=MNNConvertDeps");

    // Link libmnn_wrapper.a (for MNN interpreter/session/tensor FFI)
    println!("cargo:rustc-link-lib=static=mnn_wrapper");

    // Link libMNN.so (dependency of MNNConvertDeps)
    println!("cargo:rustc-link-lib=MNN");

    // Link libMNN_Vulkan.so (Vulkan backend)
    println!("cargo:rustc-link-lib=MNN_Vulkan");

    // Link libMNN_Express.so
    println!("cargo:rustc-link-lib=MNN_Express");

    // C++ standard library
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os == "android" {
        println!("cargo:rustc-link-lib=c++_shared");
    } else {
        println!("cargo:rustc-link-lib=stdc++");
    }

    // Allow unresolved symbols from shared libs (resolved at load time on device)
    println!("cargo:rustc-link-arg=-Wl,--allow-shlib-undefined");

    // Re-run if the libs change
    for lib in [
        "libMNNConvertDeps.so",
        "libmnn_wrapper.a",
        "libMNN.so",
        "libMNN_Vulkan.so",
        "libMNN_Express.so",
    ] {
        println!("cargo:rerun-if-changed={}", abi_dir.join(lib).display());
    }
}

/// Map Rust target arch to Android ABI directory name.
#[cfg(feature = "mnn")]
fn abi_dir() -> PathBuf {
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("isp-rectifier must live under repo root")
        .join("cam-rust")
        .join("lib");

    let arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();

    let suffix = if os == "android" {
        match arch.as_str() {
            "aarch64" | "arm64" => "aarch64-v8a",
            "arm" | "armv7" => "armeabi-v7a",
            "x86_64" => "x86_64",
            "x86" | "i686" => "x86",
            "riscv64" => "riscv64",
            other => {
                eprintln!(
                    "warning: unknown Android ABI arch '{}', defaulting to aarch64-v8a",
                    other
                );
                "aarch64-v8a"
            }
        }
    } else {
        match arch.as_str() {
            "aarch64" | "arm64" => "aarch64",
            "x86_64" => "x86_64",
            "x86" | "i686" => "x86",
            other => other,
        }
    };

    repo_root.join(suffix)
}
