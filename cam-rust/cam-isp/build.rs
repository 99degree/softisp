//! Build script for cam-isp.
//!
//! Links prebuilt native libraries from the `lib/` directory.
//!   lib/arm64-v8a/libonnxruntime.so  — ONNX Runtime (for `ort` feature)
//!   lib/arm64-v8a/libMNN.so          — MNN inference engine (for `mnn` feature)
//!
//! Set the following environment variables for custom paths:
//!   MNN_INCLUDE_DIR  — path to MNN C++ headers
//!   MNN_LIB_DIR      — path to prebuilt libMNN.so directory
//!   ORT_LIB_DIR      — path to prebuilt libonnxruntime.so directory

fn main() {
    // Library base paths — computed lazily inside feature gates

    // --- ONNX Runtime (ort feature only) ---
    #[cfg(feature = "ort")]
    link_onnxruntime();

    // --- MNN (mnn feature only) ---
    #[cfg(feature = "mnn")]
    link_mnn();
}

#[cfg(feature = "ort")]
fn link_onnxruntime() {
    let abi_dir = compute_abi_dir();
    println!("cargo:rerun-if-changed={}/libonnxruntime.so", abi_dir);
    let ort_lib_dir = std::env::var("ORT_LIB_DIR").unwrap_or_else(|_| abi_dir);
    println!("cargo:rustc-link-search=native={}", ort_lib_dir);
    println!("cargo:rustc-link-lib=onnxruntime");
}

#[cfg(feature = "mnn")]
fn link_mnn() {
    let abi_dir = compute_abi_dir();
    let mnn_include = std::env::var("MNN_INCLUDE_DIR").unwrap_or_else(|_| {
        let home = std::env::var("HOME").unwrap_or_else(|_| "/data/data/com.termux/files/home".to_string());
        format!("{}/MNN/include", home)
    });

    let wrapper_src = std::path::Path::new("mnn_sys/mnn_wrapper.cpp");
    let wrapper_hdr = std::path::Path::new("mnn_sys/mnn_wrapper.h");

    if wrapper_src.exists() && wrapper_hdr.exists() {
        println!("cargo:rerun-if-changed=mnn_sys/mnn_wrapper.cpp");
        println!("cargo:rerun-if-changed=mnn_sys/mnn_wrapper.h");
        
        let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
        
        cc::Build::new()
            .cpp(true)
            .std("c++17")
            .file("mnn_sys/mnn_wrapper.cpp")
            .include(&mnn_include)
            .compile("mnn_wrapper");
        
        // Copy .a to lib/aarch64 so it's findable by ALL targets
        let lib_dir = std::path::Path::new(&abi_dir);
        std::fs::create_dir_all(lib_dir).ok();
        let src = out_dir.join("libmnn_wrapper.a");
        let dst = lib_dir.join("libmnn_wrapper.a");
        if src.exists() {
            std::fs::copy(&src, &dst).ok();
            println!("cargo:rerun-if-changed={}", dst.display());
        }
    }

    // Always link from lib/ directory (works for all targets)
    println!("cargo:rustc-link-search=native={}", abi_dir);
    println!("cargo:rustc-link-lib=static=mnn_wrapper");
    println!("cargo:rustc-link-lib=MNN");
    println!("cargo:rustc-link-lib=c++_shared");
    println!("cargo:rustc-link-lib=MNN_Express");
}

/// Compute the ABI-specific library directory.
#[allow(dead_code)]
fn compute_abi_dir() -> String {
    let lib_dir = std::path::Path::new("lib");
    let target_arch = std::env::var("CARGO_CFG_TARGET_ARCH")
        .unwrap_or_else(|_| "arm64-v8a".to_string());
    format!("{}/{}", lib_dir.display(), target_arch)
}
