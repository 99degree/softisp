//! Build script for cam-isp.
//!
//! Native libraries are stored in a single canonical location:
//!   repo-root/lib/aarch64-v8a/
//!
//! Include priority:
//!   1. $HOME/MNN (or MNN_DIR) when present — always used for the latest headers.
//!   2. cam-isp/vendor/mnn — checked-in fallback headers copied from ~/MNN.
//!
//! Override with:
//!   MNN_DIR           — MNN source/build root (default: $HOME/MNN)
//!   MNN_INCLUDE_DIR   — MNN include directory (default: $MNN_DIR/include)
//!   MNN_LIB_DIR       — libMNN.so build directory (default: $MNN_DIR/build_vk/OFF)
//!   MNN_CONVERT_DIR   — libMNNConvertDeps.so build directory (default: $MNN_DIR/build_vk/tools/converter/OFF)
//!   MNN_CONVERT_INCLUDE_DIR — converter headers (default: $MNN_DIR/tools/converter/include)
//!   MNN_SCHEMA_DIR    — generated flatbuffer headers (default: $MNN_DIR/schema/current)
//!   ORT_LIB_DIR       — path to prebuilt libonnxruntime.so directory

fn main() {
    #[cfg(feature = "ort")]
    link_onnxruntime();

    #[cfg(feature = "mnn")]
    link_mnn();

    #[cfg(feature = "mnn")]
    link_mnnconvert();
}

#[cfg(feature = "ort")]
fn link_onnxruntime() {
    let abi_dir = abi_dir();
    println!("cargo:rerun-if-changed={}/libonnxruntime.so", abi_dir.display());
    let ort_lib_dir = std::env::var_os("ORT_LIB_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| abi_dir.clone());
    println!("cargo:rustc-link-search=native={}", ort_lib_dir.display());
    println!("cargo:rustc-link-lib=onnxruntime");
}

#[cfg(feature = "mnn")]
fn link_mnn() {
    let abi_dir = abi_dir();
    let mnn_include = mnn_include_dir();

    copy_if_newer(&mnn_lib_src().join("libMNN.so"), &abi_dir.join("libMNN.so"));

    let wrapper_src = Path::new("mnn_sys/mnn_wrapper.cpp");
    let wrapper_hdr = Path::new("mnn_sys/mnn_wrapper.h");

    if wrapper_src.exists() && wrapper_hdr.exists() {
        println!("cargo:rerun-if-changed=mnn_sys/mnn_wrapper.cpp");
        println!("cargo:rerun-if-changed=mnn_sys/mnn_wrapper.h");

        let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
        cc::Build::new()
            .cpp(true)
            .std("c++17")
            .file(wrapper_src)
            .include(&mnn_include)
            .compile("mnn_wrapper");

        std::fs::create_dir_all(&abi_dir).ok();
        let src = out_dir.join("libmnn_wrapper.a");
        let dst = abi_dir.join("libmnn_wrapper.a");
        if src.exists() {
            std::fs::copy(&src, &dst).ok();
            println!("cargo:rerun-if-changed={}", dst.display());
        }
    }

    println!("cargo:rustc-link-search=native={}", abi_dir.display());
    println!("cargo:rustc-link-lib=static=mnn_wrapper");
    println!("cargo:rustc-link-lib=MNN");
    println!("cargo:rustc-link-lib=c++_shared");
}

#[cfg(feature = "mnn")]
fn link_mnnconvert() {
    let abi_dir = abi_dir();
    let mnn_include = mnn_include_dir();
    let mnnconvert_include = mnn_convert_include_dir();
    let mnn_schema_include = mnn_schema_dir();

    copy_if_newer(
        &mnn_convert_lib_src().join("libMNNConvertDeps.so"),
        &abi_dir.join("libMNNConvertDeps.so"),
    );

    let src_dir = Path::new("vendor/mnn/mnnconvert/source");
    if !src_dir.join("mnnconvert_shared.cpp").exists() {
        panic!("missing mnnconvert_shared.cpp at {}", src_dir.join("mnnconvert_shared.cpp").display());
    }

    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .file(src_dir.join("mnnconvert_shared.cpp"))
        .file("mnn_sys/mnn_convert_api.cpp")
        .include(&mnn_include)
        .include(&mnnconvert_include)
        .include(&mnn_schema_include)
        .include("vendor/mnn/mnnconvert/include")
        .define("MNN_CONVERT_API_EXPORTS", None)
        .compile("mnnconvert");

    std::fs::create_dir_all(&abi_dir).ok();

    println!("cargo:rustc-link-search=native={}", abi_dir.display());
    println!("cargo:rustc-link-lib=mnnconvert");
    println!("cargo:rustc-link-lib=MNNConvertDeps");
}

#[allow(dead_code)]
fn abi_dir() -> PathBuf {
    repo_root().join("lib").join(abi_suffix())
}

#[allow(dead_code)]
fn abi_suffix() -> String {
    match std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default().as_str() {
        "aarch64" | "arm64" => "aarch64-v8a".to_string(),
        other if !other.is_empty() => other.to_string(),
        _ => "aarch64-v8a".to_string(),
    }
}

#[allow(dead_code)]
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("cam-isp must live under repo root")
        .to_path_buf()
}

#[allow(dead_code)]
fn mnn_dir() -> PathBuf {
    if let Ok(path) = std::env::var("MNN_DIR") {
        PathBuf::from(path)
    } else {
        let home = std::env::var("HOME").expect("HOME must be set");
        PathBuf::from(home).join("MNN")
    }
}

#[allow(dead_code)]
fn vendor_mnn_dir() -> PathBuf {
    Path::new("vendor/mnn").to_path_buf()
}

#[allow(dead_code)]
fn mnn_include_dir() -> PathBuf {
    std::env::var_os("MNN_INCLUDE_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            let primary = mnn_dir().join("include");
            if primary.exists() {
                primary
            } else {
                vendor_mnn_dir().join("include")
            }
        })
}

#[allow(dead_code)]
fn mnn_convert_include_dir() -> PathBuf {
    std::env::var_os("MNN_CONVERT_INCLUDE_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            let primary = mnn_dir().join("tools/converter/include");
            if primary.exists() {
                primary
            } else {
                vendor_mnn_dir().join("tools/converter/include")
            }
        })
}

#[allow(dead_code)]
fn mnn_schema_dir() -> PathBuf {
    std::env::var_os("MNN_SCHEMA_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            let primary = mnn_dir().join("schema/current");
            if primary.exists() {
                primary
            } else {
                vendor_mnn_dir().join("schema/current")
            }
        })
}

#[allow(dead_code)]
fn mnn_lib_src() -> PathBuf {
    std::env::var_os("MNN_LIB_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| mnn_dir().join("build_vk/OFF"))
}

#[allow(dead_code)]
fn mnn_convert_lib_src() -> PathBuf {
    std::env::var_os("MNN_CONVERT_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| mnn_dir().join("build_vk/tools/converter/OFF"))
}

#[allow(dead_code)]
fn copy_if_newer(src: &Path, dst: &Path) {
    if !src.exists() {
        eprintln!("warning: MNN artifact not found, leaving existing copy: {}", src.display());
        return;
    }
    if dst.exists() {
        let src_time = src.metadata().and_then(|m| m.modified()).ok();
        let dst_time = dst.metadata().and_then(|m| m.modified()).ok();
        if let (Some(src_time), Some(dst_time)) = (src_time, dst_time) {
            if src_time <= dst_time {
                return;
            }
        }
    }
    if let Some(parent) = dst.parent() {
        std::fs::create_dir_all(parent).ok();
    }
    std::fs::copy(src, dst).ok();
    println!("cargo:rerun-if-changed={}", src.display());
}

use std::path::{Path, PathBuf};
