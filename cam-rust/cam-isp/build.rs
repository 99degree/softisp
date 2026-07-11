//! Build script for cam-isp.
//!
//! Native libraries are stored in a single canonical location:
//!   repo-root/lib/<abi>/
//!
//! Supported ABIs:
//!   - aarch64-v8a   (Android arm64)
//!   - armeabi-v7a   (Android arm32)
//!   - x86_64        (Android x86_64)
//!   - x86            (Android x86)
//!   - aarch64       (Linux aarch64)
//!   - x86_64-linux  (Linux x86_64)
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
//!   ANDROID_NDK_HOME  — Android NDK root (auto-detected from standard paths)

use std::path::{Path, PathBuf};

fn main() {
    // Emit rerun-if-changed for all env vars that affect the build
    println!("cargo:rerun-if-env-changed=MNN_DIR");
    println!("cargo:rerun-if-env-changed=MNN_INCLUDE_DIR");
    println!("cargo:rerun-if-env-changed=MNN_LIB_DIR");
    println!("cargo:rerun-if-env-changed=MNN_CONVERT_DIR");
    println!("cargo:rerun-if-env-changed=ANDROID_NDK_HOME");

    #[cfg(feature = "ort")]
    link_onnxruntime();

    #[cfg(feature = "mnn")]
    link_mnn();

    // Note: static linker for MNNConvertDeps was removed.
    // Converter now runs as a subprocess (MNNConvert binary)
    // to isolate MNN's C++ global state from the inference runtime.
    // See cam-isp/src/mnn_converter.rs for the subprocess implementation.

    // Emit NDK linker flags for Android targets
    setup_ndk_linker();
}

// ═══════════════════════════════════════════════════════════════════
//  Android NDK linker setup
// ═══════════════════════════════════════════════════════════════════

/// Emit `cargo:rustc-link-lib` and `cargo:rustc-link-search` for the NDK
/// sysroot libraries when cross-compiling for Android. This ensures the
/// linker can find libc++, liblog, libandroid, etc.
fn setup_ndk_linker() {
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os != "android" {
        return;
    }

    let ndk = match find_ndk() {
        Some(p) => p,
        None => {
            eprintln!("warning: Android NDK not found; linking may fail");
            return;
        }
    };

    let abi = abi_suffix();
    let arch = target_ndk_arch();
    let api_level = ndk_api_level();

    // NDK r26+ uses unified toolchain under <ndk>/toolchains/llvm/prebuilt/<host>/
    let host = ndk_host();
    let toolchain = ndk.join("toolchains").join("llvm").join("prebuilt").join(host);
    let sysroot = toolchain.join("sysroot");

    if sysroot.exists() {
        // Tell cc crate about the sysroot
        println!("cargo:rustc-link-search=native={}", sysroot.join("usr").join("lib").join(format!("{}-linux-android", arch)).display());
        println!("cargo:rustc-link-search=native={}", sysroot.join("usr").join("lib").display());
    }

    // Link against NDK system libraries
    println!("cargo:rustc-link-lib=log");
    println!("cargo:rustc-link-lib=android");
    println!("cargo:rustc-link-lib=c++_shared");
    println!("cargo:rustc-link-lib=mediandk");

    eprintln!("NDK: abi={}, arch={}, api={}, sysroot={}", abi, arch, api_level, sysroot.display());
}

/// Find the Android NDK root directory.
fn find_ndk() -> Option<PathBuf> {
    // 1. Explicit env var
    if let Ok(p) = std::env::var("ANDROID_NDK_HOME") {
        let pb = PathBuf::from(p);
        if pb.exists() { return Some(pb); }
    }
    if let Ok(p) = std::env::var("NDK_HOME") {
        let pb = PathBuf::from(p);
        if pb.exists() { return Some(pb); }
    }

    // 2. Standard Android SDK/NDK locations
    let home = std::env::var("HOME").ok()?;
    let home = PathBuf::from(home);

    // Android Studio default: ~/Android/Sdk/ndk/<version>/
    let sdk = home.join("Android").join("Sdk").join("ndk");
    if sdk.exists() {
        // Pick the latest version directory
        let mut versions: Vec<PathBuf> = std::fs::read_dir(&sdk)
            .ok()?
            .filter_map(|e| e.ok())
            .filter(|e| e.file_type().map(|t| t.is_dir()).unwrap_or(false))
            .map(|e| e.path())
            .collect();
        versions.sort();
        if let Some(latest) = versions.last() {
            return Some(latest.clone());
        }
    }

    // 3. Check if ndk-build is on PATH and infer NDK location
    if let Ok(output) = std::process::Command::new("which").arg("ndk-build").output() {
        let path = String::from_utf8_lossy(&output.stdout).trim().to_string();
        // ndk-build lives in <ndk>/ndk-build
        let pb = PathBuf::from(&path).parent()?.to_path_buf();
        if pb.exists() { return Some(pb); }
    }

    // 4. ~/android-ndk (common manual install)
    let manual = home.join("android-ndk");
    if manual.exists() { return Some(manual); }

    None
}

/// Map CARGO_CFG_TARGET_ARCH to NDK arch name.
fn target_ndk_arch() -> &'static str {
    match std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default().as_str() {
        "aarch64" | "arm64" => "aarch64",
        "arm" | "armv7" => "arm",
        "x86_64" => "x86_64",
        "x86" | "i686" => "i686",
        "riscv64" => "riscv64",
        _ => "aarch64",
    }
}

/// Target triple prefix used by NDK clang (e.g., "aarch64-linux-android").
#[allow(dead_code)]
fn target_ndk_triple() -> String {
    match std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default().as_str() {
        "aarch64" | "arm64" => "aarch64-linux-android".to_string(),
        "arm" | "armv7" => "armv7a-linux-androideabi".to_string(),
        "x86_64" => "x86_64-linux-android".to_string(),
        "x86" | "i686" => "i686-linux-android".to_string(),
        "riscv64" => "riscv64-linux-android".to_string(),
        _ => "aarch64-linux-android".to_string(),
    }
}

/// NDK API level (minimum Android version). Default 24 (Android 7.0).
fn ndk_api_level() -> u32 {
    std::env::var("ANDROID_API_LEVEL")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(24)
}

/// Host platform directory name inside NDK prebuilt/.
fn ndk_host() -> &'static str {
    match std::env::var("CARGO_CFG_HOST_TRIPLE").unwrap_or_default().as_str() {
        t if t.contains("linux") => "linux-x86_64",
        t if t.contains("darwin") || t.contains("macos") => "darwin-x86_64",
        t if t.contains("windows") => "windows-x86_64",
        _ => "linux-x86_64",
    }
}

// ═══════════════════════════════════════════════════════════════════
//  ABI directory mapping
// ═══════════════════════════════════════════════════════════════════

#[allow(dead_code)]
fn abi_dir() -> PathBuf {
    repo_root().join("lib").join(abi_suffix())
}

/// Map Rust target arch to Android/Linux ABI directory name.
///
/// Android NDK convention:
///   aarch64     → arm64-v8a
///   arm         → armeabi-v7a
///   x86_64     → x86_64
///   x86 / i686 → x86
///   riscv64    → riscv64
///
/// Linux (non-Android):
///   aarch64     → aarch64
///   x86_64     → x86_64
#[allow(dead_code)]
fn abi_suffix() -> String {
    let arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    let os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();

    if os == "android" {
        match arch.as_str() {
            "aarch64" | "arm64" => "arm64-v8a".to_string(),
            "arm" | "armv7" => "armeabi-v7a".to_string(),
            "x86_64" => "x86_64".to_string(),
            "x86" | "i686" => "x86".to_string(),
            "riscv64" => "riscv64".to_string(),
            other => {
                eprintln!("warning: unknown Android ABI arch '{}', defaulting to arm64-v8a", other);
                "arm64-v8a".to_string()
            }
        }
    } else {
        // Linux / macOS / other
        match arch.as_str() {
            "aarch64" | "arm64" => "aarch64".to_string(),
            "x86_64" => "x86_64".to_string(),
            "x86" | "i686" => "x86".to_string(),
            other => other.to_string(),
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
//  MNN / converter linking
// ═══════════════════════════════════════════════════════════════════

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
        let mut build = cc::Build::new();
        build.cpp(true)
            .std("c++17")
            .file(wrapper_src)
            .include(&mnn_include);

        // Android NDK cross-compilation: set compiler flags
        setup_cc_for_android(&mut build);

        build.compile("mnn_wrapper");

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
    println!("cargo:rustc-link-lib=MNN_Vulkan");
    println!("cargo:rustc-link-lib=c++_shared");
    // Embed rpath so binary finds libMNN.so at runtime
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", abi_dir.display());
}

/// Link MNNConvertDeps (model conversion support).
/// Only needed when ONNX-to-MNN conversion at runtime is required.
/// Skipped for inference-only builds to avoid libMNN_Express dependency.
#[cfg(feature = "mnn")]
fn link_mnnconvert() {
    // Inference-only: converter linking is not needed.
    // Enable by defining MNN_CONVERT_DIR or adding 'mnn_convert' feature.
    eprintln!("info: MNN converter linking skipped (not needed for inference)");
}

// ═══════════════════════════════════════════════════════════════════
//  cc::Build Android cross-compilation setup
// ═══════════════════════════════════════════════════════════════════

/// Configure cc::Build for Android NDK cross-compilation.
/// Sets the target, sysroot, and compiler flags.
#[allow(dead_code)]
fn setup_cc_for_android(build: &mut cc::Build) {
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os != "android" {
        return;
    }

    let ndk = match find_ndk() {
        Some(p) => p,
        None => {
            eprintln!("warning: NDK not found for cc::Build setup");
            return;
        }
    };

    let host = ndk_host();
    let toolchain = ndk.join("toolchains").join("llvm").join("prebuilt").join(host);
    let triple = target_ndk_triple();
    let api = ndk_api_level();

    // Use NDK's clang as the C++ compiler
    let clangpp = toolchain.join("bin").join(format!("{}{}-clang++", triple, api));
    if clangpp.exists() {
        build.compiler(&clangpp);
        eprintln!("cc::Build compiler: {}", clangpp.display());
    } else {
        // Fallback: try without API level suffix (NDK r23+)
        let clangpp_fallback = toolchain.join("bin").join(format!("{}-clang++", triple));
        if clangpp_fallback.exists() {
            build.compiler(&clangpp_fallback);
            eprintln!("cc::Build compiler (fallback): {}", clangpp_fallback.display());
        }
    }

    // Set target for cc crate
    let target_triple = std::env::var("TARGET").unwrap_or_default();
    build.target(&target_triple);

    // Sysroot
    let sysroot = toolchain.join("sysroot");
    if sysroot.exists() {
        build.flag(format!("--sysroot={}", sysroot.display()));
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Path helpers
// ═══════════════════════════════════════════════════════════════════

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
