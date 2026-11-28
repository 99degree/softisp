use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    // This is the final cdylib link step, so it needs its own rpath entries:
    // dependency build scripts do not propagate cargo:rustc-link-arg here.
    // $ORIGIN is the APK's install-specific nativeLibraryDir, where MNN's
    // bundled .so files live; it must come before any system path.
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
    // Relative to the private nativeLibraryDir: cover both ABI directory names
    // in case a future APK keeps libs in an ABI subdir instead of flat.
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/lib/arm64-v8a");
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/lib/aarch64-v8a");
    // Termux / source-tree development build.
    println!("cargo:rustc-link-arg=-Wl,-rpath,/data/data/com.termux/files/home/softisp/cam-rust/lib/aarch64-v8a");
    // Android platform fallbacks: system/vendor for liblog/libandroid/
    // libvulkan, and the bionic APEX for libc/libdl/libm on newer releases.
    println!("cargo:rustc-link-arg=-Wl,-rpath,/system/lib64");
    println!("cargo:rustc-link-arg=-Wl,-rpath,/vendor/lib64");
    println!("cargo:rustc-link-arg=-Wl,-rpath,/apex/com.android.runtime/lib64/bionic");
    // Generic Linux runtime fallback.
    println!("cargo:rustc-link-arg=-Wl,-rpath,/usr/lib");
    println!("cargo:rustc-link-arg=-Wl,-rpath,/usr/lib64");
    println!("cargo:rustc-link-arg=-Wl,-rpath,/usr/local/lib");

    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os != "android" {
        eprintln!(
            "[softisp-anw] build: non-Android target ({target_os}), skipping native lib rpath"
        );
        return;
    }
    eprintln!("[softisp-anw] build: main entry for Android native lib (libsoftisp.so)");
    println!("cargo:rustc-link-lib=dylib=camera2ndk");
    // Add rpath for MNN libraries when building tests
    println!("cargo:rustc-link-arg=-Wl,-rpath,/data/data/com.termux/files/home/softisp/cam-rust/lib/aarch64-v8a");
    // Also the alternate arm64-v8a name that build-apk.sh copies to.
    println!("cargo:rustc-link-arg=-Wl,-rpath,/data/data/com.termux/files/home/softisp/cam-rust/lib/arm64-v8a");
    // Device system libraries (libvulkan.so lives here on Android)
    println!("cargo:rustc-link-arg=-Wl,-rpath,/system/lib64");
    println!("cargo:rustc-link-arg=-Wl,-rpath,/apex/com.android.runtime/lib64/bionic");
    // The prebuilt MNN shared objects reference versioned platform symbols
    // (`__android_log_print@LIBLOG`), while the NDK sysroot ships an
    // unversioned liblog stub — so lld refuses to leave them unresolved when
    // this crate links. The device's /system/lib64/liblog.so does define
    // LIBLOG, so the reference resolves at load time.
    //
    // cam-isp/build.rs sets this for cam-isp's own link step, but
    // `cargo:rustc-link-arg` does not propagate from a dependency's build
    // script to the final binary, so the flag has to be repeated here.
    println!("cargo:rustc-link-arg=-Wl,--allow-shlib-undefined");

    build_vulkan_loader();
}

fn build_vulkan_loader() {
    let arch = std::env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
    if arch != "aarch64" && arch != "arm64" {
        eprintln!(
            "[softisp-anw] build: Vulkan loader packaging supports aarch64 only (arch={arch})"
        );
        return;
    }

    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let repo = crate_dir
        .parent()
        .expect("softisp-anw must live in the workspace root");
    let source = repo.join("vulkan_loader/softisp_vulkan_loader.cpp");
    println!("cargo:rerun-if-changed={}", source.display());

    let lib_dirs = [
        repo.join("cam-rust/lib/aarch64-v8a"),
        repo.join("cam-rust/lib/arm64-v8a"),
    ];
    for dir in &lib_dirs {
        if let Err(e) = std::fs::create_dir_all(dir) {
            panic!("cannot create {}: {e}", dir.display());
        }
    }

    let output = lib_dirs[0].join("libvulkan.so");
    let compiler = find_clangpp().expect("NDK clang++ is required to build libvulkan.so");
    let status = Command::new(&compiler)
        .arg("-shared")
        .arg("-fPIC")
        .arg("-std=c++17")
        .arg("-O2")
        .arg("-Wl,-soname,libvulkan.so")
        .arg("-o")
        .arg(&output)
        .arg(&source)
        .status()
        .unwrap_or_else(|e| panic!("failed to run {}: {e}", compiler.display()));
    assert!(
        status.success(),
        "building {} from {} failed",
        output.display(),
        source.display()
    );

    for dir in lib_dirs.iter().skip(1) {
        let target = dir.join("libvulkan.so");
        std::fs::copy(&output, &target).unwrap_or_else(|e| {
            panic!(
                "cannot copy {} to {}: {e}",
                output.display(),
                target.display()
            )
        });
    }

    stage_adreno_hal(&lib_dirs);
    eprintln!("[softisp-anw] build: packaged {}", output.display());
}

fn find_clangpp() -> Option<PathBuf> {
    let ndk = std::env::var_os("ANDROID_NDK_ROOT")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME")
                .map(PathBuf::from)
                .map(|home| home.join("android-sdk/ndk/29.0.14206865"))
                .filter(|path| path.exists())
        })
        .or_else(|| std::env::var_os("ANDROID_NDK_HOME").map(PathBuf::from))?;
    let host = match std::env::consts::OS {
        "macos" => "darwin-x86_64",
        "windows" => "windows-x86_64",
        _ => "linux-x86_64",
    };
    let bin = ndk.join("toolchains/llvm/prebuilt").join(host).join("bin");
    let candidates = [
        bin.join("aarch64-linux-android35-clang++"),
        bin.join("aarch64-linux-android-clang++"),
    ];
    candidates.into_iter().find(|path| path.exists())
}

fn stage_adreno_hal(lib_dirs: &[PathBuf]) {
    let installed = Path::new("/vendor/lib64/hw/vulkan.adreno.so");
    if !installed.exists() {
        for dir in lib_dirs {
            let _ = std::fs::remove_file(dir.join("libsoftisp_vulkan_adreno.so"));
        }
        eprintln!("[softisp-anw] build: no installed Adreno HAL to stage");
        return;
    }
    for dir in lib_dirs {
        let target = dir.join("libsoftisp_vulkan_adreno.so");
        std::fs::copy(installed, &target).unwrap_or_else(|e| {
            panic!(
                "cannot stage {} to {}: {e}",
                installed.display(),
                target.display()
            )
        });
    }
    eprintln!("[softisp-anw] build: staged Adreno HAL into APK runtime libs");
}
