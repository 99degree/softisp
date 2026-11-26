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
        return;
    }
    println!("cargo:rustc-link-lib=dylib=camera2ndk");
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
}
