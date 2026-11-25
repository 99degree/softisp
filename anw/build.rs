fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    if target_os != "android" {
        return;
    }
    println!("cargo:rustc-link-lib=dylib=camera2ndk");
    // Add rpath for MNN libraries when building tests
    println!("cargo:rustc-link-arg=-Wl,-rpath,/data/data/com.termux/files/home/softisp/cam-rust/lib/aarch64-v8a");
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
}
