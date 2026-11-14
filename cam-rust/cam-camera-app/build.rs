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
}
