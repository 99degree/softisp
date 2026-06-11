//! Build script for cam-isp.
//!
//! If the MNN wrapper sources exist and a C++ compiler is available,
//! compiles `mnn_sys/mnn_wrapper.cpp` into a static library and
//! links it together with `libMNN.so`.
//!
//! Set the following environment variables for cross-compilation:
//!   MNN_INCLUDE_DIR  — path to MNN C++ headers (default: /data/data/.../MNN/include)
//!   MNN_LIB_DIR     — path to prebuilt libMNN.so directory

fn main() {
    // Check if MNN wrapper should be compiled
    let mnn_include = std::env::var("MNN_INCLUDE_DIR").unwrap_or_else(|_| {
        // Default: look for the MNN headers in the user's home
        let home = std::env::var("HOME").unwrap_or_else(|_| "/data/data/com.termux/files/home".to_string());
        format!("{}/MNN/include", home)
    });

    let wrapper_src = std::path::Path::new("mnn_sys/mnn_wrapper.cpp");
    let wrapper_hdr = std::path::Path::new("mnn_sys/mnn_wrapper.h");

    if wrapper_src.exists() && wrapper_hdr.exists() {
        println!("cargo:rerun-if-changed=mnn_sys/mnn_wrapper.cpp");
        println!("cargo:rerun-if-changed=mnn_sys/mnn_wrapper.h");

        // Try to compile the C++ wrapper using `cc` crate
        #[cfg(feature = "mnn")]
        {
            let mut build = cc::Build::new();
            build
                .cpp(true)
                .std("c++17")
                .file("mnn_sys/mnn_wrapper.cpp")
                .include(&mnn_include)
                .compile("mnn_wrapper");

            // Link MNN shared library
            if let Ok(lib_dir) = std::env::var("MNN_LIB_DIR") {
                println!("cargo:rustc-link-search=native={}", lib_dir);
            }
            println!("cargo:rustc-link-lib=MNN");
        }
    }

    // Generate bindgen bindings (optional)
    // Uncomment when bindgen is available:
    // let bindings = bindgen::Builder::default()
    //     .header("mnn_sys/mnn_wrapper.h")
    //     .generate()
    //     .expect("Unable to generate MNN bindings");
    // bindings
    //     .write_to_file("src/mnn_sys.rs")
    //     .expect("Couldn't write MNN bindings");
}
