//! Build script for polyglot IPA test CLI.
//!
//! Produces an ELF that is both:
//!   - executable directly (has PT_INTERP + entry point)
//!   - dlopen-able as a shared library (exports `ipa_test_main`)
//!
//! Technique:
//!   - Build as cdylib (ET_DYN)
//!   - Custom entry point via -e linker flag pointing to `my_entry`
//!   - `.interp` section injected via linker script pointing to /system/bin/linker64
//!
//! Note: True polyglot (single .so that runs directly) requires the .interp
//! section which is platform-specific. On Android/Termux this is
//! /system/bin/linker64. The crate also builds a regular [[bin]] target
//! for straightforward execution.

fn main() {
    // Set custom entry point so the ELF can be executed directly
    println!("cargo:rustc-link-arg=-e");
    println!("cargo:rustc-link-arg=my_entry");

    // Tell cargo to rerun if linker flags change
    println!("cargo:rerun-if-changed=build.rs");
}