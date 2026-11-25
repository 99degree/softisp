//! Binary entry point for cam-ipa-test-cli polyglot ELF.
//!
//! The same source compiles as:
//!   - `ipa_test_cli` binary (runnable directly)
//!   - `libcam_ipa_test_cli.so` shared library (dlopen-able, exposes `ipa_test_main`)

use cam_ipa_test_cli::ipa_test_main;

fn main() {
    std::process::exit(ipa_test_main());
}