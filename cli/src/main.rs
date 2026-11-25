//! Binary entry point for cam-ipa-test-cli polyglot ELF.
//!
//! Links against cam_ipa_test_cli lib (rlib) for the binary target.
//! The lib is also built as cdylib for dlopen use.

use cam_ipa_test_cli::ipa_test_main;

fn main() {
    std::process::exit(ipa_test_main());
}