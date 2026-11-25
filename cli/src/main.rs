//! Binary entry point for cam-ipa-sk polyglot ELF.
//!
//! Links against cam_ipa_sk lib (rlib) for the binary target.
//! The lib is also built as cdylib for dlopen use.

use cam_ipa_sk::ipa_sk_main;

fn main() {
    std::process::exit(ipa_sk_main());
}