#![no_main]
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Fuzz profile parsing
    // Tests safety of parsing untrusted profile data
    if let Ok(s) = std::str::from_utf8(data) {
        // Try to parse as performance tier
        let _ = s.parse::<cam_isp::optimizer::PerfTier>();
    }
});
