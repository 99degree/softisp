#![no_main]
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Fuzz pipeline serializer deserialization
    // Tests safety of parsing untrusted pipeline configs
    if let Ok(s) = std::str::from_utf8(data) {
        let _ = cam_isp::serializer::PipelineConfig::from_text(s);
    }
});
