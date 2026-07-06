#![no_main]
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Fuzz pipeline validation with untrusted block names
    // Tests safety of validating pipelines with arbitrary block IDs
    if let Ok(s) = std::str::from_utf8(data) {
        // Create a dummy pipeline config and validate it
        let config = cam_isp::serializer::PipelineConfig {
            width: 1920,
            height: 1080,
            blocks: vec![s.to_string()],
            ..Default::default()
        };
        let _ = config.validate();
    }
});
