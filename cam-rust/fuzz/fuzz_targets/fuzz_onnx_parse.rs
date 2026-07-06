#![no_main]
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Fuzz ONNX protobuf parsing
    // This tests the safety of parsing untrusted input
    let _ = cam_isp::onnx::proto::parse_from_bytes(data);
});
