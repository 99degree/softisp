//! Compare the old temp-file CLI conversion (mnn_convert_onnx_buffer) with
//! the new pure in-memory API (mnn_convert_onnx_to_mnn_buffer) on the same
//! MED-profile graph — the outputs must be byte-identical.
//!
//! Run on device:
//!   cargo run -p cam-isp --example probe_convert_eq --features "rectifier mnn" --target aarch64-linux-android
use cam_isp::mnn_converter::{convert_onnx_buffer, dump_mnn_to_json};
use cam_isp::mnn_sys::{
    mnn_convert_onnx_buffer as cli_convert, mnn_convert_onnx_to_mnn_buffer, MnnConvertBufferResult,
    MnnConvert_FreeBuffer,
};
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;

fn extract_op_types(json: &str) -> Vec<String> {
    let val: serde_json::Value = serde_json::from_str(json).expect("parse json");
    let oplists = val
        .get("oplists")
        .and_then(|o| o.as_array())
        .cloned()
        .unwrap_or_default();
    let mut types = Vec::new();
    for op in oplists.iter() {
        let ty = op.get("type").and_then(|t| t.as_str()).unwrap_or("?");
        if ty == "Extra" {
            let extra = op
                .get("main")
                .and_then(|m| m.get("type"))
                .and_then(|t| t.as_str())
                .unwrap_or("Extra");
            types.push(extra.to_string());
        } else {
            types.push(ty.to_string());
        }
    }
    types
}

fn main() {
    let mut blocks = PipelineProfile::MED.build_blocks(32, 0);
    GraphComposer::wire_blocks(&mut blocks);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 21).expect("compose");
    println!("ONNX bytes: {}", onnx.len());

    // Old path: temp-file CLI conversion.
    let mut result = MnnConvertBufferResult {
        success: 0,
        error_msg: [0 as std::os::raw::c_char; 1024usize],
        data: std::ptr::null_mut(),
        size: 0,
    };
    unsafe {
        cli_convert(
            onnx.as_ptr() as *const std::ffi::c_void,
            onnx.len(),
            &mut result,
        );
    }
    if result.success != 0 {
        let msg = unsafe { std::ffi::CStr::from_ptr(result.error_msg.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        panic!("CLI convert failed: {}", msg);
    }
    let cli_bytes =
        unsafe { std::slice::from_raw_parts(result.data as *const u8, result.size) }.to_vec();
    unsafe { MnnConvert_FreeBuffer(&mut result) };

    // New path: pure in-memory API.
    let buf_bytes = convert_onnx_buffer(&onnx).expect("buffer convert");

    println!(
        "CLI bytes: {}  BUFFER bytes: {}  identical: {}",
        cli_bytes.len(),
        buf_bytes.len(),
        cli_bytes == buf_bytes
    );

    let cli_json = dump_mnn_to_json(&cli_bytes).expect("cli json");
    let buf_json = dump_mnn_to_json(&buf_bytes).expect("buf json");
    std::fs::write("cli.json", cli_json.as_bytes()).ok();
    std::fs::write("buf.json", buf_json.as_bytes()).ok();
    let cli_ops = extract_op_types(&cli_json);
    let buf_ops = extract_op_types(&buf_json);
    println!("CLI ops: {}", cli_ops.len());
    println!("BUF ops: {}", buf_ops.len());
    for (i, (a, b)) in cli_ops.iter().zip(buf_ops.iter()).enumerate() {
        if a != b {
            println!("  first divergence at [{}]: CLI={} BUF={}", i, a, b);
            break;
        }
    }
    if cli_ops.len() != buf_ops.len() {
        println!(
            "  length differs; tail CLI: {:?}",
            &cli_ops[cli_ops.len().min(buf_ops.len())..]
        );
    }
}
