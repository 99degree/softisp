//! Runtime ISP parameter hot-swap via Vulkan const buffer update.
//!
//! Demonstrates live 3A adjustments (gain, bias, CCM) without rebuilding
//! the MNN model. Uses MNNVulkanHotSwapConstBuffer C API.
//!
//! Usage:
//!   LD_LIBRARY_PATH=lib/aarch64-v8a cargo run --example runtime_params -p cam-isp --features mnn -- <model.mnn>

use cam_isp::mnn_sys::{self, MnnBackendType};
use std::ffi::CString;

const BUFFER_LEN: usize = 26;

/// Type alias for preset function: takes params, returns modified params.
type PresetFn = fn([f32; BUFFER_LEN]) -> [f32; BUFFER_LEN];

fn make_default_params() -> [f32; BUFFER_LEN] {
    let mut buf = [0.0f32; BUFFER_LEN];
    buf[0] = 1920.0;
    buf[1] = 1080.0;
    buf[2] = 1.0;
    buf[7] = 1.0;
    buf[8] = 1.0;
    buf[9] = 1.0;
    buf[10] = 1.0; // WB neutral
    buf[11] = 1.0;
    buf[15] = 1.0;
    buf[19] = 1.0; // CCM identity
    buf[20] = 0.5;
    buf[23] = 2.2; // FCS, gamma
    buf
}

fn warm_wb(mut p: [f32; BUFFER_LEN]) -> [f32; BUFFER_LEN] {
    p[7] = 0.7;
    p[9] = 1.3;
    p
}
fn cool_wb(mut p: [f32; BUFFER_LEN]) -> [f32; BUFFER_LEN] {
    p[7] = 1.2;
    p[9] = 0.8;
    p
}

fn main() {
    println!("=== Runtime ISP Parameter Hot-Swap Demo ===\n");

    let model_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/data/local/tmp/test_model.mnn".into());

    unsafe {
        let c_model = CString::new(model_path.as_str()).unwrap();
        let interp = mnn_sys::mnn_interpreter_create_from_file(c_model.as_ptr());
        if interp.is_null() {
            eprintln!("Failed to load: {}", model_path);
            return;
        }
        println!("Loaded: {}", model_path);

        let session = mnn_sys::mnn_session_create(interp, MnnBackendType::Vulkan, 1);
        if session.is_null() {
            eprintln!("Failed to create session");
            mnn_sys::mnn_interpreter_destroy(interp);
            return;
        }
        println!("Session created\n");

        // Initialize
        mnn_sys::mnn_session_run(interp, session);

        let mut params = make_default_params();
        let presets: Vec<(&str, PresetFn)> = vec![
            ("neutral", |p| p),
            ("warm_wb", warm_wb),
            ("cool_wb", cool_wb),
        ];

        println!(
            "{:>12} {:>10} {:>8} {:>8}",
            "Preset", "Swap(ms)", "WB_R", "WB_B"
        );
        println!(
            "{:>12} {:>10} {:>8} {:>8}",
            "------", "-------", "----", "----"
        );

        for iter in 0..3 {
            for (name, apply_fn) in &presets {
                params = apply_fn(params);

                let t0 = std::time::Instant::now();
                mnn_sys::MNNVulkanHotSwapConstBuffer(
                    session as *mut _,
                    1,
                    params.as_ptr() as *const _,
                    (BUFFER_LEN * 4) as i32,
                );
                let swap_ms = t0.elapsed().as_secs_f64() * 1000.0;

                let t1 = std::time::Instant::now();
                mnn_sys::mnn_session_run(interp, session);
                let infer_ms = t1.elapsed().as_secs_f64() * 1000.0;

                println!(
                    "{:>12} {:>7.2}ms {:>8.2} {:>8.2}  (infer {:.2}ms)",
                    name, swap_ms, params[7], params[9], infer_ms
                );
            }
            println!("--- iter {} done ---", iter + 1);
        }

        mnn_sys::mnn_session_release(interp, session);
        mnn_sys::mnn_interpreter_destroy(interp);
        println!("\nDone! {} hot-swaps executed.", 3 * presets.len());
    }
}
