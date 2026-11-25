//! Diagnostic: figure out actual input interface of pre-trained ONNX models.
//!
//! Run with:
//!   cargo run -p isp-rectifier --example inspect_models --features onnx-runtime

use std::path::PathBuf;
use tract_onnx::prelude::*;

type TractModel = SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>;

fn try_run(runnable: &TractModel, hist: &[f32], meta: &[f32]) -> String {
    let hist_t = Tensor::from_shape(&[1, hist.len()], hist).unwrap();
    let meta_t = Tensor::from_shape(&[1, meta.len()], meta).unwrap();
    match runnable.run(tvec![hist_t.into(), meta_t.into()]) {
        Ok(outputs) => {
            let shapes: Vec<String> = outputs.iter().map(|o| format!("{:?}", o.shape())).collect();
            format!("OK, {} outputs: {:?}", outputs.len(), shapes)
        }
        Err(e) => format!("FAILED: {}", e),
    }
}

fn try_run_single(runnable: &TractModel, hist: &[f32]) -> String {
    let ht = Tensor::from_shape(&[1, hist.len()], hist).unwrap();
    match runnable.run(tvec![ht.into()]) {
        Ok(o) => format!("OK, {} outputs", o.len()),
        Err(e) => format!("FAILED: {}", e),
    }
}

fn main() {
    let models_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("models");

    for model_name in ["fusedispcontroller_light.onnx", "fusedispcontroller_light_int8.onnx"] {
        let path = models_dir.join(model_name);
        if !path.exists() {
            println!("SKIP: {} not found", model_name);
            continue;
        }
        let size = std::fs::metadata(&path).unwrap().len();
        println!("\n=== {} ({} bytes) ===", model_name, size);

        match tract_onnx::onnx().model_for_path(&path) {
            Ok(model) => {
                match model.into_optimized() {
                    Ok(opt) => {
                        match opt.into_runnable() {
                            Ok(runnable) => {
                                let hist256 = vec![0.5f32; 256];
                                let meta11 = vec![0.5f32; 11];
                                let meta52 = vec![0.5f32; 52];

                                println!("  [256]+[11]:  {}", try_run(&runnable, &hist256, &meta11));
                                println!("  [256]+[52]:  {}", try_run(&runnable, &hist256, &meta52));
                                let hist267 = vec![0.5f32; 267];
                                println!("  [267]:       {}", try_run_single(&runnable, &hist267));
                            }
                            Err(e) => println!("  into_runnable FAILED: {}", e),
                        }
                    }
                    Err(e) => println!("  into_optimized FAILED: {}", e),
                }
            }
            Err(e) => println!("  model_for_path FAILED: {}", e),
        }
    }
}
