//! Performance benchmark for real ONNX models using tract

use std::path::PathBuf;
use std::time::Instant;
use tract_onnx::prelude::*;

fn models_dir() -> PathBuf {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let models = manifest.join("models");
    if models.exists() {
        models
    } else {
        PathBuf::from("models")
    }
}

fn bench_model(name: &str, path: &PathBuf) {
    if !path.exists() {
        eprintln!("Skipping {}: not found", name);
        return;
    }
    
    let file_size = std::fs::metadata(path).map(|m| m.len()).unwrap_or(0);
    
    eprintln!("\n=== Loading {} ({:.1} KB) ===", name, file_size as f64 / 1024.0);
    let start = Instant::now();
    let model = tract_onnx::onnx()
        .model_for_path(path)
        .unwrap()
        .into_optimized()
        .unwrap()
        .into_runnable()
        .unwrap();
    let load_time = start.elapsed();
    eprintln!("Load time: {:?}", load_time);
    
    // Create input tensors
    let hist = Tensor::from_shape(&[1, 256], &vec![0.5f32; 256]).unwrap();
    let meta = Tensor::from_shape(&[1, 11], &vec![0.5f32; 11]).unwrap();
    
    // Warmup
    for _ in 0..5 {
        let _ = model.run(tvec![hist.clone().into(), meta.clone().into()]).unwrap();
    }
    
    // Benchmark
    let iterations = 20;
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = model.run(tvec![hist.clone().into(), meta.clone().into()]).unwrap();
    }
    let elapsed = start.elapsed();
    
    let avg_us = elapsed.as_micros() as f64 / iterations as f64;
    let fps = 1_000_000.0 / avg_us;
    
    println!("\n=== {} Benchmark ===", name);
    println!("File size: {:.1} KB", file_size as f64 / 1024.0);
    println!("Load time: {:?}", load_time);
    println!("Iterations: {}", iterations);
    println!("Total time: {:?}", elapsed);
    println!("Average: {:.2} µs/frame ({:.0} FPS)", avg_us, fps);
    println!("========================\n");
}

#[test]
fn bench_light_model() {
    let path = models_dir().join("fusedispcontroller_light.onnx");
    bench_model("LIGHT-FP32", &path);
}

#[test]
fn bench_light_int8_model() {
    let path = models_dir().join("fusedispcontroller_light_int8.onnx");
    bench_model("LIGHT-INT8", &path);
}

#[test]
fn bench_light_fp16_model() {
    // FP16 models require FP16 support in tract, which is limited
    eprintln!("Skipping LIGHT-FP16: tract FP16 support is limited");
}

#[test]
fn bench_medium_model() {
    let path = models_dir().join("fusedispcontroller_medium.onnx");
    bench_model("MEDIUM-FP32", &path);
}

#[test]
fn bench_medium_int8_model() {
    let path = models_dir().join("fusedispcontroller_medium_int8.onnx");
    bench_model("MEDIUM-INT8", &path);
}

#[test]
fn bench_medium_fp16_model() {
    // FP16 models require FP16 support in tract, which is limited
    eprintln!("Skipping MEDIUM-FP16: tract FP16 support is limited");
}

#[test]
fn bench_fp32_model() {
    let path = models_dir().join("fusedispcontroller.onnx");
    bench_model("FP32", &path);
}

#[test]
fn bench_int8_model() {
    let path = models_dir().join("fusedispcontroller_int8.onnx");
    bench_model("INT8", &path);
}

#[test]
fn bench_fp16_model() {
    // FP16 models require FP16 support in tract, which is limited
    eprintln!("Skipping FP16: tract FP16 support is limited");
}

#[test]
fn bench_all_models_comparison() {
    let iterations = 20;
    let mut results = Vec::new();
    
    for (name, filename) in [
        // FP16 skipped: tract doesn't support FP16 properly
        ("LIT-FP32", "fusedispcontroller_light.onnx"),
        ("LIT-INT8", "fusedispcontroller_light_int8.onnx"),
        ("MED-FP32", "fusedispcontroller_medium.onnx"),
        ("MED-INT8", "fusedispcontroller_medium_int8.onnx"),
        ("FP32", "fusedispcontroller.onnx"),
        ("INT8", "fusedispcontroller_int8.onnx"),
    ] {
        let path = models_dir().join(filename);
        if !path.exists() {
            continue;
        }
        
        let file_size = std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);
        
        let start = Instant::now();
        let model = tract_onnx::onnx().model_for_path(&path).unwrap()
            .into_optimized().unwrap().into_runnable().unwrap();
        let load_time = start.elapsed();
        
        let hist = Tensor::from_shape(&[1, 256], &vec![0.5f32; 256]).unwrap();
        let meta = Tensor::from_shape(&[1, 11], &vec![0.5f32; 11]).unwrap();
        let _ = model.run(tvec![hist.clone().into(), meta.clone().into()]).unwrap();
        
        let start = Instant::now();
        for _ in 0..iterations {
            let _ = model.run(tvec![hist.clone().into(), meta.clone().into()]).unwrap();
        }
        let elapsed = start.elapsed();
        let avg_us = elapsed.as_micros() as f64 / iterations as f64;
        let fps = 1_000_000.0 / avg_us;
        
        results.push((name, file_size, load_time, fps));
    }
    
    println!("\n=== All Models Comparison ({} iterations) ===", iterations);
    println!("{:<10} {:>10} {:>15} {:>10}", "Model", "Size", "Load Time", "FPS");
    println!("{}", "-".repeat(50));
    for (name, size, load, fps) in &results {
        println!("{:<10} {:>8.1} KB {:>12?} {:>10.0}", name, *size as f64 / 1024.0, load, fps);
    }
    println!("=============================================\n");
}
