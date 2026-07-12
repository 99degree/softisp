//! Performance benchmark for real ONNX models using tract directly

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

#[test]
fn bench_fp32_tract() {
    let path = models_dir().join("fusedispcontroller.onnx");
    if !path.exists() {
        eprintln!("Skipping: FP32 model not found");
        return;
    }
    
    eprintln!("\n=== Loading FP32 Model with Tract ===");
    let start = Instant::now();
    let model = tract_onnx::onnx()
        .model_for_path(&path)
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
    let _ = model.run(tvec![hist.clone().into(), meta.clone().into()]).unwrap();
    
    // Benchmark
    let iterations = 100;
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = model.run(tvec![hist.clone().into(), meta.clone().into()]).unwrap();
    }
    let elapsed = start.elapsed();
    
    let avg_us = elapsed.as_micros() as f64 / iterations as f64;
    let fps = 1_000_000.0 / avg_us;
    
    println!("\n=== FP32 Model Tract Benchmark ===");
    println!("Load time: {:?}", load_time);
    println!("Iterations: {}", iterations);
    println!("Total time: {:?}", elapsed);
    println!("Average: {:.2} µs/frame ({:.0} FPS)", avg_us, fps);
    println!("==================================\n");
}

#[test]
fn bench_int8_tract() {
    let path = models_dir().join("fusedispcontroller_int8.onnx");
    if !path.exists() {
        eprintln!("Skipping: INT8 model not found");
        return;
    }
    
    eprintln!("\n=== Loading INT8 Model with Tract ===");
    let start = Instant::now();
    let model = tract_onnx::onnx()
        .model_for_path(&path)
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
    let _ = model.run(tvec![hist.clone().into(), meta.clone().into()]).unwrap();
    
    // Benchmark
    let iterations = 100;
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = model.run(tvec![hist.clone().into(), meta.clone().into()]).unwrap();
    }
    let elapsed = start.elapsed();
    
    let avg_us = elapsed.as_micros() as f64 / iterations as f64;
    let fps = 1_000_000.0 / avg_us;
    
    println!("\n=== INT8 Model Tract Benchmark ===");
    println!("Load time: {:?}", load_time);
    println!("Iterations: {}", iterations);
    println!("Total time: {:?}", elapsed);
    println!("Average: {:.2} µs/frame ({:.0} FPS)", avg_us, fps);
    println!("==================================\n");
}

#[test]
fn bench_fp16_tract() {
    let path = models_dir().join("fusedispcontroller_fp16.onnx");
    if !path.exists() {
        eprintln!("Skipping: FP16 model not found");
        return;
    }
    
    eprintln!("\n=== Loading FP16 Model with Tract ===");
    let start = Instant::now();
    let model = match tract_onnx::onnx()
        .model_for_path(&path)
        .and_then(|m| m.into_optimized())
        .and_then(|m| m.into_runnable())
    {
        Ok(m) => m,
        Err(e) => {
            eprintln!("Skipping FP16: tract-optimize failed: {}", e);
            return;
        }
    };
    let load_time = start.elapsed();
    eprintln!("Load time: {:?}", load_time);
    
    // Create input tensors
    let hist = Tensor::from_shape(&[1, 256], &vec![0.5f32; 256]).unwrap();
    let meta = Tensor::from_shape(&[1, 11], &vec![0.5f32; 11]).unwrap();
    
    // Warmup
    let _ = model.run(tvec![hist.clone().into(), meta.clone().into()]).unwrap();
    
    // Benchmark
    let iterations = 100;
    let start = Instant::now();
    for _ in 0..iterations {
        let _ = model.run(tvec![hist.clone().into(), meta.clone().into()]).unwrap();
    }
    let elapsed = start.elapsed();
    
    let avg_us = elapsed.as_micros() as f64 / iterations as f64;
    let fps = 1_000_000.0 / avg_us;
    
    println!("\n=== FP16 Model Tract Benchmark ===");
    println!("Load time: {:?}", load_time);
    println!("Iterations: {}", iterations);
    println!("Total time: {:?}", elapsed);
    println!("Average: {:.2} µs/frame ({:.0} FPS)", avg_us, fps);
    println!("==================================\n");
}

#[test]
fn bench_all_models_comparison() {
    let iterations = 100;
    
    // FP32
    let fp32_path = models_dir().join("fusedispcontroller.onnx");
    let (fp32_load, fp32_fps) = if fp32_path.exists() {
        let start = Instant::now();
        let model = tract_onnx::onnx().model_for_path(&fp32_path).unwrap()
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
        (load_time, 1_000_000.0 / avg_us)
    } else {
        (std::time::Duration::ZERO, 0.0)
    };
    
    // INT8
    let int8_path = models_dir().join("fusedispcontroller_int8.onnx");
    let (int8_load, int8_fps) = if int8_path.exists() {
        let start = Instant::now();
        let model = tract_onnx::onnx().model_for_path(&int8_path).unwrap()
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
        (load_time, 1_000_000.0 / avg_us)
    } else {
        (std::time::Duration::ZERO, 0.0)
    };
    
    // FP16
    let fp16_path = models_dir().join("fusedispcontroller_fp16.onnx");
    let (fp16_load, fp16_fps) = if fp16_path.exists() {
        let start = Instant::now();
        match tract_onnx::onnx().model_for_path(&fp16_path)
            .and_then(|m| m.into_optimized())
            .and_then(|m| m.into_runnable())
        {
            Ok(model) => {
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
                (load_time, 1_000_000.0 / avg_us)
            }
            Err(e) => {
                eprintln!("Skipping FP16 in comparison: tract-optimize failed: {}", e);
                (std::time::Duration::ZERO, 0.0)
            }
        }
    } else {
        (std::time::Duration::ZERO, 0.0)
    };
    
    println!("\n=== All Models Comparison ({} iterations) ===", iterations);
    println!("FP32: Load {:?}, Inference {:.0} FPS", fp32_load, fp32_fps);
    println!("INT8: Load {:?}, Inference {:.0} FPS", int8_load, int8_fps);
    println!("FP16: Load {:?}, Inference {:.0} FPS", fp16_load, fp16_fps);
    println!("=============================================\n");
}
