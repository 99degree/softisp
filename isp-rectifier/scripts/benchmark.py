#!/usr/bin/env python3
"""
Benchmark ISP Rectifier inference across different models and providers.
"""

import argparse
import json
import time
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np
import onnxruntime as ort


def benchmark_model(
    model_path: str,
    providers: List[str],
    num_runs: int = 1000,
    warmup: int = 50,
    batch_size: int = 1,
) -> Dict:
    """Benchmark a single model."""
    print(f"\n🔍 Benchmarking: {model_path}")
    print(f"   Providers: {providers}")
    print(f"   Batch size: {batch_size}")
    
    # Load model
    sess_options = ort.SessionOptions()
    sess_options.intra_op_num_threads = 1
    sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    
    try:
        session = ort.InferenceSession(model_path, sess_options, providers=providers)
    except Exception as e:
        return {"error": str(e)}
    
    input_names = [i.name for i in session.get_inputs()]
    output_names = [o.name for o in session.get_outputs()]
    
    # Generate dummy data
    np.random.seed(42)
    histograms = np.random.randn(batch_size, 256).astype(np.float32)
    metadatas = np.random.randn(batch_size, 11).astype(np.float32)
    
    # Warmup
    for _ in range(warmup):
        inputs = {
            input_names[0]: histograms,
            input_names[1]: metadatas,
        }
        session.run(output_names, inputs)
    
    # Benchmark
    times = []
    for _ in range(num_runs):
        start = time.perf_counter()
        inputs = {
            input_names[0]: histograms,
            input_names[1]: metadatas,
        }
        session.run(output_names, inputs)
        times.append(time.perf_counter() - start)
    
    times = np.array(times) * 1000  # ms
    
    return {
        "model": model_path,
        "providers": providers,
        "batch_size": batch_size,
        "num_runs": num_runs,
        "mean_ms": float(times.mean()),
        "std_ms": float(times.std()),
        "min_ms": float(times.min()),
        "max_ms": float(times.max()),
        "p50_ms": float(np.percentile(times, 50)),
        "p90_ms": float(np.percentile(times, 90)),
        "p99_ms": float(np.percentile(times, 99)),
        "throughput_fps": float(batch_size / (times.mean() / 1000)),
    }


def get_model_size(path: str) -> float:
    """Get model size in MB."""
    import os
    return os.path.getsize(path) / (1024 * 1024)


def main():
    parser = argparse.ArgumentParser(description="Benchmark ISP Rectifier models")
    parser.add_argument("models", nargs="+", help="ONNX model paths")
    parser.add_argument("--providers", nargs="+", default=["CPUExecutionProvider"])
    parser.add_argument("--runs", type=int, default=1000, help="Number of benchmark runs")
    parser.add_argument("--warmup", type=int, default=50, help="Warmup runs")
    parser.add_argument("--batch-sizes", nargs="+", type=int, default=[1, 4, 8, 16])
    parser.add_argument("--output", help="Output JSON file")
    parser.add_argument("--cuda", action="store_true", help="Use CUDA provider if available")
    
    args = parser.parse_args()
    
    # Add CUDA provider if requested
    if args.cuda:
        providers = ["CUDAExecutionProvider"] + args.providers
    else:
        providers = args.providers
    
    results = []
    
    for model_path in args.models:
        if not Path(model_path).exists():
            print(f"❌ Model not found: {model_path}")
            continue
        
        size_mb = get_model_size(model_path)
        print(f"\n📦 Model: {model_path} ({size_mb:.2f} MB)")
        
        for batch_size in args.batch_sizes:
            result = benchmark_model(
                model_path,
                providers,
                num_runs=args.runs,
                warmup=args.warmup,
                batch_size=batch_size,
            )
            result["model_size_mb"] = size_mb
            results.append(result)
            
            if "error" not in result:
                print(f"  Batch {batch_size:2d}: {result['mean_ms']:.2f}±{result['std_ms']:.2f} ms "
                      f"({result['throughput_fps']:.1f} FPS)")
            else:
                print(f"  Batch {batch_size:2d}: ERROR - {result['error']}")
    
    # Summary table
    print("\n" + "="*100)
    print(f"{'Model':<30} {'Batch':>5} {'Mean (ms)':>10} {'Std (ms)':>10} {'FPS':>10} {'Size (MB)':>10}")
    print("-"*100)
    for r in results:
        if "error" not in r:
            model_name = Path(r['model']).name
            print(f"{model_name:<30} {r['batch_size']:>5} {r['mean_ms']:>10.2f} {r['std_ms']:>10.2f} "
                  f"{r['throughput_fps']:>10.1f} {r['model_size_mb']:>10.2f}")
    print("="*100)
    
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"\n💾 Results saved to {args.output}")


if __name__ == "__main__":
    main()