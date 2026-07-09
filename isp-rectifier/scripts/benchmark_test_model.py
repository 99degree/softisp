#!/usr/bin/env python3
"""
Quick benchmark for the test ONNX model.
Run: python scripts/benchmark_test_model.py
"""

import numpy as np
import time
import os
import sys


def benchmark():
    try:
        import onnxruntime as ort
    except ImportError:
        print("Installing onnxruntime...")
        import subprocess
        subprocess.check_call(["pip", "install", "onnxruntime"])
        import onnxruntime as ort

    model_path = "models/fusedispcontroller_test.onnx"
    if not os.path.exists(model_path):
        print(f"Model not found at {model_path}")
        print("Run: python scripts/create_test_onnx.py")
        sys.exit(1)

    print(f"Loading model: {model_path}")
    sess = ort.InferenceSession(model_path)

    # Test data
    hist = np.random.randn(1, 256).astype(np.float32)
    meta = np.random.randn(1, 11).astype(np.float32)
    inputs = np.concatenate([hist, meta], axis=1)  # [1, 267]

    # Warmup
    print("Warming up...")
    for _ in range(10):
        sess.run(None, {"input": inputs})

    # Benchmark
    print("Benchmarking (100 iterations)...")
    times = []
    for _ in range(100):
        start = time.perf_counter()
        outputs = sess.run(None, {"input": inputs})
        times.append(time.perf_counter() - start)

    output = outputs[0][0]
    print(f"\n📊 Results:")
    print(f"   Latency: {np.mean(times)*1000:.2f} ms avg, {np.std(times)*1000:.2f} ms std")
    print(f"   Output shape: {outputs[0].shape}")
    print(f"   WB gains: {output[:3]}")
    print(f"   CCM (diag): [{output[3]:.3f}, {output[7]:.3f}, {output[11]:.3f}]")
    print(f"   Tone curve: {output[12:19]}")
    print(f"   Zoom: {output[19]:.3f}")
    print("✅ Test passed!")


if __name__ == "__main__":
    benchmark()
