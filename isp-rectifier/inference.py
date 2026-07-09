#!/usr/bin/env python3
"""
Python ONNX Runtime Inference for ISP Rectifier
Simple, dependency-light inference script.
"""

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import onnxruntime as ort


class ISPOptimizer:
    """ONNX Runtime wrapper for ISP Rectifier."""
    
    def __init__(
        self,
        model_path: str,
        providers: List[str] = None,
        intra_op_threads: int = 1,
    ):
        if providers is None:
            providers = ['CPUExecutionProvider']
        
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = intra_op_threads
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        
        self.session = ort.InferenceSession(model_path, sess_options, providers=providers)
        
        self.input_names = [i.name for i in self.session.get_inputs()]
        self.output_names = [o.name for o in self.session.get_outputs()]
        
        print(f"Loaded: {model_path}")
        print(f"  Providers: {self.session.get_providers()}")
        print(f"  Inputs: {self.input_names}")
        print(f"  Outputs: {self.output_names}")
    
    def optimize(self, histogram: np.ndarray, metadata: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Run inference.
        
        Args:
            histogram: 256-bin histogram, shape (256,) or (1, 256)
            metadata: 11-dim metadata, shape (11,) or (1, 11)
            
        Returns:
            Dict with keys: wb_gains (3), ccm (9), tone_curve (7), zoom_factor (1)
        """
        # Ensure batch dimension
        if histogram.ndim == 1:
            histogram = histogram.reshape(1, -1)
        if metadata.ndim == 1:
            metadata = metadata.reshape(1, -1)
        
        # Validate
        assert histogram.shape == (1, 256), f"Expected (1, 256), got {histogram.shape}"
        assert metadata.shape == (1, 11), f"Expected (1, 11), got {metadata.shape}"
        
        inputs = {
            self.input_names[0]: histogram.astype(np.float32),
            self.input_names[1]: metadata.astype(np.float32),
        }
        
        outputs = self.session.run(self.output_names, inputs)
        return dict(zip(self.output_names, outputs))
    
    def benchmark(self, num_runs: int = 100, warmup: int = 10) -> Dict[str, float]:
        """Benchmark inference latency."""
        # Dummy inputs
        histogram = np.random.randn(1, 256).astype(np.float32)
        metadata = np.random.randn(1, 11).astype(np.float32)
        
        # Warmup
        for _ in range(warmup):
            self.optimize(histogram, metadata)
        
        # Benchmark
        times = []
        for _ in range(num_runs):
            start = time.perf_counter()
            self.optimize(histogram, metadata)
            times.append(time.perf_counter() - start)
        
        times = np.array(times) * 1000  # ms
        return {
            "mean_ms": float(times.mean()),
            "std_ms": float(times.std()),
            "min_ms": float(times.min()),
            "max_ms": float(times.max()),
            "p50_ms": float(np.percentile(times, 50)),
            "p99_ms": float(np.percentile(times, 99)),
        }


def build_feature_vector(metadata: Dict) -> Tuple[np.ndarray, np.ndarray]:
    """Build histogram + metadata feature vectors from dict."""
    # Histogram
    histogram = np.array(metadata.get("histogram", np.ones(256) * 100), dtype=np.float32)
    if len(histogram) != 256:
        raise ValueError(f"Histogram must have 256 bins, got {len(histogram)}")
    
    # Normalize histogram
    if histogram.sum() > 0:
        histogram = histogram / histogram.sum() * 10000
    
    # Metadata (11 features)
    meta_features = np.array([
        metadata.get("cct", 6500.0) / 10000.0,  # Normalized CCT
        *metadata.get("wb_gains", [1.0, 1.0, 1.0]),
        metadata.get("exposure_time", 0.033),
        metadata.get("iso_gain", 1.0),
        metadata.get("focus_position", 0.5),
        metadata.get("sharpness", 0.5),
        metadata.get("brightness", 0.5),
        metadata.get("contrast", 0.5),
        metadata.get("noise_level", 0.1),
    ], dtype=np.float32)
    
    return histogram, meta_features


def params_to_registers(params: Dict) -> Dict:
    """Convert optimized params to ISP register values."""
    def clamp(val, lo, hi):
        return max(lo, min(hi, val))
    
    wb = params['wb_gains'].flatten()
    ccm = params['ccm'].flatten()
    tone = params['tone_curve'].flatten()
    zoom = params['zoom_factor'].flatten()[0]
    
    return {
        "wb_r_gain": int(clamp(wb[0], 0.1, 10.0) * 4096),
        "wb_g_gain": int(clamp(wb[1], 0.1, 10.0) * 4096),
        "wb_b_gain": int(clamp(wb[2], 0.1, 10.0) * 4096),
        "ccm": [int(clamp(v, -2.0, 2.0) * 4096) for v in ccm],
        "tone_curve_lut": [int(clamp(v, 0.0, 1.0) * 65535) for v in tone],
        "zoom_factor": int(clamp(zoom, 1.0, 4.0) * 4096),
    }


def main():
    parser = argparse.ArgumentParser(description="ISP Rectifier ONNX Inference")
    parser.add_argument("--model", default="fusedispcontroller_int8.onnx", help="ONNX model path")
    parser.add_argument("--metadata", help="JSON metadata file")
    parser.add_argument("--output", help="Output JSON file")
    parser.add_argument("--benchmark", action="store_true", help="Run benchmark")
    parser.add_argument("--providers", nargs="+", default=["CPUExecutionProvider"], help="ONNX providers")
    parser.add_argument("--threads", type=int, default=1, help="Intra-op threads")
    
    args = parser.parse_args()
    
    # Initialize optimizer
    optimizer = ISPOptimizer(args.model, providers=args.providers, intra_op_threads=args.threads)
    
    if args.benchmark:
        print("\n🏃 Running benchmark...")
        results = optimizer.benchmark()
        print(f"  Mean: {results['mean_ms']:.2f} ms")
        print(f"  Std:  {results['std_ms']:.2f} ms")
        print(f"  Min:  {results['min_ms']:.2f} ms")
        print(f"  Max:  {results['max_ms']:.2f} ms")
        print(f"  P50:  {results['p50_ms']:.2f} ms")
        print(f"  P99:  {results['p99_ms']:.2f} ms")
        return
    
    if not args.metadata:
        parser.error("--metadata required for inference")
    
    # Load metadata
    with open(args.metadata) as f:
        metadata = json.load(f)
    
    # Build features
    histogram, metadata_vec = build_feature_vector(metadata)
    
    # Run inference
    print("\n🔮 Running inference...")
    start = time.perf_counter()
    outputs = optimizer.optimize(histogram, metadata_vec)
    latency = (time.perf_counter() - start) * 1000
    
    # Print results
    print(f"\n⏱️  Latency: {latency:.2f} ms")
    print(f"\n📊 Outputs:")
    for name, val in outputs.items():
        print(f"  {name}: {val.flatten()}")
    
    # Convert to registers
    registers = params_to_registers(outputs)
    print(f"\n🎛️  Register values:")
    for k, v in registers.items():
        if isinstance(v, list):
            print(f"  {k}: {v}")
        else:
            print(f"  {k}: {v}")
    
    # Save output
    if args.output:
        result = {
            "outputs": {k: v.tolist() for k, v in outputs.items()},
            "registers": registers,
            "latency_ms": latency,
        }
        with open(args.output, 'w') as f:
            json.dump(result, f, indent=2)
        print(f"\n💾 Saved to {args.output}")


if __name__ == "__main__":
    main()