#!/usr/bin/env python3
"""
Example: Using ISP Rectifier in a Camera Pipeline

This demonstrates how to integrate the distilled ISP controller
into a real camera processing pipeline.
"""

import json
import time
import argparse
from pathlib import Path
from typing import Dict, Any
import numpy as np
import onnxruntime as ort

# Simulated camera frame metadata (replace with real camera API)
class CameraFrame:
    def __init__(self, frame_id: int):
        self.frame_id = frame_id
        self.timestamp = time.time_ns()
        
        # Simulated sensor data
        self.histogram = np.random.randint(0, 256, 256).astype(np.uint32)
        self.cct = 5500.0
        self.wb_gains = [1.2, 1.0, 0.95]
        self.exposure_time = 0.033
        self.iso_gain = 2.0
        self.focus_position = 0.5
        self.sharpness = 0.75
        self.brightness = 0.6
        self.contrast = 0.65
        self.noise_level = 0.08
    
    def to_metadata_dict(self) -> Dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "histogram": self.histogram.tolist(),
            "cct": self.cct,
            "wb_gains": self.wb_gains,
            "exposure_time": self.exposure_time,
            "iso_gain": self.iso_gain,
            "focus_position": self.focus_position,
            "sharpness": self.sharpness,
            "brightness": self.brightness,
            "contrast": self.contrast,
            "noise_level": self.noise_level,
        }


def build_feature_vector(metadata: Dict) -> tuple:
    """Build model input from camera metadata."""
    histogram = np.array(metadata["histogram"], dtype=np.float32)
    if histogram.sum() > 0:
        histogram = histogram / histogram.sum() * 10000
    
    meta_vec = np.array([
        metadata["cct"] / 10000.0,
        *metadata["wb_gains"],
        metadata["exposure_time"],
        metadata["iso_gain"],
        metadata["focus_position"],
        metadata["sharpness"],
        metadata["brightness"],
        metadata["contrast"],
        metadata["noise_level"],
    ], dtype=np.float32)
    
    return histogram, meta_vec


def clamp_params(params: Dict) -> Dict:
    """Clamp parameters to safe register ranges."""
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


class ISPPipeline:
    """Complete ISP pipeline with neural controller."""
    
    def __init__(self, model_path: str, providers: list = None):
        self.optimizer = ort.InferenceSession(
            model_path,
            providers=providers or ['CPUExecutionProvider']
        )
        self.input_names = [i.name for i in self.optimizer.get_inputs()]
        self.output_names = [o.name for o in self.optimizer.get_outputs()]
        
        # Statistics
        self.frame_count = 0
        self.total_latency = 0.0
        
        print(f"✅ ISP Pipeline initialized with {model_path}")
        print(f"   Providers: {self.optimizer.get_providers()}")
    
    def process_frame(self, frame: CameraFrame) -> Dict:
        """Process a single frame through the ISP pipeline."""
        start = time.perf_counter()
        
        # Get frame metadata
        metadata = frame.to_metadata_dict()
        histogram, meta_vec = build_feature_vector(metadata)
        
        # Run neural ISP controller
        inputs = {
            self.input_names[0]: histogram.astype(np.float32).reshape(1, -1),
            self.input_names[1]: meta_vec.astype(np.float32).reshape(1, -1),
        }
        
        outputs = self.optimizer.run(self.output_names, inputs)
        
        # Parse outputs
        params = dict(zip(self.output_names, outputs))
        
        # Convert to register values
        registers = clamp_params(params)
        
        # Apply to ISP hardware (simulated)
        self._apply_to_isp(registers)
        
        # Track stats
        latency = (time.perf_counter() - start) * 1000
        self.frame_count += 1
        self.total_latency += latency
        
        return {
            "frame_id": frame.frame_id,
            "latency_ms": latency,
            "params": {k: v.flatten().tolist() for k, v in params.items()},
            "registers": registers,
        }
    
    def _apply_to_isp(self, registers: Dict):
        """Apply register values to ISP hardware.
        
        Replace this with actual hardware writes:
        - V4L2 controls
        - Vulkan push constants
        - Custom kernel driver
        - MMIO registers
        """
        # Simulated hardware write
        pass
    
    def get_stats(self) -> Dict:
        return {
            "frames_processed": self.frame_count,
            "avg_latency_ms": self.total_latency / max(1, self.frame_count),
            "total_latency_ms": self.total_latency,
        }


def main():
    parser = argparse.ArgumentParser(description="ISP Pipeline Demo")
    parser.add_argument("--model", default="fusedispcontroller_int8.onnx",
                        help="ONNX model path")
    parser.add_argument("--frames", type=int, default=100,
                        help="Number of frames to process")
    parser.add_argument("--providers", nargs="+", default=["CPUExecutionProvider"],
                        help="ONNX Runtime providers")
    parser.add_argument("--output", help="Output JSON file")
    args = parser.parse_args()
    
    # Initialize pipeline
    pipeline = ISPPipeline(args.model, providers=args.providers)
    
    # Process frames
    print(f"\n🎬 Processing {args.frames} frames...")
    results = []
    
    for i in range(args.frames):
        frame = CameraFrame(i)
        result = pipeline.process_frame(frame)
        results.append(result)
        
        if (i + 1) % 10 == 0:
            stats = pipeline.get_stats()
            print(f"  Frame {i+1}/{args.frames} | Avg latency: {stats['avg_latency_ms']:.2f} ms")
    
    # Final stats
    stats = pipeline.get_stats()
    print(f"\n📊 Final Statistics:")
    print(f"  Frames processed: {stats['frames_processed']}")
    print(f"  Average latency:  {stats['avg_latency_ms']:.2f} ms")
    print(f"  Throughput:       {1000/stats['avg_latency_ms']:.1f} FPS")
    
    # Save results
    if args.output:
        with open(args.output, 'w') as f:
            json.dump({
                "stats": stats,
                "frames": results,
            }, f, indent=2)
        print(f"\n💾 Results saved to {args.output}")


if __name__ == "__main__":
    main()