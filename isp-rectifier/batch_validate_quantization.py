#!/usr/bin/env python3
"""
Batch Quantization Validation with Statistical Analysis

Compares FP32 vs quantized model outputs across many samples,
generates statistics and visualizations.
"""

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import onnxruntime as ort
from tqdm import tqdm

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("⚠️  matplotlib not available, skipping plots")


class QuantizationValidator:
    def __init__(
        self,
        fp32_model: str,
        quant_model: str,
        input_shape: Tuple[int, int] = (1, 267),
        providers: List[str] = None,
    ):
        self.fp32_session = ort.InferenceSession(fp32_model, providers=providers)
        self.quant_session = ort.InferenceSession(quant_model, providers=providers)
        
        # Get input/output names
        self.input_names = [i.name for i in self.fp32_session.get_inputs()]
        self.output_names = [o.name for o in self.fp32_session.get_outputs()]
        
        print(f"Inputs: {self.input_names}")
        print(f"Outputs: {self.output_names}")
        
        # Expected output shapes (from our model)
        self.output_dims = {
            'wbgains': 3,
            'ccm': 9,
            'tonecurve': 7,
            'zoom_factor': 1,
        }
    
    def generate_test_samples(self, num_samples: int, seed: int = 42) -> List[Dict[str, np.ndarray]]:
        """Generate synthetic test samples matching training distribution."""
        np.random.seed(seed)
        samples = []
        
        for _ in range(num_samples):
            # Histogram: different exposure conditions
            hist_type = np.random.choice(["normal", "under", "over", "high_contrast"], p=[0.5, 0.15, 0.15, 0.2])
            
            if hist_type == "normal":
                hist = np.random.normal(128, 40, 256)
            elif hist_type == "under":
                hist = np.exp(-np.linspace(0, 5, 256)) * 255
            elif hist_type == "over":
                hist = np.exp(np.linspace(0, -5, 256)) * 255
            else:
                hist = np.concatenate([
                    np.random.normal(40, 10, 128),
                    np.random.normal(210, 10, 128)
                ])
            
            hist = np.clip(hist, 0, 255).astype(np.float32)
            
            # Normalize histogram
            if hist.sum() > 0:
                hist = hist / hist.sum() * 10000
            
            # Metadata features (11 dims)
            meta = np.array([
                np.random.uniform(2000, 10000) / 10000,  # CCT normalized
                *np.random.uniform(0.5, 2.0, 3),         # WB gains
                np.random.uniform(0.001, 0.1),           # exposure
                np.random.uniform(1.0, 16.0),            # ISO gain
                np.random.uniform(0, 1),                 # focus
                np.random.uniform(0.1, 1.0),             # sharpness
                np.random.uniform(0.1, 1.0),             # brightness
                np.random.uniform(0.1, 1.0),             # contrast
                np.random.uniform(0.0, 0.3),             # noise
            ], dtype=np.float32)
            
            # Combine: 256 + 11 = 267
            features = np.concatenate([hist, meta])
            
            samples.append({
                'histogram': hist.reshape(1, -1),
                'metadata': meta.reshape(1, -1),
                'features': features.reshape(1, -1),
            })
        
        return samples
    
    def run_inference(self, session: ort.InferenceSession, sample: Dict) -> Dict[str, np.ndarray]:
        """Run inference on a single sample."""
        inputs = {
            self.input_names[0]: sample['histogram'],
            self.input_names[1]: sample['metadata'],
        }
        outputs = session.run(self.output_names, inputs)
        return dict(zip(self.output_names, outputs))
    
    def validate(
        self,
        num_samples: int = 1000,
        tolerances: Dict[str, Tuple[float, float]] = None,
    ) -> Dict:
        """Run batch validation."""
        
        if tolerances is None:
            tolerances = {
                'wbgains': (0.01, 0.05),    # (mean, max)
                'ccm': (0.01, 0.05),
                'tonecurve': (0.02, 0.10),
                'zoom_factor': (0.005, 0.02),
            }
        
        print(f"\n🔍 Generating {num_samples} test samples...")
        samples = self.generate_test_samples(num_samples)
        
        # Storage for differences
        diffs = {name: [] for name in self.output_names}
        
        print("🏃 Running inference...")
        fp32_times = []
        quant_times = []
        
        for sample in tqdm(samples, desc="Validating"):
            # FP32 inference
            start = time.perf_counter()
            fp32_out = self.run_inference(self.fp32_session, sample)
            fp32_times.append(time.perf_counter() - start)
            
            # Quantized inference
            start = time.perf_counter()
            quant_out = self.run_inference(self.quant_session, sample)
            quant_times.append(time.perf_counter() - start)
            
            # Compute differences
            for name in self.output_names:
                fp32_val = fp32_out[name].flatten()
                quant_val = quant_out[name].flatten()
                diff = np.abs(fp32_val - quant_val)
                diffs[name].extend(diff.tolist())
        
        # Statistics
        stats = {}
        passed = True
        
        print("\n" + "="*70)
        print("VALIDATION RESULTS")
        print("="*70)
        print(f"{'Output':<15} {'Mean Diff':>12} {'Max Diff':>12} {'Std Diff':>12} {'Mean Tol':>10} {'Max Tol':>10} {'Status':>8}")
        print("-"*70)
        
        for name in self.output_names:
            arr = np.array(diffs[name])
            mean_diff = arr.mean()
            max_diff = arr.max()
            std_diff = arr.std()
            
            mean_tol, max_tol = tolerances.get(name, (0.01, 0.05))
            
            mean_ok = mean_diff <= mean_tol
            max_ok = max_diff <= max_tol
            ok = mean_ok and max_ok
            
            if not ok:
                passed = False
            
            status = "✅ PASS" if ok else "❌ FAIL"
            
            print(f"{name:<15} {mean_diff:>12.6f} {max_diff:>12.6f} {std_diff:>12.6f} "
                  f"{mean_tol:>10.4f} {max_tol:>10.4f} {status:>8}")
            
            stats[name] = {
                'mean': float(mean_diff),
                'max': float(max_diff),
                'std': float(std_diff),
                'tolerance_mean': mean_tol,
                'tolerance_max': max_tol,
                'passed': ok,
                'raw_values': arr.tolist() if num_samples <= 100 else None,
            }
        
        # Latency stats
        print("-"*70)
        print(f"\n⚡ Latency (FP32):  {np.mean(fp32_times)*1000:.2f} ± {np.std(fp32_times)*1000:.2f} ms")
        print(f"⚡ Latency (Quant): {np.mean(quant_times)*1000:.2f} ± {np.std(quant_times)*1000:.2f} ms")
        speedup = np.mean(fp32_times) / np.mean(quant_times)
        print(f"🚀 Speedup: {speedup:.2f}x")
        
        stats['latency'] = {
            'fp32_mean_ms': float(np.mean(fp32_times) * 1000),
            'fp32_std_ms': float(np.std(fp32_times) * 1000),
            'quant_mean_ms': float(np.mean(quant_times) * 1000),
            'quant_std_ms': float(np.std(quant_times) * 1000),
            'speedup': float(speedup),
        }
        
        stats['overall_passed'] = passed
        stats['num_samples'] = num_samples
        
        print("="*70)
        if passed:
            print("✅ ALL CHECKS PASSED")
        else:
            print("❌ SOME CHECKS FAILED")
        print("="*70)
        
        return stats
    
    def plot_errors(self, stats: Dict, output_dir: Path):
        """Generate error distribution plots."""
        if not HAS_MATPLOTLIB:
            return
        
        output_dir.mkdir(parents=True, exist_ok=True)
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        axes = axes.flatten()
        
        for idx, name in enumerate(self.output_names):
            if name not in stats:
                continue
            
            ax = axes[idx]
            data = stats[name]
            
            if data.get('raw_values'):
                values = np.array(data['raw_values'])
                ax.hist(values, bins=50, alpha=0.7, edgecolor='black', density=True)
                
                # Add tolerance lines
                ax.axvline(data['tolerance_mean'], color='orange', linestyle='--', 
                          label=f"Mean tol: {data['tolerance_mean']:.4f}")
                ax.axvline(data['tolerance_max'], color='red', linestyle='--',
                          label=f"Max tol: {data['tolerance_max']:.4f}")
                ax.axvline(data['mean'], color='green', linestyle='-',
                          label=f"Actual mean: {data['mean']:.4f}")
                
                ax.set_xlabel('Absolute Error')
                ax.set_ylabel('Density')
                ax.set_title(f'{name} Error Distribution')
                ax.legend(fontsize=8)
                ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_dir / 'quantization_errors.png', dpi=150)
        plt.close()
        
        # Latency comparison
        fig, ax = plt.subplots(figsize=(8, 5))
        lat = stats['latency']
        categories = ['FP32', 'Quantized']
        means = [lat['fp32_mean_ms'], lat['quant_mean_ms']]
        stds = [lat['fp32_std_ms'], lat['quant_std_ms']]
        
        bars = ax.bar(categories, means, yerr=stds, capsize=10, color=['steelblue', 'coral'])
        ax.set_ylabel('Latency (ms)')
        ax.set_title('Inference Latency Comparison')
        ax.grid(True, alpha=0.3, axis='y')
        
        # Add value labels
        for bar, mean in zip(bars, means):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                   f'{mean:.2f} ms', ha='center', va='bottom')
        
        plt.tight_layout()
        plt.savefig(output_dir / 'latency_comparison.png', dpi=150)
        plt.close()
        
        print(f"📊 Plots saved to {output_dir}/")


def main():
    parser = argparse.ArgumentParser(
        description="Validate quantized ONNX model against FP32 baseline"
    )
    parser.add_argument("--fp32-model", required=True, help="FP32 ONNX model path")
    parser.add_argument("--quant-model", required=True, help="Quantized ONNX model path")
    parser.add_argument("--samples", type=int, default=1000, help="Number of test samples")
    parser.add_argument("--tolerances", nargs=4, type=float, 
                        default=[0.01, 0.05, 0.02, 0.01],
                        help="Mean/Max tolerances for WB, CCM, Tone, Zoom")
    parser.add_argument("--output", default="validation_results.json", help="Output JSON path")
    parser.add_argument("--plot-dir", default="plots", help="Directory for plots")
    parser.add_argument("--providers", nargs='+', default=['CPUExecutionProvider'],
                        help="ONNX Runtime providers")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    
    args = parser.parse_args()
    
    # Build tolerances dict
    tolerances = {
        'wbgains': (args.tolerances[0], args.tolerances[1]),
        'ccm': (args.tolerances[0], args.tolerances[1]),
        'tonecurve': (args.tolerances[2], args.tolerances[2] * 5),
        'zoom_factor': (args.tolerances[3], args.tolerances[3] * 5),
    }
    
    # Run validation
    validator = QuantizationValidator(
        fp32_model=args.fp32_model,
        quant_model=args.quant_model,
        providers=args.providers,
    )
    
    stats = validator.validate(
        num_samples=args.samples,
        tolerances=tolerances,
    )
    
    # Save results
    with open(args.output, 'w') as f:
        json.dump(stats, f, indent=2)
    print(f"\n💾 Results saved to {args.output}")
    
    # Generate plots
    validator.plot_errors(stats, Path(args.plot_dir))
    
    # Exit code
    sys.exit(0 if stats['overall_passed'] else 1)


if __name__ == "__main__":
    main()