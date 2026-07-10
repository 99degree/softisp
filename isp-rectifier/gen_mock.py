#!/usr/bin/env python3
"""
Generate mock ONNX model using existing ISPDistilledModel architecture.

Usage:
    python gen_mock.py                           # → models/fusedispcontroller.onnx
    python gen_mock.py -o my_model.onnx          # custom output
    python gen_mock.py --int8                    # also quantize to INT8
    python gen_mock.py --all                     # FP32 + INT8 + FP16

Requirements:
    pip install torch onnx
    pip install onnxruntime onnxconverter-common  # for quantization
"""

import argparse
import sys
import os
import time

# Add parent to path for import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import torch
import torch.nn as nn
import numpy as np


class ISPDistilledModel(nn.Module):
    """
    Full-sized Distilled ISP Controller Model (~1.2M params).
    Input:  histogram (B, 256) + metadata (B, 11)
    Output: wb (B, 3) + ccm (B, 9) + tone (B, 7) + zoom (B, 1)
    """

    def __init__(self, metadata_dim: int = 11):
        super().__init__()

        # Histogram backbone: 1D CNN
        self.hist_backbone = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=7, padding=3),
            nn.BatchNorm1d(16),
            nn.ReLU(),
            nn.MaxPool1d(2),
            nn.Conv1d(16, 32, kernel_size=5, padding=2),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.MaxPool1d(2),
            nn.Conv1d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(32),
            nn.Flatten(),
        )

        # Metadata backbone: MLP
        self.meta_backbone = nn.Sequential(
            nn.Linear(metadata_dim, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(128, 256),
            nn.BatchNorm1d(256),
            nn.ReLU(),
            nn.Dropout(0.1),
        )

        # Fusion
        combined_dim = 64 * 32 + 256  # 2304
        self.fusion = nn.Sequential(
            nn.Linear(combined_dim, 512),
            nn.BatchNorm1d(512),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(512, 256),
            nn.BatchNorm1d(256),
            nn.ReLU(),
            nn.Dropout(0.1),
        )

        # Multi-head outputs
        self.wb_head = nn.Sequential(nn.Linear(256, 64), nn.ReLU(), nn.Linear(64, 3))
        self.ccm_head = nn.Sequential(nn.Linear(256, 128), nn.ReLU(), nn.Linear(128, 9))
        self.tone_head = nn.Sequential(nn.Linear(256, 64), nn.ReLU(), nn.Linear(64, 7))
        self.zoom_head = nn.Sequential(nn.Linear(256, 32), nn.ReLU(), nn.Linear(32, 1))

    def forward(self, histogram: torch.Tensor, metadata: torch.Tensor):
        hist_feat = self.hist_backbone(histogram.unsqueeze(1))
        meta_feat = self.meta_backbone(metadata)
        combined = torch.cat([hist_feat, meta_feat], dim=1)
        fused = self.fusion(combined)
        return {
            "wb": self.wb_head(fused),
            "ccm": self.ccm_head(fused),
            "tone": self.tone_head(fused),
            "zoom": self.zoom_head(fused),
        }


class ISPLightModel(nn.Module):
    """
    Lightweight Distilled ISP Controller Model (~118K params).
    ~10x smaller than ISPDistilledModel for embedded/mobile deployment.
    Input:  histogram (B, 256) + metadata (B, 11)
    Output: wb (B, 3) + ccm (B, 9) + tone (B, 7) + zoom (B, 1)
    """

    def __init__(self, metadata_dim: int = 11):
        super().__init__()

        # Histogram backbone: 1D CNN (half channels, smaller kernels)
        self.hist_backbone = nn.Sequential(
            nn.Conv1d(1, 8, kernel_size=5, padding=2),
            nn.BatchNorm1d(8),
            nn.ReLU(),
            nn.MaxPool1d(2),
            nn.Conv1d(8, 16, kernel_size=3, padding=1),
            nn.BatchNorm1d(16),
            nn.ReLU(),
            nn.MaxPool1d(2),
            nn.Conv1d(16, 32, kernel_size=3, padding=1),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(16),
            nn.Flatten(),
        )

        # Metadata backbone: narrow MLP
        self.meta_backbone = nn.Sequential(
            nn.Linear(metadata_dim, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.Linear(64, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(),
        )

        # Fusion
        combined_dim = 32 * 16 + 128  # 640
        self.fusion = nn.Sequential(
            nn.Linear(combined_dim, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(),
        )

        # Multi-head outputs
        self.wb_head = nn.Sequential(nn.Linear(128, 32), nn.ReLU(), nn.Linear(32, 3))
        self.ccm_head = nn.Sequential(nn.Linear(128, 64), nn.ReLU(), nn.Linear(64, 9))
        self.tone_head = nn.Sequential(nn.Linear(128, 32), nn.ReLU(), nn.Linear(32, 7))
        self.zoom_head = nn.Sequential(nn.Linear(128, 16), nn.ReLU(), nn.Linear(16, 1))

    def forward(self, histogram: torch.Tensor, metadata: torch.Tensor):
        hist_feat = self.hist_backbone(histogram.unsqueeze(1))
        meta_feat = self.meta_backbone(metadata)
        combined = torch.cat([hist_feat, meta_feat], dim=1)
        fused = self.fusion(combined)
        return {
            "wb": self.wb_head(fused),
            "ccm": self.ccm_head(fused),
            "tone": self.tone_head(fused),
            "zoom": self.zoom_head(fused),
        }


class ISPMediumModel(nn.Module):
    """
    Medium-weight Distilled ISP Controller Model (~350K params).
    ~3x smaller than full, ~3x larger than light. Balanced for most targets.
    Input:  histogram (B, 256) + metadata (B, 11)
    Output: wb (B, 3) + ccm (B, 9) + tone (B, 7) + zoom (B, 1)
    """

    def __init__(self, metadata_dim: int = 11):
        super().__init__()

        self.hist_backbone = nn.Sequential(
            nn.Conv1d(1, 12, kernel_size=7, padding=3),
            nn.BatchNorm1d(12),
            nn.ReLU(),
            nn.MaxPool1d(2),
            nn.Conv1d(12, 24, kernel_size=5, padding=2),
            nn.BatchNorm1d(24),
            nn.ReLU(),
            nn.MaxPool1d(2),
            nn.Conv1d(24, 48, kernel_size=3, padding=1),
            nn.BatchNorm1d(48),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(16),
            nn.Flatten(),
        )

        self.meta_backbone = nn.Sequential(
            nn.Linear(metadata_dim, 96),
            nn.BatchNorm1d(96),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(96, 192),
            nn.BatchNorm1d(192),
            nn.ReLU(),
            nn.Dropout(0.1),
        )

        combined_dim = 48 * 16 + 192
        self.fusion = nn.Sequential(
            nn.Linear(combined_dim, 256),
            nn.BatchNorm1d(256),
            nn.ReLU(),
            nn.Dropout(0.1),
        )

        self.wb_head = nn.Sequential(nn.Linear(256, 48), nn.ReLU(), nn.Linear(48, 3))
        self.ccm_head = nn.Sequential(nn.Linear(256, 96), nn.ReLU(), nn.Linear(96, 9))
        self.tone_head = nn.Sequential(nn.Linear(256, 48), nn.ReLU(), nn.Linear(48, 7))
        self.zoom_head = nn.Sequential(nn.Linear(256, 24), nn.ReLU(), nn.Linear(24, 1))

    def forward(self, histogram: torch.Tensor, metadata: torch.Tensor):
        hist_feat = self.hist_backbone(histogram.unsqueeze(1))
        meta_feat = self.meta_backbone(metadata)
        combined = torch.cat([hist_feat, meta_feat], dim=1)
        fused = self.fusion(combined)
        return {
            "wb": self.wb_head(fused),
            "ccm": self.ccm_head(fused),
            "tone": self.tone_head(fused),
            "zoom": self.zoom_head(fused),
        }


def export_onnx(model, output_path, metadata_dim=11, opset=13):
    """Export to ONNX."""
    model.eval()
    dummy_hist = torch.randn(1, 256)
    dummy_meta = torch.randn(1, metadata_dim)

    output = model(dummy_hist, dummy_meta)
    output_names = list(output.keys())
    dynamic_axes = {"histogram": {0: "batch"}, "metadata": {0: "batch"}}
    for name in output_names:
        dynamic_axes[name] = {0: "batch"}

    torch.onnx.export(
        model,
        (dummy_hist, dummy_meta),
        output_path,
        input_names=["histogram", "metadata"],
        output_names=output_names,
        dynamic_axes=dynamic_axes,
        opset_version=opset,
        do_constant_folding=True,
    )

    import onnx
    onnx_model = onnx.load(output_path)
    onnx.checker.check_model(onnx_model)
    # Re-save to ensure self-contained (no external .onnx.data)
    onnx.save(onnx_model, output_path)
    size_kb = os.path.getsize(output_path) / 1024
    print(f"✅ FP32 saved: {output_path} ({size_kb:.1f} KB)")
    return output_path


def quantize_int8(fp32_path, int8_path):
    """Quantize to INT8."""
    from onnxruntime.quantization import quantize_dynamic, QuantType
    quantize_dynamic(fp32_path, int8_path, weight_type=QuantType.QInt8)
    size_kb = os.path.getsize(int8_path) / 1024
    print(f"✅ INT8 saved: {int8_path} ({size_kb:.1f} KB)")
    return int8_path


def quantize_fp16(fp32_path, fp16_path):
    """Quantize to FP16."""
    from onnxconverter_common import float16
    import onnx
    model = onnx.load(fp32_path)
    model_fp16 = float16.convert_float_to_float16(model)
    onnx.save(model_fp16, fp16_path)
    size_kb = os.path.getsize(fp16_path) / 1024
    print(f"✅ FP16 saved: {fp16_path} ({size_kb:.1f} KB)")
    return fp16_path


def benchmark(model_path):
    """Quick latency benchmark."""
    import onnxruntime as ort
    sess = ort.InferenceSession(model_path)
    hist = np.zeros((1, 256), dtype=np.float32)
    hist[0, 128] = 10000.0
    meta = np.array([[0.5, 1.0, 1.0, 1.0, 0.1, 0.5, 0.5, 0.7, 0.5, 0.3, 0.1]], dtype=np.float32)

    for _ in range(10):
        sess.run(None, {"histogram": hist, "metadata": meta})

    times = []
    for _ in range(100):
        t0 = time.perf_counter()
        out = sess.run(None, {"histogram": hist, "metadata": meta})
        times.append((time.perf_counter() - t0) * 1000)

    out = out[0]
    print(f"   Latency: {np.mean(times):.2f} ms avg, {np.std(times):.2f} ms std")
    print(f"   WB:   {out[0, :3].round(3).tolist()}")
    print(f"   CCM:  diag={out[0, [0,4,8]].round(3).tolist()}")
    print(f"   Tone: {out[0, 12:19].round(3).tolist()}")
    print(f"   Zoom: {out[0, 19]:.3f}")


def main():
    parser = argparse.ArgumentParser(description="Generate mock ISP Rectifier ONNX")
    parser.add_argument("-o", "--output", default="models/fusedispcontroller.onnx")
    parser.add_argument("--int8", action="store_true", help="Also generate INT8 quantized")
    parser.add_argument("--fp16", action="store_true", help="Also generate FP16 quantized")
    parser.add_argument("--all", action="store_true", help="Generate FP32 + INT8 + FP16")
    parser.add_argument("--benchmark", action="store_true", help="Run latency benchmark")
    parser.add_argument("--metadata-dim", type=int, default=11, help="Metadata input dim (default: 11)")
    parser.add_argument("--light", action="store_true", help="Use lightweight model (~118K params)")
    parser.add_argument("--medium", action="store_true", help="Use medium-weight model (~350K params)")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    base = args.output.replace(".onnx", "")

    if args.light:
        model_cls = ISPLightModel
        model_type = "light"
    elif args.medium:
        model_cls = ISPMediumModel
        model_type = "medium"
    else:
        model_cls = ISPDistilledModel
        model_type = "full"
    print(f"Creating mock ISP Rectifier model ({model_type})...")
    model = model_cls(metadata_dim=args.metadata_dim)

    # Count params
    n_params = sum(p.numel() for p in model.parameters())
    print(f"   Parameters: {n_params:,} ({n_params * 4 / 1024:.1f} KB)")

    # Export FP32
    export_onnx(model, args.output, metadata_dim=args.metadata_dim)

    # Quantize
    if args.all or args.int8:
        try:
            quantize_int8(args.output, f"{base}_int8.onnx")
        except Exception as e:
            print(f"⚠️  INT8 failed: {e}")

    if args.all or args.fp16:
        try:
            quantize_fp16(args.output, f"{base}_fp16.onnx")
        except Exception as e:
            print(f"⚠️  FP16 failed: {e}")

    # Benchmark
    if args.benchmark or args.all:
        print(f"\nBenchmarking {args.output}:")
        try:
            benchmark(args.output)
        except Exception as e:
            print(f"⚠️  Benchmark failed: {e}")

    print("\nDone!")


if __name__ == "__main__":
    main()
