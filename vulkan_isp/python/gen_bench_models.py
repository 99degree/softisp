#!/usr/bin/env python3
"""Generate ISP ONNX models for benchmarking fused_6in1."""
import sys, os, subprocess

sizes = [
    ("HD",  1280, 720),
    ("FHD", 1920, 1080),
    ("4K",  3840, 2160),
]

gen_script = os.path.expanduser("~/softisp/vulkan_isp/gen_isp_onnx_standard.py")
mnn_convert = os.path.expanduser("~/MNN/build_vk/MNNConvert")

for name, w, h in sizes:
    bW, bH = w * 2, h * 2
    onnx_path = os.path.expanduser(f"~/tmp/bench_f6_{name}.onnx")
    mnn_path = os.path.expanduser(f"~/tmp/bench_f6_{name}.mnn")
    
    subprocess.run([sys.executable, gen_script,
                    "--bayer-width", str(w), "--bayer-height", str(h),
                    "-o", onnx_path], check=True, capture_output=True)
    subprocess.run([mnn_convert, "-f", "ONNX",
                    "--modelFile", onnx_path,
                    "--MNNModel", mnn_path,
                    "--fp16", "0"], check=True, capture_output=True)
    print(f"  {name}: {w}×{h} → {bW}×{bH} Bayer → {mnn_path}")
