#!/usr/bin/env python3
"""
Generate a mock ISP Rectifier ONNX model for testing.

Usage:
    python gen_mock_onnx.py                          # Default: models/fusedispcontroller.onnx
    python gen_mock_onnx.py -o my_model.onnx        # Custom output path
    python gen_mock_onnx.py --test                   # Generate + run quick test

Requirements:
    pip install onnx numpy

Input:  [batch, 267]  (histogram[256] + metadata[11])
Output: [batch, 20]   (wb[3] + ccm[9] + tone[7] + zoom[1])
"""

import argparse
import sys
import os
import time

try:
    import numpy as np
except ImportError:
    print("ERROR: numpy required. Run: pip install numpy")
    sys.exit(1)

try:
    import onnx
    from onnx import helper, TensorProto, checker
except ImportError:
    print("ERROR: onnx required. Run: pip install onnx")
    sys.exit(1)


def create_mock_model(output_path: str = "models/fusedispcontroller.onnx") -> str:
    """Create a mock ONNX model that mimics ISP Rectifier (267→20)."""

    print(f"Creating mock ONNX model: {output_path}")

    # --- Input/Output definitions ---
    X = helper.make_tensor_value_info("input", TensorProto.FLOAT, [None, 267])
    Y = helper.make_tensor_value_info("output", TensorProto.FLOAT, [None, 20])

    # --- Weights: 2-layer MLP (267→64→20) ---
    np.random.seed(42)
    W1 = (np.random.randn(267, 64) * 0.05).astype(np.float32)
    B1 = np.zeros(64, dtype=np.float32)
    W2 = (np.random.randn(64, 20) * 0.05).astype(np.float32)
    B2 = np.zeros(20, dtype=np.float32)

    # Bias B2 to produce reasonable defaults:
    # WB: exp(0)≈1.0, CCM: ≈identity, Tone: ≈linear, Zoom: ≈1.0
    B2[0] = 0.0    # wb_r
    B2[1] = 0.0    # wb_g
    B2[2] = 0.0    # wb_b
    B2[3] = 1.0    # ccm[0] ≈ 1.0 (identity)
    B2[7] = 1.0    # ccm[4] ≈ 1.0
    B2[11] = 1.0   # ccm[8] ≈ 1.0
    for i in range(7):
        B2[12 + i] = i / 6.0  # tone: linear [0, 1/6, ..., 1]
    B2[19] = 0.0   # zoom → relu(x+0) → ~1.0

    # --- Graph nodes ---
    # Layer 1: h1 = relu(input @ W1 + B1)
    fc1 = helper.make_node("MatMul", ["input", "W1"], ["h1_raw"], name="fc1")
    add1 = helper.make_node("Add", ["h1_raw", "B1"], ["h1"], name="add1")
    relu1 = helper.make_node("Relu", ["h1"], ["h1_relu"], name="relu1")

    # Layer 2: out = h1_relu @ W2 + B2
    fc2 = helper.make_node("MatMul", ["h1_relu", "W2"], ["out_raw"], name="fc2")
    add2 = helper.make_node("Add", ["out_raw", "B2"], ["output"], name="add2")

    nodes = [fc1, add1, relu1, fc2, add2]

    # --- Initializers ---
    init_W1 = helper.make_tensor("W1", TensorProto.FLOAT, [267, 64], W1.flatten().tolist())
    init_B1 = helper.make_tensor("B1", TensorProto.FLOAT, [64], B1.tolist())
    init_W2 = helper.make_tensor("W2", TensorProto.FLOAT, [64, 20], W2.flatten().tolist())
    init_B2 = helper.make_tensor("B2", TensorProto.FLOAT, [20], B2.tolist())

    # --- Graph & Model ---
    graph = helper.make_graph(
        nodes,
        "isp_rectifier_mock",
        [X],
        [Y],
        [init_W1, init_B1, init_W2, init_B2],
    )

    model = helper.make_model(
        graph,
        opset_imports=[helper.make_opsetid("", 13)],
        ir_version=8,
    )
    model.doc_string = (
        "Mock ISP Rectifier model for pipeline testing.\n"
        "Input: [batch, 267] = histogram[256] + metadata[11]\n"
        "Output: [batch, 20] = wb[3] + ccm[9] + tone[7] + zoom[1]"
    )
    model.producer_name = "isp-rectifier-mock"

    # --- Validate & Save ---
    checker.check_model(model)

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    onnx.save(model, output_path)
    size_kb = os.path.getsize(output_path) / 1024

    print(f"✅ Saved: {output_path} ({size_kb:.1f} KB)")
    print(f"   Input:  [batch, 267]")
    print(f"   Output: [batch, 20]")
    return output_path


def test_model(model_path: str) -> bool:
    """Quick smoke test on the mock model."""
    try:
        import onnxruntime as ort
    except ImportError:
        print("onnxruntime not installed, skipping test")
        return True

    print(f"\nTesting: {model_path}")
    sess = ort.InferenceSession(model_path)

    # Synthetic input: neutral gray scene
    hist = np.zeros((1, 256), dtype=np.float32)
    hist[128] = 10000.0  # peak at mid-gray
    meta = np.array([[0.5, 1.0, 1.0, 1.0, 0.1, 0.5, 0.5, 0.7, 0.5, 0.3, 0.1]], dtype=np.float32)
    inputs = np.concatenate([hist, meta], axis=1)  # [1, 267]

    # Warmup + benchmark
    for _ in range(5):
        sess.run(None, {"input": inputs})

    times = []
    for _ in range(50):
        t0 = time.perf_counter()
        outputs = sess.run(None, {"input": inputs})
        times.append((time.perf_counter() - t0) * 1000)

    out = outputs[0][0]
    wb = out[:3]
    ccm_diag = [out[3], out[7], out[11]]
    tone = out[12:19]
    zoom = out[19]

    print(f"   Latency: {np.mean(times):.2f} ms avg")
    print(f"   WB gains:  [{wb[0]:.3f}, {wb[1]:.3f}, {wb[2]:.3f}]")
    print(f"   CCM diag:  [{ccm_diag[0]:.3f}, {ccm_diag[1]:.3f}, {ccm_diag[2]:.3f}]")
    print(f"   Tone LUT:  [{', '.join(f'{v:.3f}' for v in tone)}]")
    print(f"   Zoom:      {zoom:.3f}")

    # Sanity checks
    assert outputs[0].shape == (1, 20), f"Expected shape (1,20), got {outputs[0].shape}"
    assert all(0.1 < g < 5.0 for g in wb), f"WB gains out of range: {wb}"
    assert 0.5 < zoom < 3.0, f"Zoom out of range: {zoom}"

    print("✅ All checks passed!")
    return True


def main():
    parser = argparse.ArgumentParser(description="Generate mock ISP Rectifier ONNX model")
    parser.add_argument("-o", "--output", default="models/fusedispcontroller.onnx",
                        help="Output ONNX path (default: models/fusedispcontroller.onnx)")
    parser.add_argument("--test", action="store_true",
                        help="Run quick inference test after generation")
    args = parser.parse_args()

    create_mock_model(args.output)

    if args.test:
        test_model(args.output)


if __name__ == "__main__":
    main()
