#!/usr/bin/env python3
"""
Create a dummy test ONNX model for pipeline validation.

Input:  [batch, 267] (histogram[256] + metadata[11])
Output: [batch, 20]  (wb[3] + ccm[9] + tone[7] + zoom[1])
"""

import numpy as np

try:
    import onnx
    from onnx import helper, TensorProto
except ImportError:
    print("Installing onnx...")
    import subprocess
    subprocess.check_call(["pip", "install", "onnx"])
    import onnx
    from onnx import helper, TensorProto


def create_test_onnx(output_path="models/fusedispcontroller_test.onnx"):
    """Create a simple ONNX model that mimics the ISP Rectifier architecture."""

    # Input tensor
    X = helper.make_tensor_value_info("input", TensorProto.FLOAT, [None, 267])

    # Output tensor
    Y = helper.make_tensor_value_info("output", TensorProto.FLOAT, [None, 20])

    # Initialize weights with simple identity-like transformation
    np.random.seed(42)
    W1 = np.random.randn(267, 64).astype(np.float32) * 0.01
    B1 = np.zeros(64, dtype=np.float32)
    W2 = np.random.randn(64, 20).astype(np.float32) * 0.01
    B2 = np.zeros(20, dtype=np.float32)

    # Ensure tone curve output (indices 12-18) is in [0,1]
    # and WB gains (indices 0-2) are positive
    # Simple approach: use sigmoid for tone, exp for WB

    # Nodes
    fc1 = helper.make_node("MatMul", ["input", "W1"], ["h1"], name="fc1")
    add1 = helper.make_node("Add", ["h1", "B1"], ["h1b"], name="add1")
    relu1 = helper.make_node("Relu", ["h1b"], ["h2"], name="relu1")
    fc2 = helper.make_node("MatMul", ["h2", "W2"], ["out_raw"], name="fc2")
    add2 = helper.make_node("Add", ["out_raw", "B2"], ["output"], name="add2")

    # Initializers (weights)
    W1_init = helper.make_tensor("W1", TensorProto.FLOAT, [267, 64], W1.flatten().tolist())
    B1_init = helper.make_tensor("B1", TensorProto.FLOAT, [64], B1.tolist())
    W2_init = helper.make_tensor("W2", TensorProto.FLOAT, [64, 20], W2.flatten().tolist())
    B2_init = helper.make_tensor("B2", TensorProto.FLOAT, [20], B2.tolist())

    # Graph
    graph = helper.make_graph(
        [fc1, add1, relu1, fc2, add2],
        "isp_rectifier_test",
        [X],
        [Y],
        [W1_init, B1_init, W2_init, B2_init],
    )

    # Model
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.doc_string = "Test ONNX model for ISP Rectifier pipeline validation (267→20)"

    onnx.checker.check_model(model)
    onnx.save(model, output_path)
    print(f"✅ Test model saved to {output_path}")
    print(f"   Input:  [batch, 267]")
    print(f"   Output: [batch, 20]")
    return output_path


if __name__ == "__main__":
    create_test_onnx()
