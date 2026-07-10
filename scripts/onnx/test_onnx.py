import onnxruntime as ort
import numpy as np
import time
from datetime import datetime

# --------------------------
# Load ONNX model
# --------------------------
session = ort.InferenceSession("isp_pipeline.onnx", providers=["CPUExecutionProvider"])

# Inspect input name
input_name = session.get_inputs()[0].name
print("Model input:", input_name)

# --------------------------
# Generate synthetic Bayer frames
# --------------------------
def make_bayer_frame(h=1080, w=1920, seed=None):
    rng = np.random.default_rng(seed)
    # Bayer pattern: single channel, values 0..1
    frame = rng.random((1, 1, h, w), dtype=np.float32)
    return frame

# --------------------------
# Run 5 test cases
# --------------------------
for i in range(5):
    # Create a new random Bayer frame each iteration
    bayer = make_bayer_frame(seed=i)

    # Timestamp before inference
    ts_start = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

    t0 = time.perf_counter()
    outputs = session.run(None, {input_name: bayer})
    t1 = time.perf_counter()

    # Timestamp after inference
    ts_end = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

    print(f"Sample {i+1}:")
    print(f"  Start: {ts_start}")
    print(f"  End:   {ts_end}")
    print(f"  Process time: {(t1 - t0)*1000:.2f} ms")
    print(f"  Outputs: {[o.shape for o in outputs]}")
    print("-"*40)
