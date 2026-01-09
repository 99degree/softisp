import onnxruntime as ort
import numpy as np
import sys

def run_ruleengine(model_path="ruleengine_full.onnx"):
    # Load ONNX model
    sess = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])

    # Build dummy inputs
    inputs = {
        "raw_wb": np.array([1.2, 0.9, 1.1], dtype=np.float32),
        "prev_wb": np.array([1.0, 1.0, 1.0], dtype=np.float32),
        "raw_ccm": np.eye(3, dtype=np.float32).flatten(),
        "prev_ccm": np.eye(3, dtype=np.float32).flatten(),
        "raw_gamma": np.linspace(0, 1, 256).astype(np.float32),
        "prev_gamma": np.linspace(0, 1, 256).astype(np.float32),
        "raw_sharpness": np.array([1.0], dtype=np.float32),
        "prev_sharpness": np.array([1.0], dtype=np.float32),
        "raw_nr": np.array([0.5], dtype=np.float32),
        "prev_nr": np.array([0.5], dtype=np.float32),

        # Sensor meta
        "analog_gain": np.array([2.0], dtype=np.float32),
        "exposure_time": np.array([0.01], dtype=np.float32),
        "sensor_temp": np.array([40.0], dtype=np.float32),
        "scene_change": np.array([0.0], dtype=np.float32),

        # Rule params
        "alpha_wb": np.array([0.2], dtype=np.float32),
        "alpha_ccm": np.array([0.2], dtype=np.float32),
        "alpha_gamma": np.array([0.2], dtype=np.float32),
        "alpha_sharp": np.array([0.2], dtype=np.float32),
        "alpha_nr": np.array([0.2], dtype=np.float32),
        "alpha_fast": np.array([0.5], dtype=np.float32),

        "wb_step": np.array([0.05], dtype=np.float32),
        "sharp_step": np.array([0.05], dtype=np.float32),

        "wb_min": np.array([0.5], dtype=np.float32),
        "wb_max": np.array([2.0], dtype=np.float32),
        "gamma_min": np.array([0.0], dtype=np.float32),
        "gamma_max": np.array([1.0], dtype=np.float32),
        "ccm_min": np.array([-2.0], dtype=np.float32),
        "ccm_max": np.array([2.0], dtype=np.float32),
        "nr_min": np.array([0.0], dtype=np.float32),
        "nr_max": np.array([5.0], dtype=np.float32),
    }

    # Run inference
    outputs = sess.run(None, inputs)

    # Collect outputs
    wb_stab, ccm_stab, gamma_stab, sharpness_stab, nr_stab = outputs

    # Basic sanity checks
    assert wb_stab.shape == (3,)
    assert ccm_stab.shape == (9,)
    assert gamma_stab.shape == (256,)
    assert sharpness_stab.shape == (1,)
    assert nr_stab.shape == (1,)

    # Check ranges
    assert np.all(wb_stab >= 0.5) and np.all(wb_stab <= 2.0)
    assert np.all(gamma_stab >= 0.0) and np.all(gamma_stab <= 1.0)
    assert np.all(ccm_stab >= -2.0) and np.all(ccm_stab <= 2.0)
    assert sharpness_stab[0] >= 0.0 and sharpness_stab[0] <= 1.0
    assert nr_stab[0] >= 0.0 and nr_stab[0] <= 5.0

    print("RuleEngine ONNX test passed ✅")
    print("wb_stab:", wb_stab)
    print("sharpness_stab:", sharpness_stab)
    print("nr_stab:", nr_stab)

if __name__ == "__main__":
    model_path = sys.argv[1] if len(sys.argv) > 1 else "ruleengine_full.onnx"
    run_ruleengine(model_path)
