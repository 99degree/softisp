# test_integration_algos.py
import onnxruntime as ort
import numpy as np
import time
from datetime import datetime

def make_bayer_frame(h=1080, w=1920, seed=None):
    """Generate a synthetic Bayer frame of the correct resolution."""
    rng = np.random.default_rng(seed)
    return rng.random((1,1,h,w), dtype=np.float32)

# Load sessions
algo_sess = ort.InferenceSession("isp_algo_coeffs_from_bayer.onnx", providers=["CPUExecutionProvider"])
isp_sess  = ort.InferenceSession("isp_pipeline_3a.onnx", providers=["CPUExecutionProvider"])

# Discover pipeline input names
pipeline_inputs = {inp.name for inp in isp_sess.get_inputs()}
print("ISP pipeline expects inputs:", pipeline_inputs)

algo_times = []
isp_times = []

# Run a few frames
for i in range(5):
    bayer = make_bayer_frame(seed=i)

    ts_start = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

    # Step 1: run algorithm ONNX to compute coefficients
    t0 = time.perf_counter()
    algo_outs = algo_sess.run(None, {"input.bayer": bayer})
    (ae_gain_next, awb_gains_next, ccm_next, gamma_next,
     lens_next, noise_next, sharp_next, color_next) = algo_outs
    t1 = time.perf_counter()
    algo_times.append((t1 - t0)*1000)

    # Step 2: build ISP inputs dynamically based on supported names
    isp_inputs = {}
    if "input.bayer" in pipeline_inputs:
        isp_inputs["input.bayer"] = bayer
    if "ae.gain" in pipeline_inputs:
        isp_inputs["ae.gain"] = ae_gain_next
    if "awb.gains" in pipeline_inputs:
        isp_inputs["awb.gains"] = awb_gains_next
    if "ccm" in pipeline_inputs:
        isp_inputs["ccm"] = ccm_next
    if "gamma" in pipeline_inputs:
        isp_inputs["gamma"] = gamma_next
    if "lens.coeffs" in pipeline_inputs:
        isp_inputs["lens.coeffs"] = lens_next
    if "noise.strength" in pipeline_inputs:
        isp_inputs["noise.strength"] = noise_next
    if "sharp.strength" in pipeline_inputs:
        isp_inputs["sharp.strength"] = sharp_next
    if "color.coeffs" in pipeline_inputs:
        isp_inputs["color.coeffs"] = color_next

    # Step 3: run ISP pipeline
    t2 = time.perf_counter()
    isp_outs = isp_sess.run(None, isp_inputs)
    t3 = time.perf_counter()
    isp_times.append((t3 - t2)*1000)

    ts_end = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

    # Print results
    print(f"Frame {i+1}:")
    print(f"  Start: {ts_start}")
    print(f"  End:   {ts_end}")
    print(f"  Algo duration: {algo_times[-1]:.2f} ms")
    print(f"  ISP duration:  {isp_times[-1]:.2f} ms")
    print(f"  AE.gain_next: {ae_gain_next.reshape(-1)[0]:.3f}")
    print(f"  AWB.gains_next: {awb_gains_next.reshape(3)}")
    print(f"  Gamma: {gamma_next.reshape(-1)[0]:.3f}")
    print(f"  Noise strength: {noise_next.reshape(-1)[0]:.3f}")
    if "sharp.strength" in pipeline_inputs:
        print(f"  Sharp strength: {sharp_next.reshape(-1)[0]:.3f}")
    print(f"  Color coeffs: {color_next.reshape(3)}")
    print("-"*50)

# Print averages
print("Average timings over 5 frames:")
print(f"  Algo avg: {np.mean(algo_times):.2f} ms")
print(f"  ISP avg:  {np.mean(isp_times):.2f} ms")
