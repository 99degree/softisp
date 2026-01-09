# test_integration_coeff_first.py
import onnxruntime as ort
import numpy as np
import time
from datetime import datetime

# Helper to pack scalars into [1,1,1,1]
def pack_scalar(x):
    return np.array([[[[x]]]], dtype=np.float32)

# Generate synthetic Bayer frame
def make_bayer_frame(h=1080, w=1920, seed=None):
    rng = np.random.default_rng(seed)
    return rng.random((1,1,h,w), dtype=np.float32)

# Load sessions
coeff_sess = ort.InferenceSession("isp_coeff_controller.onnx", providers=["CPUExecutionProvider"])
isp_sess   = ort.InferenceSession("isp_pipeline_3a.onnx", providers=["CPUExecutionProvider"])

# Initial coefficients (starting point)
ae_gain = pack_scalar(1.0)
awb_gains = np.array([[[[1.0]], [[1.0]], [[1.0]]]], dtype=np.float32)
ccm = np.eye(3, dtype=np.float32).reshape(3,3,1,1)
gamma = pack_scalar(1/2.2)
lens_coeffs = np.array([[0.0,0.0,0.0,0.0]], dtype=np.float32)
noise_strength = pack_scalar(1.0)
sharp_strength = pack_scalar(1.0)
color_coeffs = np.array([[1.0,1.0,0.0]], dtype=np.float32)

# Run 5 frames
for i in range(5):
    bayer = make_bayer_frame(seed=i)

    ts_start = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    t0 = time.perf_counter()

    # Step 1: run coeff controller with Bayer + current coeffs
    coeff_inputs = {
        "AE.meanY": pack_scalar(0.4),   # placeholder measurement if controller expects it
        "AWB.Rmean": pack_scalar(0.5),
        "AWB.Gmean": pack_scalar(0.5),
        "AWB.Bmean": pack_scalar(0.5),
        "AF.score": pack_scalar(0.1),
        "ae.gain": ae_gain,
        "awb.gains": awb_gains,
        "ccm": ccm,
        "gamma": gamma,
        "lens.coeffs": lens_coeffs,
        "noise.strength": noise_strength,
        "sharp.strength": sharp_strength,
        "color.coeffs": color_coeffs
    }
    coeff_outs = coeff_sess.run(None, coeff_inputs)
    (ae_gain_next, awb_gains_next, ccm_next, gamma_next,
     lens_next, noise_next, sharp_next, color_next) = coeff_outs

    # Step 2: run ISP pipeline with Bayer + updated coeffs
    isp_inputs = {
        "input.bayer": bayer,
        "ae.gain": ae_gain_next,
        "awb.gains": awb_gains_next
        # other coeffs can be wired in if your ISP graph accepts them
    }
    isp_outs = isp_sess.run(None, isp_inputs)
    rgb_out, y_out, uv_out, ae_meas, awb_r, awb_g, awb_b, af_score = isp_outs

    t1 = time.perf_counter()
    ts_end = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

    # Print results
    print(f"Frame {i+1}:")
    print(f"  Start: {ts_start}")
    print(f"  End:   {ts_end}")
    print(f"  Process time: {(t1 - t0)*1000:.2f} ms")
    print(f"  AE.gain_next: {ae_gain_next.reshape(-1)[0]:.3f}")
    print(f"  AWB.gains_next: {awb_gains_next.reshape(3)}")
    print(f"  AF.score (ISP): {af_score.reshape(-1)[0]:.3f}")
    print("-"*40)

    # Update coefficients for next iteration
    ae_gain = ae_gain_next
    awb_gains = awb_gains_next
    ccm = ccm_next
    gamma = gamma_next
    lens_coeffs = lens_next
    noise_strength = noise_next
    sharp_strength = sharp_next
    color_coeffs = color_next
