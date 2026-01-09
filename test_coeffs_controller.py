# test_coeff_controller.py
import onnxruntime as ort
import numpy as np

# Helper to pack scalars into [1,1,1,1]
def pack_scalar(x):
    return np.array([[[[x]]]], dtype=np.float32)

# Load controller model
sess = ort.InferenceSession("isp_coeff_controller.onnx", providers=["CPUExecutionProvider"])

# Example measurements
inputs = {
    "AE.meanY":    pack_scalar(0.35),
    "AWB.Rmean":   pack_scalar(0.40),
    "AWB.Gmean":   pack_scalar(0.60),
    "AWB.Bmean":   pack_scalar(0.50),
    "AF.score":    pack_scalar(0.12),
    # Current coefficients
    "ae.gain":     pack_scalar(1.00),
    "awb.gains":   np.array([[[[1.0]], [[1.0]], [[1.0]]]], dtype=np.float32),  # [1,3,1,1]
    "ccm":         np.eye(3, dtype=np.float32).reshape(3,3,1,1),
    "gamma":       pack_scalar(1/2.2),
    "lens.coeffs": np.array([[0.0, 0.0, 0.0, 0.0]], dtype=np.float32),  # radial coeffs placeholder
    "noise.strength": pack_scalar(1.0),
    "sharp.strength": pack_scalar(1.0),
    "color.coeffs": np.array([[1.0, 1.0, 0.0]], dtype=np.float32)  # contrast, saturation, hue
}

# Run controller
outs = sess.run(None, inputs)

# Unpack outputs
ae_next, awb_next, ccm_next, gamma_next, lens_next, noise_next, sharp_next, color_next = outs

print("Updated coefficients:")
print(f"  AE.gain_next: {ae_next.reshape(-1)[0]:.3f}")
print(f"  AWB.gains_next: {awb_next.reshape(3)}")
print(f"  CCM_next:\n{ccm_next.reshape(3,3)}")
print(f"  Gamma_next: {gamma_next.reshape(-1)[0]:.3f}")
print(f"  Lens.coeffs_next: {lens_next.reshape(-1)}")
print(f"  Noise.strength_next: {noise_next.reshape(-1)[0]:.3f}")
print(f"  Sharpen.strength_next: {sharp_next.reshape(-1)[0]:.3f}")
print(f"  Color.coeffs_next: {color_next.reshape(-1)}")
