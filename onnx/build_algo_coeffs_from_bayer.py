# build_algo_coeffs_from_bayer.py
import onnx
from onnx import helper, TensorProto
import numpy as np

def make_scalar_const(name, value, dtype=np.float32):
    arr = np.array([value], dtype=dtype)
    return helper.make_tensor(name=name, data_type=TensorProto.FLOAT if dtype==np.float32 else TensorProto.INT64,
                              dims=[1], vals=arr)

def make_int64_const(name, values):
    arr = np.array(values, dtype=np.int64)
    return helper.make_tensor(name=name, data_type=TensorProto.INT64, dims=[len(values)], vals=arr)

def build_algo_coeffs():
    # Input: Bayer image with symbolic dimensions (N,C,H,W)
    bayer_in = helper.make_tensor_value_info('input.bayer', TensorProto.FLOAT, ['N','C','H','W'])

    nodes, inits = [], []

    # --- RGB proxy: replicate Bayer to 3 channels (placeholder demosaic proxy)
    nodes.append(helper.make_node('Concat', ['input.bayer','input.bayer','input.bayer'], ['RGB'], axis=1))

    # --- Luminance Y = M709 * RGB (Conv with 1x1 weights)
    M709 = np.array([[0.2126, 0.7152, 0.0722]], dtype=np.float32).reshape(1,3,1,1)
    W_y = helper.make_tensor('Y.W', TensorProto.FLOAT, M709.shape, M709.ravel())
    inits.append(W_y)
    nodes.append(helper.make_node('Conv', ['RGB','Y.W'], ['Y'], name='rgb2y'))

    # --- AE: target vs mean luminance -> proportional gain
    # y_mean = ReduceMean(Y) over all dims -> scalar
    nodes.append(helper.make_node('ReduceMean', ['Y'], ['y_mean'], keepdims=0))
    inits += [
        make_scalar_const('AE.target', 0.18),   # mid-grey target
        make_scalar_const('AE.Kp', 0.7),        # proportional term
        make_scalar_const('AE.zero', 0.0)
    ]
    nodes.append(helper.make_node('Sub', ['AE.target','y_mean'], ['ae_err']))
    nodes.append(helper.make_node('Mul', ['AE.Kp','ae_err'], ['ae_delta']))
    nodes.append(helper.make_node('Add', ['ae_delta','AE.zero'], ['ae_next_scalar']))
    # shape to [1,1,1,1]
    ae_shape = helper.make_tensor('ae.shape', TensorProto.INT64, [4], np.array([1,1,1,1],dtype=np.int64))
    inits.append(ae_shape)
    nodes.append(helper.make_node('Reshape', ['ae_next_scalar','ae.shape'], ['ae.gain_next']))

    # --- AWB: per-channel averages via ReduceMean over H,W
    # rgb_mean_hw: [N,3,1,1]
    inits.append(make_int64_const('axes_hw', [2,3]))
    nodes.append(helper.make_node('ReduceMean', ['RGB'], ['rgb_mean_hw'], axes=[2,3], keepdims=1))
    # Slice R,G,B along channel axis=1
    starts_r = helper.make_tensor('starts_r', TensorProto.INT64, [1], np.array([0], dtype=np.int64))
    ends_r   = helper.make_tensor('ends_r',   TensorProto.INT64, [1], np.array([1], dtype=np.int64))
    axes_c   = helper.make_tensor('axes_c',   TensorProto.INT64, [1], np.array([1], dtype=np.int64))
    starts_g = helper.make_tensor('starts_g', TensorProto.INT64, [1], np.array([1], dtype=np.int64))
    ends_g   = helper.make_tensor('ends_g',   TensorProto.INT64, [1], np.array([2], dtype=np.int64))
    starts_b = helper.make_tensor('starts_b', TensorProto.INT64, [1], np.array([2], dtype=np.int64))
    ends_b   = helper.make_tensor('ends_b',   TensorProto.INT64, [1], np.array([3], dtype=np.int64))
    inits += [starts_r, ends_r, axes_c, starts_g, ends_g, starts_b, ends_b]
    nodes.append(helper.make_node('Slice', ['rgb_mean_hw','starts_r','ends_r','axes_c'], ['r_mean']))
    nodes.append(helper.make_node('Slice', ['rgb_mean_hw','starts_g','ends_g','axes_c'], ['g_mean']))
    nodes.append(helper.make_node('Slice', ['rgb_mean_hw','starts_b','ends_b','axes_c'], ['b_mean']))
    # AWB gains: relative to green (grey-world)
    nodes.append(helper.make_node('Div', ['g_mean','r_mean'], ['rgain']))
    nodes.append(helper.make_node('Identity', ['g_mean'], ['ggain']))
    nodes.append(helper.make_node('Div', ['g_mean','b_mean'], ['bgain']))
    nodes.append(helper.make_node('Concat', ['rgain','ggain','bgain'], ['awb_vec'], axis=0))
    awb_shape = helper.make_tensor('awb.shape', TensorProto.INT64, [4], np.array([1,3,1,1], dtype=np.int64))
    inits.append(awb_shape)
    nodes.append(helper.make_node('Reshape', ['awb_vec','awb.shape'], ['awb.gains_next']))

    # --- Adaptive Gamma: base + k * (target - y_mean), clamped
    inits += [
        make_scalar_const('gamma_base', 1/2.2),
        make_scalar_const('gamma_k', 0.35),
        helper.make_tensor('gamma_min', TensorProto.FLOAT, [1], np.array([0.25], dtype=np.float32)),
        helper.make_tensor('gamma_max', TensorProto.FLOAT, [1], np.array([1.2], dtype=np.float32))
    ]
    nodes.append(helper.make_node('Sub', ['AE.target','y_mean'], ['gamma_err']))
    nodes.append(helper.make_node('Mul', ['gamma_k','gamma_err'], ['gamma_adj']))
    nodes.append(helper.make_node('Add', ['gamma_base','gamma_adj'], ['gamma_raw']))
    nodes.append(helper.make_node('Clip', ['gamma_raw','gamma_min','gamma_max'], ['gamma_clamped']))
    nodes.append(helper.make_node('Reshape', ['gamma_clamped','ae.shape'], ['gamma_next']))

    # --- Noise estimate from luminance variance (simple proxy)
    # y_mean_hw: mean over H,W -> [N,1,1,1]
    nodes.append(helper.make_node('ReduceMean', ['Y'], ['y_mean_hw'], axes=[2,3], keepdims=1))
    nodes.append(helper.make_node('Sub', ['Y','y_mean_hw'], ['y_centered']))
    nodes.append(helper.make_node('Mul', ['y_centered','y_centered'], ['y_sq']))
    # variance over H,W -> scalar (keepdims=0)
    nodes.append(helper.make_node('ReduceMean', ['y_sq'], ['y_var'], keepdims=0))
    # noise strength = k_noise * y_var, clamped
    inits += [
        make_scalar_const('k_noise', 10.0),
        helper.make_tensor('noise_min', TensorProto.FLOAT, [1], np.array([0.0], dtype=np.float32)),
        helper.make_tensor('noise_max', TensorProto.FLOAT, [1], np.array([3.0], dtype=np.float32))
    ]
    nodes.append(helper.make_node('Mul', ['k_noise','y_var'], ['noise_raw']))
    nodes.append(helper.make_node('Clip', ['noise_raw','noise_min','noise_max'], ['noise_clamped']))
    nodes.append(helper.make_node('Reshape', ['noise_clamped','ae.shape'], ['noise.strength_next']))

    # --- Sharpness inversely tied to noise: sharp = sharp_base / (1 + alpha*noise)
    inits += [
        make_scalar_const('sharp_base', 1.0),
        make_scalar_const('sharp_alpha', 0.6)
    ]
    nodes.append(helper.make_node('Mul', ['sharp_alpha','noise_clamped'], ['noise_scaled']))
    nodes.append(helper.make_node('Add', ['noise_scaled','AE.zero'], ['one_plus_noise']))  # AE.zero=0; treat as +1 via Add after Const(1)
    # We need +1; create explicit 1.0 const to add
    inits.append(make_scalar_const('ONE', 1.0))
    nodes.append(helper.make_node('Add', ['noise_scaled','ONE'], ['denom']))
    nodes.append(helper.make_node('Div', ['sharp_base','denom'], ['sharp_raw']))
    nodes.append(helper.make_node('Reshape', ['sharp_raw','ae.shape'], ['sharp.strength_next']))

    # --- CCM (identity), Lens coeffs (zeros), Color coeffs (example tweak)
    ccm = np.eye(3, dtype=np.float32).reshape(3,3,1,1)
    ccm_t = helper.make_tensor('ccm_init', TensorProto.FLOAT, ccm.shape, ccm.ravel())
    inits.append(ccm_t)
    nodes.append(helper.make_node('Identity', ['ccm_init'], ['ccm_next']))

    lens_t = helper.make_tensor('lens_init', TensorProto.FLOAT, [4], np.zeros(4, dtype=np.float32))
    lens_shape = helper.make_tensor('lens.shape', TensorProto.INT64, [2], np.array([1,4], dtype=np.int64))
    inits += [lens_t, lens_shape]
    nodes.append(helper.make_node('Reshape', ['lens_init','lens.shape'], ['lens.coeffs_next']))

    color_t = helper.make_tensor('color_init', TensorProto.FLOAT, [3], np.array([1.0, 1.0, 0.0], dtype=np.float32))
    color_shape = helper.make_tensor('color.shape', TensorProto.INT64, [2], np.array([1,3], dtype=np.int64))
    inits += [color_t, color_shape]
    nodes.append(helper.make_node('Reshape', ['color_init','color.shape'], ['color.coeffs_next']))

    # --- Outputs
    outs = [
        helper.make_tensor_value_info('ae.gain_next',            TensorProto.FLOAT, [1,1,1,1]),
        helper.make_tensor_value_info('awb.gains_next',          TensorProto.FLOAT, [1,3,1,1]),
        helper.make_tensor_value_info('ccm_next',                TensorProto.FLOAT, [3,3,1,1]),
        helper.make_tensor_value_info('gamma_next',              TensorProto.FLOAT, [1,1,1,1]),
        helper.make_tensor_value_info('lens.coeffs_next',        TensorProto.FLOAT, [1,4]),
        helper.make_tensor_value_info('noise.strength_next',     TensorProto.FLOAT, [1,1,1,1]),
        helper.make_tensor_value_info('sharp.strength_next',     TensorProto.FLOAT, [1,1,1,1]),
        helper.make_tensor_value_info('color.coeffs_next',       TensorProto.FLOAT, [1,3])
    ]

    graph = helper.make_graph(nodes, 'ISP_Algo_Coeffs_From_Bayer_Real', [bayer_in], outs, inits)
    # opset 11 with IR 7 keeps broad compatibility
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 11)], ir_version=7)
    onnx.checker.check_model(model)
    return model

if __name__ == "__main__":
    model = build_algo_coeffs()
    onnx.save(model, "isp_algo_coeffs_from_bayer.onnx")
    print("Saved isp_algo_coeffs_from_bayer.onnx")
