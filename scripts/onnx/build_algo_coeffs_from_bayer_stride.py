# build_algo_coeffs_stride_crop.py
import onnx
from onnx import helper, TensorProto
import numpy as np

def make_scalar_const(name, value):
    arr = np.array([value], dtype=np.float32)
    return helper.make_tensor(name=name, data_type=TensorProto.FLOAT, dims=[1], vals=arr)

def make_int64_const(name, values):
    arr = np.array(values, dtype=np.int64)
    return helper.make_tensor(name=name, data_type=TensorProto.INT64, dims=[len(values)], vals=arr)

def build_algo_coeffs_stride_crop():
    # --- Inputs ---
    # Bayer input with symbolic NCHW (expect W=2048 at runtime; crop will reduce to 1920)
    bayer_in = helper.make_tensor_value_info('input.bayer', TensorProto.FLOAT, ['N','C','H','W'])

    # Dynamic crop parameters (int64 vectors of length 2): [start_h, start_w], [end_h, end_w], [2,3]
    crop_starts = helper.make_tensor_value_info('crop_starts', TensorProto.INT64, [2])
    crop_ends   = helper.make_tensor_value_info('crop_ends',   TensorProto.INT64, [2])
    crop_axes   = helper.make_tensor_value_info('crop_axes',   TensorProto.INT64, [2])

    nodes, inits = [], []

    # --- Remove stride/padding: Slice to FHD (1920x1080) dynamically ---
    nodes.append(helper.make_node(
        'Slice',
        ['input.bayer','crop_starts','crop_ends','crop_axes'],
        ['bayer_fhd'],
        name='remove_stride_crop'
    ))

    # --- RGB proxy: replicate Bayer to 3 channels (N,3,H,W) ---
    nodes.append(helper.make_node(
        'Concat',
        ['bayer_fhd','bayer_fhd','bayer_fhd'],
        ['RGB'],
        axis=1,
        name='bayer_to_rgb_proxy'
    ))

    # --- Luminance Y = M709 * RGB via 1x1 Conv (N,1,H,W) ---
    M709 = np.array([[0.2126, 0.7152, 0.0722]], dtype=np.float32).reshape(1,3,1,1)  # outC=1, inC=3, k=1x1
    W_y = helper.make_tensor('Y.W', TensorProto.FLOAT, M709.shape, M709.ravel())
    inits.append(W_y)
    nodes.append(helper.make_node(
        'Conv',
        ['RGB','Y.W'],
        ['Y'],
        name='rgb_to_y'
    ))

    # --- AE ---
    nodes.append(helper.make_node('ReduceMean', ['Y'], ['y_mean'], keepdims=0, name='ae_reduce_mean'))
    inits += [
        make_scalar_const('AE.target', 0.18),
        make_scalar_const('AE.Kp', 0.7),
        make_scalar_const('AE.zero', 0.0)
    ]
    nodes.append(helper.make_node('Sub', ['AE.target','y_mean'], ['ae_err'], name='ae_err'))
    nodes.append(helper.make_node('Mul', ['AE.Kp','ae_err'], ['ae_delta'], name='ae_delta'))
    nodes.append(helper.make_node('Add', ['ae_delta','AE.zero'], ['ae_next_scalar'], name='ae_next_scalar'))
    ae_shape = make_int64_const('ae.shape', [1,1,1,1])
    inits.append(ae_shape)
    nodes.append(helper.make_node('Reshape', ['ae_next_scalar','ae.shape'], ['ae.gain_next'], name='ae_gain_next'))

    # --- AWB ---
    # Mean over H,W
    nodes.append(helper.make_node('ReduceMean', ['RGB'], ['rgb_mean_hw'], axes=[2,3], keepdims=1, name='awb_mean_hw'))

    # Slice channel-wise means: R,G,B from axis=1
    starts_r = make_int64_const('starts_r', [0]); ends_r = make_int64_const('ends_r', [1])
    starts_g = make_int64_const('starts_g', [1]); ends_g = make_int64_const('ends_g', [2])
    starts_b = make_int64_const('starts_b', [2]); ends_b = make_int64_const('ends_b', [3])
    axes_c   = make_int64_const('axes_c',   [1])
    inits += [starts_r, ends_r, starts_g, ends_g, starts_b, ends_b, axes_c]
    nodes.append(helper.make_node('Slice', ['rgb_mean_hw','starts_r','ends_r','axes_c'], ['r_mean'], name='awb_r_mean'))
    nodes.append(helper.make_node('Slice', ['rgb_mean_hw','starts_g','ends_g','axes_c'], ['g_mean'], name='awb_g_mean'))
    nodes.append(helper.make_node('Slice', ['rgb_mean_hw','starts_b','ends_b','axes_c'], ['b_mean'], name='awb_b_mean'))

    nodes.append(helper.make_node('Div', ['g_mean','r_mean'], ['rgain'], name='awb_r_gain'))
    nodes.append(helper.make_node('Identity', ['g_mean'], ['ggain'], name='awb_g_gain'))
    nodes.append(helper.make_node('Div', ['g_mean','b_mean'], ['bgain'], name='awb_b_gain'))
    nodes.append(helper.make_node('Concat', ['rgain','ggain','bgain'], ['awb_vec'], axis=0, name='awb_vec'))
    awb_shape = make_int64_const('awb.shape', [1,3,1,1])
    inits.append(awb_shape)
    nodes.append(helper.make_node('Reshape', ['awb_vec','awb.shape'], ['awb.gains_next'], name='awb_gains_next'))

    # --- Adaptive Gamma ---
    inits += [
        make_scalar_const('gamma_base', 1/2.2),
        make_scalar_const('gamma_k', 0.35),
        helper.make_tensor('gamma_min', TensorProto.FLOAT, [1], np.array([0.25], dtype=np.float32)),
        helper.make_tensor('gamma_max', TensorProto.FLOAT, [1], np.array([1.2], dtype=np.float32))
    ]
    nodes.append(helper.make_node('Sub', ['AE.target','y_mean'], ['gamma_err'], name='gamma_err'))
    nodes.append(helper.make_node('Mul', ['gamma_k','gamma_err'], ['gamma_adj'], name='gamma_adj'))
    nodes.append(helper.make_node('Add', ['gamma_base','gamma_adj'], ['gamma_raw'], name='gamma_raw'))
    nodes.append(helper.make_node('Clip', ['gamma_raw','gamma_min','gamma_max'], ['gamma_clamped'], name='gamma_clamped'))
    nodes.append(helper.make_node('Reshape', ['gamma_clamped','ae.shape'], ['gamma_next'], name='gamma_next'))

    # --- Noise estimation ---
    nodes.append(helper.make_node('ReduceMean', ['Y'], ['y_mean_hw'], axes=[2,3], keepdims=1, name='noise_mean_hw'))
    nodes.append(helper.make_node('Sub', ['Y','y_mean_hw'], ['y_centered'], name='noise_center'))
    nodes.append(helper.make_node('Mul', ['y_centered','y_centered'], ['y_sq'], name='noise_square'))
    nodes.append(helper.make_node('ReduceMean', ['y_sq'], ['y_var'], keepdims=0, name='noise_var'))
    inits += [
        make_scalar_const('k_noise', 10.0),
        helper.make_tensor('noise_min', TensorProto.FLOAT, [1], np.array([0.0], dtype=np.float32)),
        helper.make_tensor('noise_max', TensorProto.FLOAT, [1], np.array([3.0], dtype=np.float32))
    ]
    nodes.append(helper.make_node('Mul', ['k_noise','y_var'], ['noise_raw'], name='noise_raw'))
    nodes.append(helper.make_node('Clip', ['noise_raw','noise_min','noise_max'], ['noise_clamped'], name='noise_clip'))
    nodes.append(helper.make_node('Reshape', ['noise_clamped','ae.shape'], ['noise.strength_next'], name='noise_next'))

    # --- Sharpness strength (inverse of noise with base and alpha) ---
    inits += [
        make_scalar_const('sharp_base', 1.0),
        make_scalar_const('sharp_alpha', 0.6),
        make_scalar_const('ONE', 1.0)
    ]
    nodes.append(helper.make_node('Mul', ['sharp_alpha','noise_clamped'], ['noise_scaled'], name='sharp_noise_scaled'))
    nodes.append(helper.make_node('Add', ['noise_scaled','ONE'], ['denom'], name='sharp_denom'))
    nodes.append(helper.make_node('Div', ['sharp_base','denom'], ['sharp_raw'], name='sharp_raw'))
    nodes.append(helper.make_node('Reshape', ['sharp_raw','ae.shape'], ['sharp.strength_next'], name='sharp_next'))

    # --- CCM (identity 3x3) ---
    ccm = np.eye(3, dtype=np.float32).reshape(3,3,1,1)
    ccm_t = helper.make_tensor('ccm_init', TensorProto.FLOAT, ccm.shape, ccm.ravel())
    inits.append(ccm_t)
    nodes.append(helper.make_node('Identity', ['ccm_init'], ['ccm_next'], name='ccm_next'))

    # --- Lens coefficients (zeros, 4-vector) ---
    lens_t = helper.make_tensor('lens_init', TensorProto.FLOAT, [4], np.zeros(4, dtype=np.float32))
    lens_shape = make_int64_const('lens.shape', [1,4])
    inits += [lens_t, lens_shape]
    nodes.append(helper.make_node('Reshape', ['lens_init','lens.shape'], ['lens.coeffs_next'], name='lens_next'))

    # --- Color coefficients (example 3-vector) ---
    color_t = helper.make_tensor('color_init', TensorProto.FLOAT, [3], np.array([1.0, 1.0, 0.0], dtype=np.float32))
    color_shape = make_int64_const('color.shape', [1,3])
    inits += [color_t, color_shape]
    nodes.append(helper.make_node('Reshape', ['color_init','color.shape'], ['color.coeffs_next'], name='color_next'))

    # --- Outputs ---
    outs = [
        helper.make_tensor_value_info('ae.gain_next',        TensorProto.FLOAT, [1,1,1,1]),
        helper.make_tensor_value_info('awb.gains_next',      TensorProto.FLOAT, [1,3,1,1]),
        helper.make_tensor_value_info('ccm_next',            TensorProto.FLOAT, [3,3,1,1]),
        helper.make_tensor_value_info('gamma_next',          TensorProto.FLOAT, [1,1,1,1]),
        helper.make_tensor_value_info('lens.coeffs_next',    TensorProto.FLOAT, [1,4]),
        helper.make_tensor_value_info('noise.strength_next', TensorProto.FLOAT, [1,1,1,1]),
        helper.make_tensor_value_info('sharp.strength_next', TensorProto.FLOAT, [1,1,1,1]),
        helper.make_tensor_value_info('color.coeffs_next',   TensorProto.FLOAT, [1,3])
    ]

    # --- Build graph and model ---
    graph = helper.make_graph(
        nodes,
        name='ISP_Algo_Coeffs_StrideCrop_Dynamic',
        inputs=[bayer_in, crop_starts, crop_ends, crop_axes],
        outputs=outs,
        initializer=inits
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 11)], ir_version=7)
    onnx.checker.check_model(model)
    return model

if __name__ == "__main__":
    model = build_algo_coeffs_stride_crop()
    onnx.save(model, "isp_algo_coeffs_stride_crop.onnx")
    print("Saved isp_algo_coeffs_stride_crop.onnx")
