# build_onnx.py
import onnx
from onnx import helper, TensorProto
import numpy as np

def make_scalar_const(name, value):
    return helper.make_tensor(
        name=name,
        data_type=TensorProto.FLOAT,
        dims=[1],
        vals=np.array([value], dtype=np.float32)
    )

def build_algo_model():
    # Input: Bayer image with symbolic dimensions
    bayer_in = helper.make_tensor_value_info(
        'input.bayer', TensorProto.FLOAT, ['N','C','H','W']
    )

    nodes, inits = [], []

    # --- Simplified RGB extraction: replicate Bayer to 3 channels ---
    nodes.append(helper.make_node(
        'Concat', ['input.bayer','input.bayer','input.bayer'],
        ['RGB'], axis=1
    ))

    # --- Compute luminance Y ---
    M709 = np.array([[0.2126,0.7152,0.0722]],dtype=np.float32).reshape(1,3,1,1)
    W_y = helper.make_tensor('Y.W', TensorProto.FLOAT, M709.shape, M709.ravel())
    inits.append(W_y)
    nodes.append(helper.make_node('Conv',['RGB','Y.W'],['Y'],name='rgb2y'))

    # --- AE update ---
    inits += [make_scalar_const('AE.target',0.5), make_scalar_const('AE.Kp',0.5)]
    nodes.append(helper.make_node('ReduceMean',['Y'],['y_mean'],keepdims=0))
    nodes.append(helper.make_node('Sub',['AE.target','y_mean'],['ae_err']))
    nodes.append(helper.make_node('Mul',['AE.Kp','ae_err'],['ae_delta']))
    zero_const = make_scalar_const('AE.zero',0.0)
    inits.append(zero_const)
    nodes.append(helper.make_node('Add',['ae_delta','AE.zero'],['ae_next_scalar']))
    ae_shape = helper.make_tensor('ae.shape', TensorProto.INT64, [4], np.array([1,1,1,1],dtype=np.int64))
    inits.append(ae_shape)
    nodes.append(helper.make_node('Reshape',['ae_next_scalar','ae.shape'],['ae.gain_next']))

    # --- AWB update (dummy normalized gains) ---
    rgain = make_scalar_const('rgain',1.0)
    ggain = make_scalar_const('ggain',1.0)
    bgain = make_scalar_const('bgain',1.0)
    inits += [rgain,ggain,bgain]
    nodes.append(helper.make_node('Concat',['rgain','ggain','bgain'],['awb_vec'],axis=0))
    awb_shape = helper.make_tensor('awb.shape', TensorProto.INT64, [4], np.array([1,3,1,1],dtype=np.int64))
    inits.append(awb_shape)
    nodes.append(helper.make_node('Reshape',['awb_vec','awb.shape'],['awb.gains_next']))

    # --- CCM constant ---
    ccm = np.eye(3,dtype=np.float32).reshape(3,3,1,1)
    ccm_t = helper.make_tensor('ccm_init',TensorProto.FLOAT,ccm.shape,ccm.ravel())
    inits.append(ccm_t)
    nodes.append(helper.make_node('Identity',['ccm_init'],['ccm_next']))

    # --- Gamma constant ---
    gamma_t = make_scalar_const('gamma_init',1/2.2)
    inits.append(gamma_t)
    nodes.append(helper.make_node('Reshape',['gamma_init','ae.shape'],['gamma_next']))

    # --- Lens coeffs constant ---
    lens_t = helper.make_tensor('lens_init',TensorProto.FLOAT,[4],np.zeros(4,dtype=np.float32))
    inits.append(lens_t)
    lens_shape = helper.make_tensor('lens.shape',TensorProto.INT64,[2],np.array([1,4],dtype=np.int64))
    inits.append(lens_shape)
    nodes.append(helper.make_node('Reshape',['lens_init','lens.shape'],['lens.coeffs_next']))

    # --- Noise, sharp, color constants ---
    noise_t = make_scalar_const('noise_init',1.0)
    sharp_t = make_scalar_const('sharp_init',1.0)
    color_t = helper.make_tensor('color_init',TensorProto.FLOAT,[3],np.array([1.0,1.0,0.0],dtype=np.float32))
    inits += [noise_t,sharp_t,color_t]

    nodes.append(helper.make_node('Reshape',['noise_init','ae.shape'],['noise.strength_next']))
    nodes.append(helper.make_node('Reshape',['sharp_init','ae.shape'],['sharp.strength_next']))

    color_shape = helper.make_tensor('color.shape',TensorProto.INT64,[2],np.array([1,3],dtype=np.int64))
    inits.append(color_shape)
    nodes.append(helper.make_node('Reshape',['color_init','color.shape'],['color.coeffs_next']))

    # Outputs
    outs = [
        helper.make_tensor_value_info('ae.gain_next',TensorProto.FLOAT,[1,1,1,1]),
        helper.make_tensor_value_info('awb.gains_next',TensorProto.FLOAT,[1,3,1,1]),
        helper.make_tensor_value_info('ccm_next',TensorProto.FLOAT,[3,3,1,1]),
        helper.make_tensor_value_info('gamma_next',TensorProto.FLOAT,[1,1,1,1]),
        helper.make_tensor_value_info('lens.coeffs_next',TensorProto.FLOAT,[1,4]),
        helper.make_tensor_value_info('noise.strength_next',TensorProto.FLOAT,[1,1,1,1]),
        helper.make_tensor_value_info('sharp.strength_next',TensorProto.FLOAT,[1,1,1,1]),
        helper.make_tensor_value_info('color.coeffs_next',TensorProto.FLOAT,[1,3])
    ]

    graph = helper.make_graph(nodes,'ISP_Algo_Flexible',[bayer_in],outs,inits)
    model = helper.make_model(graph,opset_imports=[helper.make_opsetid("",11)],ir_version=7)
    onnx.checker.check_model(model)
    return model

if __name__ == "__main__":
    model = build_algo_model()
    onnx.save(model,"isp_algo_flexible.onnx")
    print("Saved isp_algo_flexible.onnx")
