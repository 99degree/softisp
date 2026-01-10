# build_test_algos_coeffs.py
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

def build_coeff_controller():
    # Inputs
    meanY = helper.make_tensor_value_info('AE.meanY', TensorProto.FLOAT, [1,1,1,1])
    Rm    = helper.make_tensor_value_info('AWB.Rmean', TensorProto.FLOAT, [1,1,1,1])
    Gm    = helper.make_tensor_value_info('AWB.Gmean', TensorProto.FLOAT, [1,1,1,1])
    Bm    = helper.make_tensor_value_info('AWB.Bmean', TensorProto.FLOAT, [1,1,1,1])
    AFs   = helper.make_tensor_value_info('AF.score',  TensorProto.FLOAT, [1,1,1,1])

    ae_in = helper.make_tensor_value_info('ae.gain',   TensorProto.FLOAT, [1,1,1,1])
    awb_in= helper.make_tensor_value_info('awb.gains', TensorProto.FLOAT, [1,3,1,1])
    ccm_in= helper.make_tensor_value_info('ccm',       TensorProto.FLOAT, [3,3,1,1])
    gamma_in = helper.make_tensor_value_info('gamma',  TensorProto.FLOAT, [1,1,1,1])
    lens_in  = helper.make_tensor_value_info('lens.coeffs', TensorProto.FLOAT, [1,4])
    noise_in = helper.make_tensor_value_info('noise.strength', TensorProto.FLOAT, [1,1,1,1])
    sharp_in = helper.make_tensor_value_info('sharp.strength', TensorProto.FLOAT, [1,1,1,1])
    color_in = helper.make_tensor_value_info('color.coeffs', TensorProto.FLOAT, [1,3])

    nodes, inits = [], []

    # AE update
    inits += [make_scalar_const('AE.target', 0.5), make_scalar_const('AE.Kp', 0.5)]
    nodes.append(helper.make_node('ReduceMean', ['AE.meanY'], ['y_mean'], keepdims=0))
    nodes.append(helper.make_node('ReduceMean', ['ae.gain'], ['ae_curr'], keepdims=0))
    nodes.append(helper.make_node('Sub', ['AE.target','y_mean'], ['ae_err']))
    nodes.append(helper.make_node('Mul', ['AE.Kp','ae_err'], ['ae_delta']))
    nodes.append(helper.make_node('Add', ['ae_curr','ae_delta'], ['ae_next_scalar']))
    ae_shape = helper.make_tensor('ae.shape', TensorProto.INT64, [4], np.array([1,1,1,1], dtype=np.int64))
    inits.append(ae_shape)
    nodes.append(helper.make_node('Reshape',['ae_next_scalar','ae.shape'],['ae.gain_next']))

    # AWB update
    nodes.append(helper.make_node('ReduceMean',['AWB.Rmean'],['r_mean'],keepdims=0))
    nodes.append(helper.make_node('ReduceMean',['AWB.Gmean'],['g_mean'],keepdims=0))
    nodes.append(helper.make_node('ReduceMean',['AWB.Bmean'],['b_mean'],keepdims=0))
    nodes.append(helper.make_node('Add',['r_mean','g_mean'],['rg_sum']))
    nodes.append(helper.make_node('Add',['rg_sum','b_mean'],['rgb_sum']))
    inits.append(make_scalar_const('ONE_THIRD',1/3))
    nodes.append(helper.make_node('Mul',['rgb_sum','ONE_THIRD'],['avg_rgb']))
    nodes.append(helper.make_node('Div',['avg_rgb','r_mean'],['rgain']))
    nodes.append(helper.make_node('Div',['avg_rgb','g_mean'],['ggain']))
    nodes.append(helper.make_node('Div',['avg_rgb','b_mean'],['bgain']))
    nodes.append(helper.make_node('Concat',['rgain','ggain','bgain'],['awb_vec'],axis=0))
    awb_shape = helper.make_tensor('awb.shape', TensorProto.INT64, [4], np.array([1,3,1,1], dtype=np.int64))
    inits.append(awb_shape)
    nodes.append(helper.make_node('Reshape',['awb_vec','awb.shape'],['awb.gains_next']))

    # Pass-throughs
    nodes.append(helper.make_node('Identity',['ccm'],['ccm_next']))
    nodes.append(helper.make_node('Identity',['gamma'],['gamma_next']))
    nodes.append(helper.make_node('Identity',['lens.coeffs'],['lens.coeffs_next']))
    nodes.append(helper.make_node('Identity',['noise.strength'],['noise.strength_next']))
    nodes.append(helper.make_node('Identity',['sharp.strength'],['sharp.strength_next']))
    nodes.append(helper.make_node('Identity',['color.coeffs'],['color.coeffs_next']))

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

    graph = helper.make_graph(
        nodes,
        'ISP_Coeff_Controller',
        [meanY,Rm,Gm,Bm,AFs,ae_in,awb_in,ccm_in,gamma_in,lens_in,noise_in,sharp_in,color_in],
        outs,
        inits
    )
    '''model = helper.make_model(graph,opset_imports=[helper.make_opsetid("",11)])'''

    model = helper.make_model(
        graph,
        opset_imports=[helper.make_opsetid("", 11)],
        ir_version=7  # corresponds to ONNX IR v11 support in onnxruntime
    )
    onnx.checker.check_model(model)
    return model



if __name__ == "__main__":
    model = build_coeff_controller()
    onnx.save(model,"isp_coeff_controller.onnx")
    print("Saved isp_coeff_controller.onnx")
