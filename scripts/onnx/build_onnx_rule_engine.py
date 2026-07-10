import onnx
from onnx import helper, TensorProto, numpy_helper

def const_scalar(name, value):
    return helper.make_node(
        "Constant", [], [name],
        value=helper.make_tensor(name + "_t", TensorProto.FLOAT, [1], [float(value)])
    )

def neg_scalar(input_name, output_name):
    return helper.make_node("Neg", [input_name], [output_name])

def clamp_scalar(x, xmin, xmax, out):
    return helper.make_node("Clip", [x, xmin, xmax], [out])

def broadcast_mul(a, b, out):
    return helper.make_node("Mul", [a, b], [out])

def broadcast_add(a, b, out):
    return helper.make_node("Add", [a, b], [out])

def broadcast_sub(a, b, out):
    return helper.make_node("Sub", [a, b], [out])

def make_input(name, shape):
    return helper.make_tensor_value_info(name, TensorProto.FLOAT, shape)

def ema_vector(raw, prev, alpha, out):
    # out = alpha*raw + (1-alpha)*prev
    one = const_scalar(out + "_one", 1.0)
    alpha_comp = helper.make_node("Sub", [one.output[0], alpha], [out + "_alpha_comp"])
    mul_raw = helper.make_node("Mul", [raw, alpha], [out + "_mul_raw"])
    mul_prev = helper.make_node("Mul", [prev, alpha_comp.output[0]], [out + "_mul_prev"])
    add = helper.make_node("Add", [mul_raw.output[0], mul_prev.output[0]], [out])
    return [one, alpha_comp, mul_raw, mul_prev, add]

def rate_limit_vector(raw, prev, step, out):
    # out = prev + clip(raw - prev, -step, +step)
    delta = helper.make_node("Sub", [raw, prev], [out + "_delta"])
    neg_step = neg_scalar(step, out + "_neg_step")
    clip = helper.make_node("Clip", [delta.output[0], neg_step.output[0], step], [out + "_clip"])
    add = helper.make_node("Add", [prev, clip.output[0]], [out])
    return [delta, neg_step, clip, add]

def where_scalar(cond, a, b, out):
    # cond is scalar 0/1; we broadcast to match shapes
    # ONNX Where expects same shape; we keep a/b as [1]
    return helper.make_node("Where", [cond, a, b], [out])

def reduce_mean_vec(x, out):
    # ReduceMean over vector (keepdims=0)
    return helper.make_node("ReduceMean", [x], [out], keepdims=0)

def div_vector(a, b, out):
    return helper.make_node("Div", [a, b], [out])

def min_scalar(a, b, out):
    return helper.make_node("Min", [a, b], [out])

def max_scalar(a, b, out):
    return helper.make_node("Max", [a, b], [out])

def build_ruleengine(gammaN=256, model_name="ruleengine_full.onnx"):
    nodes = []

    # Inputs: raw coeffs
    raw_wb = make_input("raw_wb", [3])
    raw_ccm = make_input("raw_ccm", [9])
    raw_gamma = make_input("raw_gamma", [gammaN])
    raw_sharpness = make_input("raw_sharpness", [1])
    raw_nr = make_input("raw_nr", [1])

    # Inputs: prev coeffs
    prev_wb = make_input("prev_wb", [3])
    prev_ccm = make_input("prev_ccm", [9])
    prev_gamma = make_input("prev_gamma", [gammaN])
    prev_sharpness = make_input("prev_sharpness", [1])
    prev_nr = make_input("prev_nr", [1])

    # Inputs: sensor meta
    analog_gain = make_input("analog_gain", [1])
    exposure_time = make_input("exposure_time", [1])
    sensor_temp = make_input("sensor_temp", [1])
    scene_change = make_input("scene_change", [1])  # 0.0 or 1.0

    # Inputs: rule params (runtime-tunable)
    alpha_wb = make_input("alpha_wb", [1])
    alpha_ccm = make_input("alpha_ccm", [1])
    alpha_gamma = make_input("alpha_gamma", [1])
    alpha_sharp = make_input("alpha_sharp", [1])
    alpha_nr = make_input("alpha_nr", [1])
    alpha_fast = make_input("alpha_fast", [1])       # used on scene change

    wb_step = make_input("wb_step", [1])
    sharp_step = make_input("sharp_step", [1])

    wb_min = make_input("wb_min", [1])
    wb_max = make_input("wb_max", [1])

    gamma_min = make_input("gamma_min", [1])
    gamma_max = make_input("gamma_max", [1])

    ccm_min = make_input("ccm_min", [1])
    ccm_max = make_input("ccm_max", [1])

    nr_min = make_input("nr_min", [1])
    nr_max = make_input("nr_max", [1])

    # -------- WB: normalize + EMA + rate limit + clamp + scene-change alpha --------
    # Normalize raw_wb by mean to keep neutral average
    wb_mean = "wb_mean"
    nodes.append(reduce_mean_vec("raw_wb", wb_mean))
    wb_mean_expand = helper.make_node("Unsqueeze", [wb_mean], ["wb_mean_u"], axes=[0])
    nodes.append(wb_mean_expand)
    wb_norm = "wb_norm"
    nodes.append(div_vector("raw_wb", "wb_mean_u", wb_norm))

    # Adaptive alpha: alpha_eff = scene_change ? alpha_fast : alpha_wb
    alpha_wb_eff_min = "alpha_wb_eff_min"
    # Ensure alpha_fast >= alpha_wb via max; it’s optional but safe
    nodes.append(max_scalar("alpha_wb", "alpha_fast", alpha_wb_eff_min))
    alpha_wb_eff = "alpha_wb_eff"
    nodes.append(where_scalar("scene_change", alpha_wb_eff_min, "alpha_wb", alpha_wb_eff))

    # EMA
    nodes += ema_vector(wb_norm, "prev_wb", alpha_wb_eff, "wb_ema")

    # Rate limit relative to prev
    nodes += rate_limit_vector(wb_norm, "prev_wb", "wb_step", "wb_rl")

    # Blend EMA and RL (simple average)
    half = const_scalar("half", 0.5)
    nodes.append(half)
    wb_sum = helper.make_node("Add", ["wb_ema", "wb_rl"], ["wb_sum"])
    nodes.append(wb_sum)
    wb_blend = helper.make_node("Mul", ["wb_sum", half.output[0]], ["wb_blend"])
    nodes.append(wb_blend)

    # Clamp
    wb_stab = helper.make_node("Clip", ["wb_blend", "wb_min", "wb_max"], ["wb_stab"])
    nodes.append(wb_stab)

    # -------- CCM: EMA + clamp + scene-change alpha --------
    alpha_ccm_eff_min = "alpha_ccm_eff_min"
    nodes.append(max_scalar("alpha_ccm", "alpha_fast", alpha_ccm_eff_min))
    alpha_ccm_eff = "alpha_ccm_eff"
    nodes.append(where_scalar("scene_change", alpha_ccm_eff_min, "alpha_ccm", alpha_ccm_eff))

    nodes += ema_vector("raw_ccm", "prev_ccm", alpha_ccm_eff, "ccm_ema")
    ccm_stab = helper.make_node("Clip", ["ccm_ema", "ccm_min", "ccm_max"], ["ccm_stab"])
    nodes.append(ccm_stab)

    # -------- Gamma: EMA + clamp + scene-change alpha --------
    alpha_gamma_eff_min = "alpha_gamma_eff_min"
    nodes.append(max_scalar("alpha_gamma", "alpha_fast", alpha_gamma_eff_min))
    alpha_gamma_eff = "alpha_gamma_eff"
    nodes.append(where_scalar("scene_change", alpha_gamma_eff_min, "alpha_gamma", alpha_gamma_eff))

    nodes += ema_vector("raw_gamma", "prev_gamma", alpha_gamma_eff, "gamma_ema")
    gamma_stab = helper.make_node("Clip", ["gamma_ema", "gamma_min", "gamma_max"], ["gamma_stab"])
    nodes.append(gamma_stab)

    # -------- Sharpness: sensor-aware modulation + EMA + rate limit + clamp --------
    # noise_factor = clamp(analog_gain, 1.0, 16.0)
    gain_min = const_scalar("gain_min", 1.0)
    gain_max = const_scalar("gain_max", 16.0)
    nodes.append(gain_min); nodes.append(gain_max)
    gain_clamped = helper.make_node("Clip", ["analog_gain", gain_min.output[0], gain_max.output[0]], ["noise_factor"])
    nodes.append(gain_clamped)

    # target_sharp = raw_sharpness / noise_factor
    target_sharp = helper.make_node("Div", ["raw_sharpness", "noise_factor"], ["target_sharp"])
    nodes.append(target_sharp)

    # Adaptive alpha
    alpha_sharp_eff_min = "alpha_sharp_eff_min"
    nodes.append(max_scalar("alpha_sharp", "alpha_fast", alpha_sharp_eff_min))
    alpha_sharp_eff = "alpha_sharp_eff"
    nodes.append(where_scalar("scene_change", alpha_sharp_eff_min, "alpha_sharp", alpha_sharp_eff))

    # EMA (scalar shape [1])
    nodes += ema_vector("target_sharp", "prev_sharpness", alpha_sharp_eff, "sharp_ema")

    # Rate limit
    nodes += rate_limit_vector("target_sharp", "prev_sharpness", "sharp_step", "sharp_rl")

    # Blend EMA/RL and clamp
    sharp_sum = helper.make_node("Add", ["sharp_ema", "sharp_rl"], ["sharp_sum"])
    nodes.append(sharp_sum)
    sharp_blend = helper.make_node("Mul", ["sharp_sum", half.output[0]], ["sharp_blend"])
    nodes.append(sharp_blend)
    sharpness_stab = helper.make_node("Clip", ["sharp_blend", "gamma_min", "gamma_max"], ["sharpness_stab"])
    # NOTE: reuse gamma_min/max as generic clamps; adjust if you want distinct sharp_min/max
    nodes.append(sharpness_stab)

    # -------- NR: sensor-aware modulation + EMA + clamp --------
    # target_nr = raw_nr * noise_factor
    target_nr = helper.make_node("Mul", ["raw_nr", "noise_factor"], ["target_nr"])
    nodes.append(target_nr)

    alpha_nr_eff_min = "alpha_nr_eff_min"
    nodes.append(max_scalar("alpha_nr", "alpha_fast", alpha_nr_eff_min))
    alpha_nr_eff = "alpha_nr_eff"
    nodes.append(where_scalar("scene_change", alpha_nr_eff_min, "alpha_nr", alpha_nr_eff))

    nodes += ema_vector("target_nr", "prev_nr", alpha_nr_eff, "nr_ema")
    nr_stab = helper.make_node("Clip", ["nr_ema", "nr_min", "nr_max"], ["nr_stab"])
    nodes.append(nr_stab)

    # Outputs
    out_wb = helper.make_tensor_value_info("wb_stab", TensorProto.FLOAT, [3])
    out_ccm = helper.make_tensor_value_info("ccm_stab", TensorProto.FLOAT, [9])
    out_gamma = helper.make_tensor_value_info("gamma_stab", TensorProto.FLOAT, [gammaN])
    out_sharp = helper.make_tensor_value_info("sharpness_stab", TensorProto.FLOAT, [1])
    out_nr = helper.make_tensor_value_info("nr_stab", TensorProto.FLOAT, [1])

    # Assemble graph
    graph = helper.make_graph(
        nodes,
        "RuleEngineFull",
        [
            raw_wb, raw_ccm, raw_gamma, raw_sharpness, raw_nr,
            prev_wb, prev_ccm, prev_gamma, prev_sharpness, prev_nr,
            analog_gain, exposure_time, sensor_temp, scene_change,
            alpha_wb, alpha_ccm, alpha_gamma, alpha_sharp, alpha_nr, alpha_fast,
            wb_step, sharp_step,
            wb_min, wb_max, gamma_min, gamma_max, ccm_min, ccm_max, nr_min, nr_max,
        ],
        [out_wb, out_ccm, out_gamma, out_sharp, out_nr],
    )

    model = helper.make_model(graph, producer_name="ruleengine_builder")
    onnx.checker.check_model(model)
    onnx.save(model, model_name)
    return model

if __name__ == "__main__":
    build_ruleengine(gammaN=256, model_name="ruleengine_full.onnx")
    print("Saved ruleengine_full.onnx")
