import onnx
from onnx import helper, TensorProto

# Inputs
raw_wb = helper.make_tensor_value_info("raw_wb", TensorProto.FLOAT, [3])
prev_wb = helper.make_tensor_value_info("prev_wb", TensorProto.FLOAT, [3])
alpha_wb = helper.make_tensor_value_info("alpha_wb", TensorProto.FLOAT, [1])
wb_step = helper.make_tensor_value_info("wb_step", TensorProto.FLOAT, [1])
wb_min = helper.make_tensor_value_info("wb_min", TensorProto.FLOAT, [1])
wb_max = helper.make_tensor_value_info("wb_max", TensorProto.FLOAT, [1])

# EMA smoothing: wb_s = alpha*raw + (1-alpha)*prev
one = helper.make_node("Constant", [], ["one"],
    value=helper.make_tensor("one_t", TensorProto.FLOAT, [1], [1.0]))
alpha_comp = helper.make_node("Sub", ["one", "alpha_wb"], ["alpha_comp"])
mul_raw = helper.make_node("Mul", ["raw_wb", "alpha_wb"], ["mul_raw"])
mul_prev = helper.make_node("Mul", ["prev_wb", "alpha_comp"], ["mul_prev"])
ema = helper.make_node("Add", ["mul_raw", "mul_prev"], ["ema_out"])

# Rate limit: delta = raw - prev; clip to +/- wb_step
delta = helper.make_node("Sub", ["raw_wb", "prev_wb"], ["delta"])
clip_delta = helper.make_node("Clip", ["delta", "wb_step", "wb_step"], ["clip_delta"])
rate_limited = helper.make_node("Add", ["prev_wb", "clip_delta"], ["rl_out"])

# Combine EMA + rate limit (average them here, or choose one via Where)
final = helper.make_node("Add", ["ema_out", "rl_out"], ["wb_sum"])
half = helper.make_node("Constant", [], ["half"],
    value=helper.make_tensor("half_t", TensorProto.FLOAT, [1], [0.5]))
wb_out = helper.make_node("Mul", ["wb_sum", "half"], ["wb_out"])

# Clamp
wb_clamped = helper.make_node("Clip", ["wb_out", "wb_min", "wb_max"], ["wb_stab"])

# Outputs
stab_wb = helper.make_tensor_value_info("wb_stab", TensorProto.FLOAT, [3])

graph = helper.make_graph(
    [one, alpha_comp, mul_raw, mul_prev, ema,
     delta, clip_delta, rate_limited,
     final, half, wb_out, wb_clamped],
    "RuleEngineWB",
    [raw_wb, prev_wb, alpha_wb, wb_step, wb_min, wb_max],
    [stab_wb]
)

model = helper.make_model(graph, producer_name="ruleengine")
onnx.checker.check_model(model)
onnx.save(model, "ruleengine_wb.onnx")
