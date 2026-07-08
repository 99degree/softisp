/** C wrapper for MNNConverter library (libMNNConvertDeps.so).
    Exposes a simple C API for converting ONNX/TF/TFLite models to MNN format.

    Compile with:
      clang++ -std=c++17 -c mnn_convert_api.cpp
          -I${MNN_INCLUDE_DIR} -I${MNN_TOOLS_DIR}/converter/include
          -o mnn_convert_api.o -fPIC -O2

    Then link with libMNNConvertDeps.so when building the final binary. */

#include <string>
#include <cstdio>
#include "config.hpp"
#include "cli.hpp"

extern "C" {

/// Result of a conversion operation.
typedef struct {
    int success;    // 0 = success, -1 = error
    char error_msg[1024];
} MnnConvertResult;

/// Convert an ONNX model to MNN format.
/// @param onnx_path   Path to the input ONNX model file.
/// @param mnn_path    Path where the output .mnn file will be written.
/// @param biz_code    Business code (e.g. "MNN").
/// @param optimize_level  Graph optimization level (0=none, 1=safe, 2=aggressive).
/// @param weight_quant_bits  Weight quantization bits (0=no quant, 2-8 for INT8 etc.).
/// @param fp16        Whether to convert weights to FP16.
/// @param preserve_input_type  Preserve int16/uint16/float16 input types (no widening to int32).
/// @param result      Output result structure.
MNN_PUBLIC void mnn_convert_onnx_to_mnn(
    const char* onnx_path,
    const char* mnn_path,
    const char* biz_code,
    int optimize_level,
    int weight_quant_bits,
    int fp16,
    int preserve_input_type,
    MnnConvertResult* result)
{
    result->success = 0;
    result->error_msg[0] = '\0';

    try {
        modelConfig config;
        config.model = modelConfig::ONNX;
        config.modelFile = onnx_path;
        config.MNNModel = mnn_path;
        config.bizCode = biz_code ? biz_code : "MNN";
        config.optimizeLevel = optimize_level;
        config.weightQuantBits = weight_quant_bits;
        config.saveHalfFloat = fp16 != 0;
        config.preserveInputType = preserve_input_type != 0;

        if (!MNN::Cli::convertModel(config)) {
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg),
                     "MNN::Cli::convertModel returned false");
        }
    } catch (const std::exception& e) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "%s", e.what());
    } catch (...) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "Unknown exception");
    }
}

} // extern "C"
