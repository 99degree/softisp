/** C wrapper for MNNConverter library (libMNNConvertDeps.so).
    Exposes a simple C API for converting ONNX/TF/TFLite models to MNN format.

    Supports both file-path and memory-buffer conversion.

    Compile with:
      clang++ -std=c++17 -c mnn_convert_api.cpp
          -I${MNN_INCLUDE_DIR} -I${MNN_TOOLS_DIR}/converter/include
          -o mnn_convert_api.o -fPIC -O2

    Then link with libMNNConvertDeps.so when building the final binary. */

#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "config.hpp"
#include "cli.hpp"

extern "C" {

/// Result of a conversion operation.
typedef struct {
    int success;    // 0 = success, -1 = error
    char error_msg[1024];
} MnnConvertResult;

/// Buffer-based conversion result.
/// Caller must free `data` via MnnConvert_FreeBuffer when done.
typedef struct {
    int success;    // 0 = success, -1 = error
    char error_msg[1024];
    void* data;     // MNN model bytes (allocated with malloc)
    size_t size;    // size of MNN model data
} MnnConvertBufferResult;

/// Free the data pointer from a buffer result.
MNN_PUBLIC void MnnConvert_FreeBuffer(MnnConvertBufferResult* result) {
    if (result && result->data) {
        free(result->data);
        result->data = nullptr;
        result->size = 0;
    }
}

/// Convert an ONNX model to MNN format (file paths).
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

/// Convert ONNX model bytes directly to MNN model bytes.
/// Uses temp files for the converter (which requires file paths)
/// but immediately unlinks them — no persistent file on disk.
///
/// @param onnx_data   Pointer to ONNX protobuf bytes.
/// @param onnx_len    Length of ONNX data in bytes.
/// @param result      Output result structure with MNN data pointer.
///                    Caller must free result->data via MnnConvert_FreeBuffer.
MNN_PUBLIC void mnn_convert_onnx_buffer(
    const void* onnx_data,
    size_t onnx_len,
    MnnConvertBufferResult* result)
{
    result->success = 0;
    result->error_msg[0] = '\0';
    result->data = nullptr;
    result->size = 0;

    if (!onnx_data || onnx_len == 0) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "NULL or empty ONNX data");
        return;
    }

    // Write ONNX data to a temp file (in CWD, unlinked after read)
    char in_template[] = "mnn_onnx_XXXXXX";
    int in_fd = mkstemp(in_template);
    if (in_fd < 0) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "failed to create input tempfile");
        return;
    }
    // Write all ONNX data
    size_t written = 0;
    while (written < onnx_len) {
        ssize_t n = write(in_fd, (const char*)onnx_data + written, onnx_len - written);
        if (n < 0) { close(in_fd); unlink(in_template);
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg), "write input failed");
            return;
        }
        written += n;
    }
    close(in_fd);

    // Create output temp file
    char out_template[] = "mnn_out_XXXXXX";
    int out_fd = mkstemp(out_template);
    if (out_fd < 0) { unlink(in_template);
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "failed to create output tempfile");
        return;
    }
    close(out_fd);

    try {
        modelConfig config;
        config.model = modelConfig::ONNX;
        config.modelFile = in_template;
        config.MNNModel = out_template;
        config.bizCode = "MNN";
        config.optimizeLevel = 1;
        config.weightQuantBits = 0;
        config.saveHalfFloat = false;
        config.preserveInputType = true;

        if (!MNN::Cli::convertModel(config)) {
            unlink(in_template);
            unlink(out_template);
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg),
                     "MNN::Cli::convertModel returned false");
            return;
        }

        // Read output file into allocated buffer
        FILE* f = fopen(out_template, "rb");
        if (!f) {
            unlink(in_template);
            unlink(out_template);
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg), "failed to open output");
            return;
        }
        fseek(f, 0, SEEK_END);
        long fsize = ftell(f);
        rewind(f);

        void* mnn_data = malloc(fsize > 0 ? fsize : 1);
        if (!mnn_data) {
            fclose(f);
            unlink(in_template);
            unlink(out_template);
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg),
                     "Failed to allocate %ld bytes", fsize);
            return;
        }
        size_t total = fread(mnn_data, 1, fsize, f);
        fclose(f);

        // Delete both temp files
        unlink(in_template);
        unlink(out_template);

        result->data = mnn_data;
        result->size = total;
    } catch (const std::exception& e) {
        unlink(in_template);
        unlink(out_template);
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "%s", e.what());
        if (result->data) { free(result->data); result->data = nullptr; }
    } catch (...) {
        unlink(in_template);
        unlink(out_template);
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "Unknown exception");
        if (result->data) { free(result->data); result->data = nullptr; }
    }
}

} // extern "C"
