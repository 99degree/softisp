/** C wrapper for MNNConverter library (libMNNConvertDeps.so).
    Exposes a simple C API for converting ONNX/TF/TFLite models to MNN format.

    Supports both file-path and memory-buffer conversion with safe temp file
    management. Temp files are tracked globally and cleaned up on normal exit
    (atexit) and signal termination (SIGINT, SIGTERM).

    Compile with:
      clang++ -std=c++17 -c mnn_convert_api.cpp
          -I${MNN_INCLUDE_DIR} -I${MNN_TOOLS_DIR}/converter/include
          -o mnn_convert_api.o -fPIC -O2

    Then link with libMNNConvertDeps.so when building the final binary. */

#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <mutex>
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

// ── Safe Temp File Management ─────────────────────────────────────

/// Global registry of active temp file paths for cleanup on crash/exit.
/// Lock-free after initialization — only modified during conversion.
namespace {

std::mutex g_tempfile_mutex;
std::vector<std::string>* g_tempfiles = nullptr;

/// atexit handler: removes all tracked temp files.
void cleanup_tempfiles() {
    std::vector<std::string> paths;
    {
        std::lock_guard<std::mutex> lock(g_tempfile_mutex);
        if (!g_tempfiles) return;
        paths.swap(*g_tempfiles);
    }
    for (const auto& path : paths) {
        unlink(path.c_str());
    }
}

/// Signal handler: removes tracked temp files then re-raises.
void signal_handler(int sig) {
    cleanup_tempfiles();
    // Reset to default and re-raise for core dump
    signal(sig, SIG_DFL);
    raise(sig);
}

/// Register cleanup once.
std::once_flag g_cleanup_once;
void ensure_cleanup_registered() {
    std::call_once(g_cleanup_once, []() {
        g_tempfiles = new std::vector<std::string>();
        atexit(cleanup_tempfiles);
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        signal(SIGABRT, signal_handler);
    });
}

/// RAII guard: registers a temp file path on construction,
/// unlinks it on destruction. Safe for early returns and exceptions.
struct TempFileGuard {
    std::string path;
    bool active;

    TempFileGuard(const char* tmpl) : active(false) {
        char buf[512];
        snprintf(buf, sizeof(buf), "%s", tmpl);
        int fd = mkstemp(buf);
        if (fd < 0) return;
        close(fd);
        path = buf;
        active = true;
        // Register for global cleanup
        std::lock_guard<std::mutex> lock(g_tempfile_mutex);
        if (g_tempfiles) {
            g_tempfiles->push_back(path);
        }
    }

    ~TempFileGuard() {
        if (active) {
            unlink(path.c_str());
            // Remove from global registry
            std::lock_guard<std::mutex> lock(g_tempfile_mutex);
            if (g_tempfiles) {
                for (auto it = g_tempfiles->begin(); it != g_tempfiles->end(); ++it) {
                    if (*it == path) {
                        g_tempfiles->erase(it);
                        break;
                    }
                }
            }
        }
    }

    // Move-only
    TempFileGuard(TempFileGuard&& other) noexcept
        : path(std::move(other.path)), active(other.active) {
        other.active = false;
    }
    TempFileGuard& operator=(TempFileGuard&& other) noexcept {
        if (this != &other) {
            if (active) unlink(path.c_str());
            path = std::move(other.path);
            active = other.active;
            other.active = false;
        }
        return *this;
    }

    // No copy
    TempFileGuard(const TempFileGuard&) = delete;
    TempFileGuard& operator=(const TempFileGuard&) = delete;
};

} // anonymous namespace

// ── File-path conversion ─────────────────────────────────────────

/// Convert an ONNX model to MNN format (file paths).
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
    ensure_cleanup_registered();
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
#ifdef MNN_HAS_PRESERVE_INPUT_TYPE
        config.preserveInputType = preserve_input_type != 0;
#endif

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

// ── Buffer-based conversion ──────────────────────────────────────

/// Convert ONNX model bytes directly to MNN model bytes.
/// Uses RAII temp file guards that auto-cleanup on any exit path.
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
    ensure_cleanup_registered();
    result->success = 0;
    result->error_msg[0] = '\0';
    result->data = nullptr;
    result->size = 0;

    if (!onnx_data || onnx_len == 0) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "NULL or empty ONNX data");
        return;
    }

    // RAII temp file guards — auto-cleanup on any exit path
    TempFileGuard in_guard("mnn_onnx_XXXXXX");
    if (!in_guard.active) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "failed to create input tempfile");
        return;
    }

    const char* in_template = in_guard.path.c_str();

    // Write ONNX data to temp file
    {
        int fd = open(in_template, O_WRONLY);
        if (fd < 0) {
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg), "failed to open input tempfile");
            return;
        }
        size_t written = 0;
        while (written < onnx_len) {
            ssize_t n = write(fd, (const char*)onnx_data + written, onnx_len - written);
            if (n < 0) {
                close(fd);
                result->success = -1;
                snprintf(result->error_msg, sizeof(result->error_msg), "write input failed");
                return;
            }
            written += n;
        }
        close(fd);
    }

    TempFileGuard out_guard("mnn_out_XXXXXX");
    if (!out_guard.active) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "failed to create output tempfile");
        return;
    }

    const char* out_template = out_guard.path.c_str();

    try {
        modelConfig config;
        config.model = modelConfig::ONNX;
        config.modelFile = in_template;
        config.MNNModel = out_template;
        config.bizCode = "MNN";
        config.optimizeLevel = 1;
        config.weightQuantBits = 0;
        config.saveHalfFloat = false;
#ifdef MNN_HAS_PRESERVE_INPUT_TYPE
        config.preserveInputType = true;
#endif

        if (!MNN::Cli::convertModel(config)) {
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg),
                     "MNN::Cli::convertModel returned false");
            return;
        }

        // Read output file into allocated buffer
        FILE* f = fopen(out_template, "rb");
        if (!f) {
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
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg),
                     "Failed to allocate %ld bytes", fsize);
            return;
        }
        size_t total = fread(mnn_data, 1, fsize, f);
        fclose(f);

        // TempFileGuard destructors will unlink both temp files.
        // Copy data pointer to result (ownership transfers to caller).
        result->data = mnn_data;
        result->size = total;
    } catch (const std::exception& e) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "%s", e.what());
        if (result->data) { free(result->data); result->data = nullptr; }
    } catch (...) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "Unknown exception");
        if (result->data) { free(result->data); result->data = nullptr; }
    }
}

} // extern "C"
