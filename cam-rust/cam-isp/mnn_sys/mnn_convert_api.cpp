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
    if (result == nullptr) {
        return;
    }
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
    if (result == nullptr) {
        return;
    }
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

// ── memfd-based conversion (zero heap round-trip) ──────────────────────
//
// Convert ONNX bytes -> MNN bytes entirely via file descriptors:
//   - input ONNX written to a memfd (no real disk)
//   - converter writes .mnn to an output memfd
//   - returns the output memfd to the caller
// The caller loads MNN from /proc/self/fd/<fd> (MNN::Interpreter::createFromFile)
// so the model never lands in the Rust/heap Vec. On Linux memfd_create is used;
// on other platforms a temp file fd is returned instead.

/// Portable memfd_create: some NDK/sysroot headers omit the SYS_memfd_create
/// macro even when the kernel supports the syscall. Define __NR_memfd_create
/// and the MFD_* flags explicitly when missing, then fall back to a temp file
/// (still no real disk round-trip concern for the ONNX→MNN path on hosts
/// without memfd).
#if defined(__linux__) && !defined(SYS_memfd_create)
#include <asm/unistd.h>
#if !defined(__NR_memfd_create)
#if defined(__aarch64__)
#define __NR_memfd_create 279
#elif defined(__arm__)
#define __NR_memfd_create 385
#elif defined(__x86_64__)
#define __NR_memfd_create 319
#elif defined(__i386__)
#define __NR_memfd_create 356
#else
#define __NR_memfd_create 0
#endif
#endif
#define SYS_memfd_create __NR_memfd_create
#endif

#if defined(__linux__) && !defined(MFD_CLOEXEC)
#define MFD_CLOEXEC 0x0001U
#define MFD_ALLOW_SEALING 0x0002U
#endif

/// Open a memfd (or temp file fallback) of `size` bytes, return the fd.
static int open_output_fd(const char* name, size_t size) {
#if defined(__linux__)
#if SYS_memfd_create != 0
    int fd = (int)syscall(SYS_memfd_create, name, (unsigned int)(MFD_CLOEXEC | MFD_ALLOW_SEALING));
    if (fd < 0) return -1;
    if (ftruncate(fd, (off_t)size) != 0) { close(fd); return -1; }
    return fd;
#else
    (void)name;
    char tmpl[512];
    snprintf(tmpl, sizeof(tmpl), "/tmp/mnn_XXXXXX");
    int fd = mkstemp(tmpl);
    if (fd < 0) return -1;
    if (ftruncate(fd, (off_t)size) != 0) { close(fd); return -1; }
    return fd;
#endif
#else
    char tmpl[512];
    snprintf(tmpl, sizeof(tmpl), "/tmp/%s_XXXXXX", name);
    int fd = mkstemp(tmpl);
    if (fd < 0) return -1;
    if (ftruncate(fd, (off_t)size) != 0) { close(fd); return -1; }
    return fd;
#endif
}

/// @return output fd on success (>=0), or -1 on error (msg in error_msg).
MNN_PUBLIC int mnn_convert_onnx_memfd(
    const void* onnx_data,
    size_t onnx_len,
    char* error_msg,
    size_t error_msg_cap)
{
    if (!onnx_data || onnx_len == 0 || !error_msg || error_msg_cap == 0) {
        if (error_msg && error_msg_cap) {
            snprintf(error_msg, error_msg_cap, "NULL/empty ONNX or bad out buffer");
        }
        return -1;
    }
    error_msg[0] = '\0';

    // Input memfd holds the ONNX protobuf; we point the converter at it.
    int in_fd = open_output_fd("mnn_onnx", onnx_len > 0 ? onnx_len : 1);
    if (in_fd < 0) {
        snprintf(error_msg, error_msg_cap, "failed to create input memfd");
        return -1;
    }
    {
        size_t written = 0;
        while (written < onnx_len) {
            ssize_t n = write(in_fd, (const char*)onnx_data + written, onnx_len - written);
            if (n < 0) {
                close(in_fd);
                snprintf(error_msg, error_msg_cap, "write input memfd failed");
                return -1;
            }
            written += (size_t)n;
        }
        // Seal the input read-only so the converter cannot mutate it.
#if defined(__linux__)
        fcntl(in_fd, F_ADD_SEALS, F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_WRITE | F_SEAL_SEAL);
#endif
    }

    // Output memfd placeholder (1 byte); the converter grows it via ftruncate.
    int out_fd = open_output_fd("mnn_out", 1);
    if (out_fd < 0) {
        close(in_fd);
        snprintf(error_msg, error_msg_cap, "failed to create output memfd");
        return -1;
    }

    char in_path[64];
    char out_path[64];
    snprintf(in_path, sizeof(in_path), "/proc/self/fd/%d", in_fd);
    snprintf(out_path, sizeof(out_path), "/proc/self/fd/%d", out_fd);

    int rc = -1;
    try {
        modelConfig config;
        config.model = modelConfig::ONNX;
        config.modelFile = in_path;
        config.MNNModel = out_path;
        config.bizCode = "MNN";
        config.optimizeLevel = 1;
        config.weightQuantBits = 0;
        config.saveHalfFloat = false;
#ifdef MNN_HAS_PRESERVE_INPUT_TYPE
        config.preserveInputType = true;
#endif

        if (!MNN::Cli::convertModel(config)) {
            snprintf(error_msg, error_msg_cap, "MNN::Cli::convertModel returned false");
            rc = -1;
        } else {
            rc = out_fd;  // success: hand the fd to the caller
        }
    } catch (const std::exception& e) {
        snprintf(error_msg, error_msg_cap, "%s", e.what());
        rc = -1;
    } catch (...) {
        snprintf(error_msg, error_msg_cap, "Unknown exception");
        rc = -1;
    }

    close(in_fd);
    if (rc < 0) {
        close(out_fd);
    }
    // On success out_fd is returned and owned by the caller (must be closed).
    return rc;
}

// ── MNN→MNN conversion (Pass1: applies IspChainFusion) ──────────────────
//
// Same in-process buffer pattern as mnn_convert_onnx_buffer, but the input
// is an existing MNN model. The converter re-parses it with framework=MNN,
// which runs the IspChainFusion post-converter: standard primitive ops
// (Conv/BinaryOp/Pool/...) are fused into isp.* custom ops carrying
// pre-compiled SPIR-V.

/// Convert MNN bytes to MNN bytes (framework=MNN → IspChainFusion).
/// Caller must free result->data via MnnConvert_FreeBuffer.
MNN_PUBLIC void mnn_convert_mnn_buffer(
    const void* mnn_data,
    size_t mnn_len,
    MnnConvertBufferResult* result)
{
    if (result == nullptr) {
        return;
    }
    ensure_cleanup_registered();
    result->success = 0;
    result->error_msg[0] = '\0';
    result->data = nullptr;
    result->size = 0;

    if (!mnn_data || mnn_len == 0) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "NULL or empty MNN data");
        return;
    }

    TempFileGuard in_guard("mnn_in_XXXXXX");
    if (!in_guard.active) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "failed to create input tempfile");
        return;
    }
    {
        int fd = open(in_guard.path.c_str(), O_WRONLY);
        if (fd < 0) {
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg), "failed to open input tempfile");
            return;
        }
        size_t written = 0;
        while (written < mnn_len) {
            ssize_t n = write(fd, (const char*)mnn_data + written, mnn_len - written);
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

    try {
        modelConfig config;
        config.model = modelConfig::MNN;   // MNN→MNN: triggers IspChainFusion
        config.modelFile = in_guard.path;
        config.MNNModel = out_guard.path;
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

        FILE* f = fopen(out_guard.path.c_str(), "rb");
        if (!f) {
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg), "failed to open output");
            return;
        }
        fseek(f, 0, SEEK_END);
        long fsize = ftell(f);
        rewind(f);

        void* mnn_out = malloc(fsize > 0 ? fsize : 1);
        if (!mnn_out) {
            fclose(f);
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg),
                     "Failed to allocate %ld bytes", fsize);
            return;
        }
        size_t total = fread(mnn_out, 1, fsize, f);
        fclose(f);

        result->data = mnn_out;
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

// ── MNN→JSON dump (Pass1 opset verification) ────────────────────────────
//
// Dumps an MNN model to the flatbuffer-JSON text format via
// MNN::Cli::mnn2json. The JSON contains one object per op with fields
// "type" (MNN OpType enum name, e.g. "Conv", "BinaryOp") and, for custom
// ops, "main_type": "Extra" with "main.type" = the isp.* string.
// The test parses this JSON in Rust to verify the B-test constraint:
// Pass1 MNN must contain ONLY isp.* custom opset operations.

/// Result of a MNN→JSON dump.
/// Caller must free `data` via MnnConvert_FreeBuffer.
typedef struct {
    int success;    // 0 = success, -1 = error
    char error_msg[1024];
    void* data;     // JSON string bytes (NUL-terminated, malloc'd)
    size_t size;    // size of JSON data (incl. NUL)
} MnnConvertJsonResult;

/// Dump MNN bytes to JSON text via MNN::Cli::mnn2json (in-process).
MNN_PUBLIC void mnn_dump_mnn_to_json(
    const void* mnn_data,
    size_t mnn_len,
    MnnConvertJsonResult* result)
{
    if (result == nullptr) {
        return;
    }
    ensure_cleanup_registered();
    result->success = 0;
    result->error_msg[0] = '\0';
    result->data = nullptr;
    result->size = 0;

    if (!mnn_data || mnn_len == 0) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "NULL or empty MNN data");
        return;
    }

    TempFileGuard in_guard("mnn_in_XXXXXX");
    if (!in_guard.active) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "failed to create input tempfile");
        return;
    }
    {
        int fd = open(in_guard.path.c_str(), O_WRONLY);
        if (fd < 0) {
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg), "failed to open input tempfile");
            return;
        }
        size_t written = 0;
        while (written < mnn_len) {
            ssize_t n = write(fd, (const char*)mnn_data + written, mnn_len - written);
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

    TempFileGuard out_guard("mnn_json_XXXXXX");
    if (!out_guard.active) {
        result->success = -1;
        snprintf(result->error_msg, sizeof(result->error_msg), "failed to create json tempfile");
        return;
    }

    try {
        modelConfig config;
        config.model = modelConfig::MNN;
        config.modelFile = in_guard.path;
        config.MNNModel = out_guard.path;
        config.mnn2json = true;

        if (!MNN::Cli::convertModel(config)) {
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg),
                     "MNN::Cli::convertModel (mnn2json) returned false");
            return;
        }

        FILE* f = fopen(out_guard.path.c_str(), "rb");
        if (!f) {
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg), "failed to open json output");
            return;
        }
        fseek(f, 0, SEEK_END);
        long fsize = ftell(f);
        rewind(f);

        void* json_data = malloc(fsize + 1);
        if (!json_data) {
            fclose(f);
            result->success = -1;
            snprintf(result->error_msg, sizeof(result->error_msg),
                     "Failed to allocate %ld bytes", fsize);
            return;
        }
        size_t total = fread(json_data, 1, fsize, f);
        fclose(f);
        ((char*)json_data)[total] = '\0';  // NUL-terminate

        result->data = json_data;
        result->size = total + 1;
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
