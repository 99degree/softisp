// test_isp_fusion.cpp — Compile-time + runtime test for IspChainFusion pass.
// Reads .mnn files, deserializes via FlatBuffers, runs 3-pass ISP chain fusion,
// and prints per-pass statistics.
//
// Build:
//   cd cam-rust/cam-isp/vendor/mnn
//   clang++ -std=c++17 -O0 -g \
//     -I schema/current \
//     -I include \
//     -I tools/converter/include \
#    -I tools/converter/source/optimizer/postconvert \
//     -I ~/MNN/3rd_party/flatbuffers/include \
//     test_isp_fusion.cpp \
//     tools/converter/source/optimizer/postconvert/IspChainFusion.cpp \
//     -o test_isp_fusion
//
// Run:
//   LD_LIBRARY_PATH=../../lib/aarch64-v8a ./test_isp_fusion [model.mnn ...]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

// MNN headers
#include <MNN/MNNDefine.h>
#include "MNN_generated.h"
#include "TensorflowOp_generated.h"

// ISP fusion pass (includes isp_fusion_patterns.h → ExactPattern.h)
#include "IspChainFusion.cpp"

using namespace MNN;

// Provide the missing modelConfig destructor (declared in config.hpp but
// defined only in the full MNN converter build).
::modelConfig::~modelConfig() = default;

// ─── helpers ───────────────────────────────────────────────────────

static std::vector<uint8_t> read_file(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return {};
    }
    size_t sz = f.tellg();
    f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read(reinterpret_cast<char*>(buf.data()), sz);
    return buf;
}

static int count_ops(const MNN::NetT& net) {
    int n = 0;
    for (auto& op : net.oplists) {
        if (!op) continue;
        if (op->type != MNN::OpType_Extra) ++n;
    }
    return n;
}

static int count_extras(const MNN::NetT& net) {
    int n = 0;
    for (auto& op : net.oplists) {
        if (!op) continue;
        if (op->type == MNN::OpType_Extra) ++n;
    }
    return n;
}

// ─── main ──────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s model.mnn [model2.mnn ...]\n", argv[0]);
        return 1;
    }

    // Print table sizes (compile-time verification)
    printf("=== IspChainFusion Test ===\n");
    printf("Pass0 entries: %zu\n",
           sizeof(kExactFusionTablesPass0) / sizeof(kExactFusionTablesPass0[0]));
    printf("Pass1 entries: %zu\n",
           sizeof(kExactFusionTablesPass1) / sizeof(kExactFusionTablesPass1[0]));
    printf("Profile entries: %zu\n",
           sizeof(kExactProfileVariants) / sizeof(kExactProfileVariants[0]));
    printf("\n");

    int total_ok = 0;
    int total_fail = 0;

    for (int i = 1; i < argc; ++i) {
        const char* path = argv[i];
        printf("--- %s ---\n", path);

        auto buf = read_file(path);
        if (buf.empty()) {
            printf("  SKIP (read failed)\n");
            total_fail++;
            continue;
        }

        // Verify FlatBuffer magic
        if (buf.size() < 8) {
            printf("  SKIP (too small: %zu bytes)\n", buf.size());
            total_fail++;
            continue;
        }

        // Deserialize via FlatBuffers
        flatbuffers::Verifier verifier(buf.data(), buf.size());
        bool ok = MNN::VerifyNetBuffer(verifier);
        if (!ok) {
            printf("  SKIP (FlatBuffer verification failed)\n");
            total_fail++;
            continue;
        }

        auto net = MNN::UnPackNet(buf.data());
        if (!net) {
            printf("  SKIP (UnPackNet failed)\n");
            total_fail++;
            continue;
        }

        int ops_before = count_ops(*net);
        int extras_before = count_extras(*net);
        printf("  ops: %d  extras: %d\n", ops_before, extras_before);

        // Run ISP chain fusion — triggered by config.model == MNN (MNN→MNN)
        modelConfig config;
        config.model = modelConfig::MNN;

        int consumed = optimizeIspChain(net.get(), config);
        printf("  consumed by ISP fusion: %d\n", consumed);

        int ops_after = count_ops(*net);
        int extras_after = count_extras(*net);
        printf("  ops after: %d  extras after: %d\n", ops_after, extras_after);

        if (consumed >= 0) {
            printf("  PASS\n");
            total_ok++;
        } else {
            printf("  FAIL (returned %d)\n", consumed);
            total_fail++;
        }
        printf("\n");
    }

    printf("=== Results: %d PASS, %d FAIL ===\n", total_ok, total_fail);
    return total_fail > 0 ? 1 : 0;
}
