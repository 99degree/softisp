// test_isp_fusion.cpp — Per-pass JSON verification for IspChainFusion.
// Reads .mnn files, runs pass0/pass1/profile separately, prints JSON results.
//
// Build:
//   cd cam-rust/cam-isp/vendor/mnn
//   clang++ -std=c++17 -O0 -g \
//     -I schema/current -I include -I tools/converter/include \
//     -I tools/converter/source/optimizer/postconvert \
//     -I ~/MNN/3rd_party/flatbuffers/include \
//     test_isp_fusion.cpp -o test_isp_fusion
//
// Run:
//   env LD_LIBRARY_PATH=... ./test_isp_fusion model.mnn [model2.mnn ...]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <MNN/MNNDefine.h>
#include "MNN_generated.h"
#include "TensorflowOp_generated.h"

#include "IspChainFusion.cpp"

using namespace MNN;

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

static int count_ops(const NetT& net) {
    int n = 0;
    for (auto& op : net.oplists) {
        if (!op) continue;
        if (op->type != OpType_Extra) ++n;
    }
    return n;
}

static int count_extras(const NetT& net) {
    int n = 0;
    for (auto& op : net.oplists) {
        if (!op) continue;
        if (op->type == OpType_Extra) ++n;
    }
    return n;
}

// Extract basename from path for JSON output
static const char* basename_of(const char* path) {
    const char* p = strrchr(path, '/');
    return p ? p + 1 : path;
}

// ─── main ──────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s model.mnn [model2.mnn ...]\n", argv[0]);
        return 1;
    }

    int total_ok = 0;
    int total_fail = 0;

    // JSON array open
    printf("[\n");

    for (int i = 1; i < argc; ++i) {
        const char* path = argv[i];
        const char* name = basename_of(path);

        auto buf = read_file(path);
        if (buf.empty()) {
            printf("  {\"model\": \"%s\", \"status\": \"SKIP\", \"reason\": \"read_failed\"}%s\n",
                   name, i + 1 < argc ? "," : "");
            total_fail++;
            continue;
        }

        if (buf.size() < 8) {
            printf("  {\"model\": \"%s\", \"status\": \"SKIP\", \"reason\": \"too_small\"}%s\n",
                   name, i + 1 < argc ? "," : "");
            total_fail++;
            continue;
        }

        flatbuffers::Verifier verifier(buf.data(), buf.size());
        bool ok = MNN::VerifyNetBuffer(verifier);
        if (!ok) {
            printf("  {\"model\": \"%s\", \"status\": \"SKIP\", \"reason\": \"flatbuf_verify\"}%s\n",
                   name, i + 1 < argc ? "," : "");
            total_fail++;
            continue;
        }

        auto net = MNN::UnPackNet(buf.data());
        if (!net) {
            printf("  {\"model\": \"%s\", \"status\": \"SKIP\", \"reason\": \"unpack\"}%s\n",
                   name, i + 1 < argc ? "," : "");
            total_fail++;
            continue;
        }

        int ops_before = count_ops(*net);
        int extras_before = count_extras(*net);

        // ── Pass 0: first-match fusion ────────────────────────
        auto producer = buildProducerMap(net.get());
        int pass0 = runPass(net.get(), kExactFusionTablesPass0,
                           sizeof(kExactFusionTablesPass0) / sizeof(ExactPattern),
                           producer);

        // Rebuild producer map after pass0 mutations
        if (pass0 > 0) {
            producer = buildProducerMap(net.get());
        }

        // ── Pass 1: guard chains ──────────────────────────────
        int pass1 = runPass(net.get(), kExactFusionTablesPass1,
                           sizeof(kExactFusionTablesPass1) / sizeof(ExactPattern),
                           producer);

        // Rebuild producer map after pass1 mutations
        if (pass1 > 0) {
            producer = buildProducerMap(net.get());
        }

        // ── Pass 2: profile structural matching ───────────────
        int pass2 = runPass(net.get(), kExactProfileVariants,
                           sizeof(kExactProfileVariants) / sizeof(ExactPattern),
                           producer);

        int ops_after = count_ops(*net);
        int extras_after = count_extras(*net);
        int total_consumed = pass0 + pass1 + pass2;

        const char* status = total_consumed >= 0 ? "PASS" : "FAIL";
        if (total_consumed >= 0) total_ok++; else total_fail++;

        printf("  {\"model\": \"%s\", \"status\": \"%s\", "
               "\"ops_before\": %d, \"extras_before\": %d, "
               "\"pass0\": {\"consumed\": %d}, "
               "\"pass1\": {\"consumed\": %d}, "
               "\"pass2_profile\": {\"consumed\": %d}, "
               "\"total_consumed\": %d, "
               "\"ops_after\": %d, \"extras_after\": %d}%s\n",
               name, status,
               ops_before, extras_before,
               pass0, pass1, pass2,
               total_consumed,
               ops_after, extras_after,
               i + 1 < argc ? "," : "");
    }

    // JSON array close + summary
    printf("],\n");
    printf("{\"summary\": {\"pass0_entries\": %zu, \"pass1_entries\": %zu, "
           "\"profile_entries\": %zu, \"total_models\": %d, "
           "\"pass\": %d, \"fail\": %d}}\n",
           sizeof(kExactFusionTablesPass0) / sizeof(kExactFusionTablesPass0[0]),
           sizeof(kExactFusionTablesPass1) / sizeof(kExactFusionTablesPass1[0]),
           sizeof(kExactProfileVariants) / sizeof(kExactProfileVariants[0]),
           argc - 1, total_ok, total_fail);

    return total_fail > 0 ? 1 : 0;
}
