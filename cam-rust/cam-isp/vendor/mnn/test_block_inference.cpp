// test_block_inference.cpp — Per-block MNN inference + pass2 fusion verification.
//
// Build:
//   cd cam-rust/cam-isp/vendor/mnn
//   clang++ -std=c++17 -O0 -g \
//     -I schema/current -I include -I tools/converter/include \
//     -I tools/converter/source/optimizer/postconvert \
//     -I ~/MNN/3rd_party/flatbuffers/include \
//     test_block_inference.cpp -o test_block_inference
//
// Run:
//   env LD_LIBRARY_PATH=... ./test_block_inference block1.mnn block2.mnn ...

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <numeric>

#include <MNN/MNNDefine.h>
#include <MNN/Interpreter.hpp>
#include <MNN/MNNForwardType.h>
#include <MNN/Tensor.hpp>

#include "MNN_generated.h"
#include "TensorflowOp_generated.h"
#include "IspChainFusion.cpp"

using namespace MNN;

::modelConfig::~modelConfig() = default;

// --- helpers ---

static std::vector<uint8_t> read_file(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) { fprintf(stderr, "ERROR: cannot open %s\n", path); return {}; }
    size_t sz = f.tellg();
    f.seekg(0);
    std::vector<uint8_t> buf(sz);
    f.read(reinterpret_cast<char*>(buf.data()), sz);
    return buf;
}

static int count_ops(const NetT& net) {
    int n = 0;
    for (auto& op : net.oplists) { if (op && op->type != OpType_Extra) ++n; }
    return n;
}

static int count_extras(const NetT& net) {
    int n = 0;
    for (auto& op : net.oplists) { if (op && op->type == OpType_Extra) ++n; }
    return n;
}

static const char* basename_of(const char* path) {
    const char* p = strrchr(path, '/');
    return p ? p + 1 : path;
}

// --- inference via Interpreter ---

struct InferenceResult {
    bool ok = false;
    std::string error;
    std::vector<int> input_dims;
    std::vector<int> output_dims;
    std::vector<float> output_sample;
    int total_output_elements = 0;
    float output_mean = 0.0f;
    float output_min = 0.0f;
    float output_max = 0.0f;
};

static InferenceResult run_inference(const char* path) {
    InferenceResult res;
    auto interp = Interpreter::createFromFile(path);
    if (!interp) { res.error = "createFromFile failed"; return res; }

    ScheduleConfig config;
    config.type = MNN_FORWARD_CPU;
    config.numThread = 1;

    auto session = interp->createSession(config);
    if (!session) { res.error = "createSession failed"; Interpreter::destroy(interp); return res; }

    auto input_map = interp->getSessionInputAll(session);
    if (input_map.empty()) { res.error = "no inputs"; Interpreter::destroy(interp); return res; }
    auto input_name = input_map.begin()->first;
    auto* input_tensor = input_map.begin()->second;
    if (!input_tensor) { res.error = "no input tensor"; Interpreter::destroy(interp); return res; }

    auto idims = input_tensor->shape();
    for (int d : idims) res.input_dims.push_back(d);

    interp->resizeTensor(input_tensor, idims);
    {
        int total = 1;
        for (int d : idims) total *= d;
        std::unique_ptr<Tensor> host(Tensor::create<float>(idims, nullptr, input_tensor->getDimensionType()));
        float* ptr = host->host<float>();
        for (int i = 0; i < total; i++) ptr[i] = 0.5f;
        input_tensor->copyFromHostTensor(host.get());
    }

    if (interp->runSession(session) != 0) {
        res.error = "runSession failed"; Interpreter::destroy(interp); return res;
    }

    auto output_map = interp->getSessionOutputAll(session);
    if (output_map.empty()) { res.error = "no outputs"; Interpreter::destroy(interp); return res; }
    auto output_name = output_map.begin()->first;
    auto* output_tensor = output_map.begin()->second;
    if (!output_tensor) { res.error = "no output tensor"; Interpreter::destroy(interp); return res; }

    auto odims = output_tensor->shape();
    for (int d : odims) res.output_dims.push_back(d);
    res.total_output_elements = 1;
    for (int d : odims) res.total_output_elements *= d;

    std::unique_ptr<Tensor> output_host(Tensor::create<float>(odims, nullptr, output_tensor->getDimensionType()));
    output_tensor->copyToHostTensor(output_host.get());

    float* data = output_host->host<float>();
    if (data && res.total_output_elements > 0) {
        int n = std::min(res.total_output_elements, 16);
        res.output_sample.assign(data, data + n);
        double sum = 0;
        res.output_min = data[0]; res.output_max = data[0];
        for (int i = 0; i < res.total_output_elements; i++) {
            sum += data[i];
            if (data[i] < res.output_min) res.output_min = data[i];
            if (data[i] > res.output_max) res.output_max = data[i];
        }
        res.output_mean = static_cast<float>(sum / res.total_output_elements);
        res.ok = true;
    }

    Interpreter::destroy(interp);
    return res;
}

// --- fusion analysis ---

struct FusionResult {
    int pass0 = 0, pass1 = 0, pass2 = 0;
    int ops_before = 0, extras_before = 0;
    int ops_after = 0, extras_after = 0;
};

static FusionResult run_fusion(const char* path) {
    FusionResult res;
    auto buf = read_file(path);
    if (buf.empty() || buf.size() < 8) return res;

    flatbuffers::Verifier verifier(buf.data(), buf.size());
    if (!MNN::VerifyNetBuffer(verifier)) return res;

    auto net = MNN::UnPackNet(buf.data());
    if (!net) return res;

    res.ops_before = count_ops(*net);
    res.extras_before = count_extras(*net);

    auto producer = buildProducerMap(net.get());
    res.pass0 = runPass(net.get(), kExactFusionTablesPass0,
                       sizeof(kExactFusionTablesPass0) / sizeof(ExactPattern), producer);
    if (res.pass0 > 0) producer = buildProducerMap(net.get());

    res.pass1 = runPass(net.get(), kExactFusionTablesPass1,
                       sizeof(kExactFusionTablesPass1) / sizeof(ExactPattern), producer);
    if (res.pass1 > 0) producer = buildProducerMap(net.get());

    res.pass2 = runPass(net.get(), kExactProfileVariants,
                       sizeof(kExactProfileVariants) / sizeof(ExactPattern), producer);

    res.ops_after = count_ops(*net);
    res.extras_after = count_extras(*net);
    return res;
}

// --- main ---

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "Usage: %s block.mnn ...\n", argv[0]); return 1; }

    int total_ok = 0, total_fail = 0;
    printf("[\n");

    for (int i = 1; i < argc; ++i) {
        const char* path = argv[i];
        const char* name = basename_of(path);
        bool comma = (i + 1 < argc);

        auto inf = run_inference(path);
        auto fus = run_fusion(path);
        int total_consumed = fus.pass0 + fus.pass1 + fus.pass2;
        const char* status = (inf.ok && total_consumed >= 0) ? "PASS" : "FAIL";
        if (inf.ok && total_consumed >= 0) total_ok++; else total_fail++;

        printf("  {\n");
        printf("    \"model\": \"%s\",\n", name);
        printf("    \"status\": \"%s\",\n", status);

        printf("    \"inference\": {");
        if (inf.ok) {
            printf("\"input_dims\": [");
            for (int j = 0; j < (int)inf.input_dims.size(); j++) {
                if (j) printf(", "); printf("%d", inf.input_dims[j]);
            }
            printf("], \"output_dims\": [");
            for (int j = 0; j < (int)inf.output_dims.size(); j++) {
                if (j) printf(", "); printf("%d", inf.output_dims[j]);
            }
            printf("], \"total_elements\": %d, \"mean\": %.6f, \"min\": %.6f, \"max\": %.6f",
                   inf.total_output_elements, inf.output_mean, inf.output_min, inf.output_max);
            if (!inf.output_sample.empty()) {
                printf(", \"sample\": [");
                for (int j = 0; j < (int)inf.output_sample.size(); j++) {
                    if (j) printf(", "); printf("%.6f", inf.output_sample[j]);
                }
                printf("]");
            }
        } else {
            printf("\"error\": \"%s\"", inf.error.c_str());
        }
        printf("},\n");

        printf("    \"fusion\": {");
        printf("\"ops_before\": %d, \"extras_before\": %d, ", fus.ops_before, fus.extras_before);
        printf("\"pass0\": {\"consumed\": %d}, ", fus.pass0);
        printf("\"pass1\": {\"consumed\": %d}, ", fus.pass1);
        printf("\"pass2_profile\": {\"consumed\": %d}, ", fus.pass2);
        printf("\"total_consumed\": %d, ", total_consumed);
        printf("\"ops_after\": %d, \"extras_after\": %d", fus.ops_after, fus.extras_after);
        printf("}\n");

        printf("  }%s\n", comma ? "," : "");
    }

    printf("],\n");
    printf("{\"summary\": {\"total_models\": %d, \"pass\": %d, \"fail\": %d}}\n",
           argc - 1, total_ok, total_fail);
    return total_fail > 0 ? 1 : 0;
}
