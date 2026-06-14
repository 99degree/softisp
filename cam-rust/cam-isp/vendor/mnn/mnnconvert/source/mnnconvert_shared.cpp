//
//  mnnconvert_shared.cpp
//  mnnconvert - Shared library implementation
//
#include "mnnconvert_api.h"
#include "config.hpp"
#include "cli.hpp"
#include <cstring>
#include <string>

// The MNN::Cli API expects argc/argv. We build synthetic args.
static int _convert(const char* onnxPath, const char* mnnPath,
                    const MnnConvert_Options* opts, std::string& outMsg) {
    modelConfig cfg;
    MnnConvert_Options dflt = MNN_CONVERT_OPTIONS_DEFAULT;
    if (!opts) opts = &dflt;

    cfg.model = modelConfig::ONNX;
    cfg.modelFile = onnxPath;
    cfg.MNNModel = mnnPath;
    cfg.bizCode = opts->bizCode ? opts->bizCode : dflt.bizCode;
    cfg.optimizeLevel = opts->optimizeLevel;
    cfg.saveHalfFloat = opts->fp16 ? true : false;
    cfg.weightQuantBits = opts->weightQuantBits;
    cfg.weightQuantBlock = opts->weightQuantBlock;
    cfg.saveStaticModel = opts->saveStaticModel ? true : false;
    cfg.transformerFuse = opts->transformerFuse ? true : false;
    cfg.allowCustomOp = opts->allowCustomOp ? true : false;
    cfg.useGeluApproximation = opts->useGeluApproximation;
    if (opts->inputConfigFile)
        cfg.inputConfigFile = opts->inputConfigFile;
    if (opts->targetVersion > 0.0f)
        cfg.targetVersion = opts->targetVersion;

    // Run conversion
    bool ok = MNN::Cli::convertModel(cfg);
    outMsg = ok ? "conversion succeeded" : "conversion failed";
    return ok ? 0 : 1;
}

extern "C" {

MnnConvert_Result MnnConvert_OnnxToMnn(const char* onnxPath, const char* mnnPath,
                                       const MnnConvert_Options* options) {
    MnnConvert_Result result;
    result.code = 0;
    result.message = nullptr;
    if (!onnxPath || !mnnPath) {
        result.code = -1;
        result.message = strdup("onnxPath and mnnPath must not be NULL");
        return result;
    }
    std::string msg;
    int rc = _convert(onnxPath, mnnPath, options, msg);
    result.code = rc;
    result.message = strdup(msg.c_str());
    return result;
}

void MnnConvert_FreeResult(MnnConvert_Result* result) {
    if (result && result->message) {
        free(result->message);
        result->message = nullptr;
    }
}

} // extern "C"