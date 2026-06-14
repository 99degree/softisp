//
//  mnnconvert_main.cpp
//  mnnconvert - ONNX-to-MNN standalone converter (executable entry)
//
//  Usage: mnnconvert -f ONNX --modelFile model.onnx --MNNModel model.mnn --bizCode MNN
//

#include "cli.hpp"

int main(int argc, char* argv[]) {
    modelConfig cfg;
    auto ok = MNN::Cli::initializeMNNConvertArgs(cfg, argc, argv);
    if (!ok) return 0;
    MNN::Cli::convertModel(cfg);
    return 0;
}