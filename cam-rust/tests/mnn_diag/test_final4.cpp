#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;
int main() {
    auto* interp = Interpreter::createFromFile("test_final2.mnn");
    ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
    auto* sess = interp->createSession(cfg);
    auto* in = interp->getSessionInput(sess, nullptr);
    
    std::cout << "Input shape empty: " << (in->shape().empty() ? "yes" : "no") << "\n";
    // Try getting input dimensions individually
    std::cout << "dimensions: " << in->dimensions() << "\n";
    for (int i = 0; i < in->dimensions(); i++) {
        std::cout << "  dim[" << i << "]=" << in->shape()[i] << "\n";
    }
    std::cout << "elementSize: " << in->elementSize() << "\n";
    std::cout << "size: " << in->size() << "\n";
    // Try writing data directly (zero-copy)
    void* hostData = malloc(48*64*4); // UINT16 * 1 channel
    uint16_t* p = (uint16_t*)hostData;
    for (int i = 0; i < 48*64; i++) p[i] = (uint16_t)(i % 256);
    
    // For zero-copy: create a tensor that wraps our buffer
    auto* hostIn = Tensor::create({1, 1, 48, 64}, halide_type_of<uint16_t>(), hostData, Tensor::CAFFE);
    std::cout << "hostIn=" << (void*)hostIn << "\n";
    if (!hostIn) return 1;
    std::cout << "hostIn host=" << (void*)hostIn->host<void>() << "\n";
    std::cout << "hostIn size=" << hostIn->size() << "\n";
    
    in->copyFromHostTensor(hostIn);
    std::cout << "Copied\n";
    
    interp->runSession(sess);
    std::cout << "Ran\n";
    
    auto* out = interp->getSessionOutput(sess, nullptr);
    std::cout << "out=" << (void*)out << "\n";
    
    Tensor::destroy(hostIn);
    free(hostData);
    interp->releaseSession(sess);
    Interpreter::destroy(interp);
    return 0;
}
