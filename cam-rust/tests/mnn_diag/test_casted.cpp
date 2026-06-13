#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <iostream>
using namespace MNN;

int main() {
    for (auto model : {"casted.mnn", "identity.mnn"}) {
        std::cout << "\n=== Testing " << model << " ===\n";
        auto* interp = Interpreter::createFromFile(model);
        ScheduleConfig cfg; cfg.type = MNN_FORWARD_CPU; cfg.numThread = 1;
        auto* sess = interp->createSession(cfg);
        auto* in = interp->getSessionInput(sess, nullptr);
        
        int H = 4, W = 4;
        interp->resizeTensor(in, {1, 1, H, W});
        interp->resizeSession(sess);
        
        auto inType = in->getType();
        std::cout << "Input type: code=" << (int)inType.code << "\n";
        std::cout << "NOTE: code=0=INT32, code=2=FLOAT on this platform\n";
        
        // Create CAFFE-format host tensor with the SAME type as device expects
        auto* hostIn = Tensor::create({1, 1, H, W}, inType, nullptr, Tensor::CAFFE);
        
        // If device expects FLOAT (code=2), write float data
        // If device expects INT32 (code=0), write int32 data
        if (inType.code == 2) { // FLOAT on this platform
            float* d = (float*)hostIn->host<void>();
            for (int i = 0; i < H*W; i++) d[i] = (float)(i % 256);
            std::cout << "Wrote FLOAT data\n";
        } else { // INT32 on this platform
            int32_t* d = (int32_t*)hostIn->host<void>();
            for (int i = 0; i < H*W; i++) d[i] = (int32_t)(i % 256);
            std::cout << "Wrote INT32 data\n";
        }
        
        in->copyFromHostTensor(hostIn);
        interp->runSession(sess);
        
        auto* out = interp->getSessionOutput(sess, nullptr);
        std::cout << "Output shape: ["; 
        for (auto d : out->shape()) std::cout << d << ",";
        std::cout << "]\n";
        
        auto* hostOut = new Tensor(out, Tensor::CAFFE);
        out->copyToHostTensor(hostOut);
        
        auto s = hostOut->shape();
        int n = 1; for (auto d : s) n *= d;
        std::cout << "HostOut: shape n=" << n << "\n";
        
        if (n > 0) {
            float* d = (float*)hostOut->host<void>();
            std::cout << "First 4 (float): " << d[0] << " " << d[1] << " " << d[2] << " " << d[3] << "\n";
            int32_t* di = (int32_t*)d;
            std::cout << "First 4 (int32): " << di[0] << " " << di[1] << " " << di[2] << " " << di[3] << "\n";
        }
        
        Tensor::destroy(hostIn);
        delete hostOut;
        interp->releaseSession(sess);
        Interpreter::destroy(interp);
    }
    return 0;
}
