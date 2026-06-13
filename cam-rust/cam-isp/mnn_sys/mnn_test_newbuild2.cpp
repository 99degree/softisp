#include <MNN/expr/Expr.hpp>
#include <MNN/expr/Module.hpp>
#include <MNN/expr/ExprCreator.hpp>
#include <iostream>
#include <map>
#include <cstring>
using namespace MNN::Express;

int main() {
    const char* path = "/data/data/com.termux/files/home/MNN/benchmark/models/MobileNetV2_224.mnn";
    
    // Module::load
    auto* m1 = Module::load({}, {}, path);
    std::cout << "Module::load: " << (m1 ? "OK" : "NULL") << std::endl;
    if (m1) {
        auto outputs = m1->onForward({_Input({1,3,224,224}, NCHW, halide_type_of<float>())});
        std::cout << "  onForward: " << outputs.size() << std::endl;
        Module::destroy(m1);
    }
    
    // Module::load with names
    auto* m2 = Module::load({"input"}, {"MobilenetV2/Predictions/Reshape_1"}, path);
    std::cout << "Module::load names: " << (m2 ? "OK" : "NULL") << std::endl;
    if (m2) Module::destroy(m2);
    
    // Variable::load + extract
    auto allVars = Variable::load(path);
    std::map<std::string, VARP> vm;
    for (auto& v : allVars) vm[v->name()] = v;
    
    auto* m3 = Module::extract({vm["input"]}, {vm["MobilenetV2/Predictions/Reshape_1"]}, false);
    std::cout << "Module::extract: " << (m3 ? "OK" : "NULL") << std::endl;
    
    if (m3) {
        auto input = _Input({1,3,224,224}, NCHW, halide_type_of<float>());
        float* ptr = input->writeMap<float>();
        memset(ptr, 0, 3*224*224*4);
        
        auto outputs = m3->onForward({input});
        std::cout << "onForward: " << outputs.size() << " outputs" << std::endl;
        
        if (!outputs.empty()) {
            auto* info = outputs[0]->getInfo();
            if (info) {
                std::cout << "output shape: [" << info->dim[0] << "," << info->dim[1] << "] size=" << info->size << std::endl;
                
                // Direct read from Variable using readInternal
                // The issue is that readMap returns NULL.
                // Let's try a workaround: create a copy on host
                auto copy = _Input(info->dim, info->order, info->type);
                // Force compute the source
                Variable::prepareCompute({outputs[0]}, true);
                Variable::compute({outputs[0]}, true);
                
                const float* d = outputs[0]->readMap<float>();
                std::cout << "readMap after compute: " << (d ? "OK" : "NULL") << std::endl;
                
                // Try with forceCPU flag
                Variable::prepareCompute({outputs[0]}, true);
                // Need to set the executor to CPU mode
                
                // Let's try creating a fresh input and running again
                // with correct format
                auto input2 = _Input({1,224,224,3}, NHWC, halide_type_of<float>());
                float* p2 = input2->writeMap<float>();
                memset(p2, 0, 3*224*224*4);
                auto outputs2 = m3->onForward({input2});
                if (!outputs2.empty()) {
                    auto* info2 = outputs2[0]->getInfo();
                    std::cout << "NHWC output: dims=[" << info2->dim[0] << "," << info2->dim[1] << "] size=" << info2->size << std::endl;
                    Variable::compute({outputs2[0]}, true);
                    const float* d2 = outputs2[0]->readMap<float>();
                    std::cout << "NHWC readMap: " << (d2 ? "OK" : "NULL") << std::endl;
                    if (d2) std::cout << "First: " << d2[0] << std::endl;
                }
            }
        }
        Module::destroy(m3);
    }
    
    return 0;
}
