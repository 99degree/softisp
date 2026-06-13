#include <MNN/expr/Expr.hpp>
#include <MNN/expr/Module.hpp>
#include <MNN/expr/ExprCreator.hpp>
#include <iostream>
using namespace MNN::Express;
int main() {
    const char* path = "/data/data/com.termux/files/home/MNN/benchmark/models/MobileNetV2_224.mnn";
    
    // Test 1: Module::load with just filename (no names)
    auto* m1 = Module::load({}, {}, path);
    std::cout << "Module::load empty names: " << (m1 ? "OK" : "NULL") << std::endl;
    if (m1) Module::destroy(m1);
    
    // Test 2: Module::load with correct names
    auto* m2 = Module::load({"input"}, {"MobilenetV2/Predictions/Reshape_1"}, path);
    std::cout << "Module::load with names: " << (m2 ? "OK" : "NULL") << std::endl;
    if (m2) Module::destroy(m2);
    
    // Test 3: Variable::load + Module::extract + onForward + readMap via different approach
    auto allVars = Variable::load(path);
    std::map<std::string, VARP> vm;
    for (auto& v : allVars) vm[v->name()] = v;
    
    auto* m3 = Module::extract({vm["input"]}, {vm["MobilenetV2/Predictions/Reshape_1"]}, false);
    auto input = _Input({1,3,224,224}, NCHW, halide_type_of<float>());
    float* ptr = input->writeMap<float>();
    memset(ptr, 0, 3*224*224*4);
    auto outputs = m3->onForward({input});
    
    // Try to get data through Tensor API
    auto* tensor = outputs[0]->get()->getTensor();
    if (tensor) {
        std::cout << "Tensor: host=" << tensor->host<float>() << " size=" << tensor->elementSize() << std::endl;
        if (tensor->host<float>()) {
            std::cout << "First: " << tensor->host<float>()[0] << std::endl;
        }
    } else {
        std::cout << "getTensor NULL" << std::endl;
    }
    
    // Try Variable::loadMap approach
    auto allMap = Variable::loadMap(path);
    std::cout << "loadMap keys: " << allMap.size() << std::endl;
    
    Module::destroy(m3);
    return 0;
}
