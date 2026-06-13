#include <MNN/HalideRuntime.h>
#include <iostream>
using namespace std;

int main() {
    auto tf = halide_type_of<float>();
    cout << "float: code=" << (int)tf.code << " bits=" << (int)tf.bits << " bytes=" << tf.bytes() << "\n";
    auto ti = halide_type_of<int32_t>();
    cout << "int32: code=" << (int)ti.code << " bits=" << (int)ti.bits << " bytes=" << ti.bytes() << "\n";
    auto tu8 = halide_type_of<uint8_t>();
    cout << "uint8: code=" << (int)tu8.code << " bits=" << (int)tu8.bits << " bytes=" << tu8.bytes() << "\n";
    auto ti16 = halide_type_of<int16_t>();
    cout << "int16: code=" << (int)ti16.code << " bits=" << (int)ti16.bits << " bytes=" << ti16.bytes() << "\n";
    return 0;
}
