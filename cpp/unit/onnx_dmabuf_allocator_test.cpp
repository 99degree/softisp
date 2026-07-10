#include "../core/onnx_microblocks/onnx_dmabuf_allocator.h"
#include <iostream>

int main() {
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
  Ort::SessionOptions opt;
  DmabufAllocator alloc(env, opt);

  TensorRef tref;
  tref.is_dmabuf = false;
  tref.ptr = nullptr;
  tref.shape = {1, 1};

  auto res = alloc.create_tensor_from_ref(nullptr, tref);
  if (res.tensor.IsTensor()) {
    std::cout << "Allocator produced a tensor (copy path) OK\n";
  } else {
    std::cout << "Allocator fallback path produced no tensor\n";
  }
  return 0;
}