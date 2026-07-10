#pragma once
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <functional>
#include "../../include/process_item.h"
#include "../../include/ownership_token.h"
#include <unordered_map>
#include <mutex>

// Forward-compatible TensorRef definition (shared)
struct TensorRef {
  void* ptr = nullptr;
  int fd = -1;
  bool is_dmabuf = false;
  std::vector<int64_t> shape;
  ONNXTensorElementDataType dtype = ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
  size_t byte_length() const { size_t s = 1; for (auto d : shape) s *= (size_t)d; return s * (dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ? sizeof(float) : 1); }
};

struct TensorCreateResult {
  Ort::Value tensor;
  bool zero_copy = false;
  void* mapped_ptr = nullptr;
};

class DmabufAllocator {
public:
  DmabufAllocator(Ort::Env& env, Ort::SessionOptions& session_options);
  ~DmabufAllocator();

  TensorCreateResult create_tensor_from_ref(ProcessItem* item, const TensorRef& tref);
  bool supports_zero_copy() const;

  void set_zero_copy_metric_cb(std::function<void(int)> cb);
  void set_copy_metric_cb(std::function<void(int)> cb);

private:
  Ort::Env& env_;
  Ort::SessionOptions& session_options_;
  bool zero_copy_available_;
  std::function<void(int)> zero_copy_metric_cb_;
  std::function<void(int)> copy_metric_cb_;

  // track mapped dmabufs to ensure unregister on tensor destroy
  std::mutex map_mtx_;
  std::unordered_map<const void*, void*> mapped_registry_;

  bool register_dmabuf(const TensorRef& tref, void** out_ptr);
  void unregister_dmabuf(const TensorRef& tref, void* mapped_ptr);
};