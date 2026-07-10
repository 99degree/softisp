#include "onnx_dmabuf_allocator.h"
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <memory>

DmabufAllocator::DmabufAllocator(Ort::Env& env, Ort::SessionOptions& session_options)
  : env_(env), session_options_(session_options), zero_copy_available_(true) {}

DmabufAllocator::~DmabufAllocator() {
  // ensure any mappings are released (best-effort)
  std::lock_guard<std::mutex> lk(map_mtx_);
  for (auto &it : mapped_registry_) {
    if (it.second) munmap(it.second, 4096);
  }
  mapped_registry_.clear();
}

bool DmabufAllocator::supports_zero_copy() const { return zero_copy_available_; }

void DmabufAllocator::set_zero_copy_metric_cb(std::function<void(int)> cb) { zero_copy_metric_cb_ = cb; }
void DmabufAllocator::set_copy_metric_cb(std::function<void(int)> cb) { copy_metric_cb_ = cb; }

bool DmabufAllocator::register_dmabuf(const TensorRef& tref, void** out_ptr) {
  if (tref.fd < 0 || !tref.is_dmabuf) return false;
  // map only the required size; align to page
  size_t len = tref.byte_length();
  size_t page_size = static_cast<size_t>(sysconf(_SC_PAGESIZE));
  size_t map_len = ((len + page_size - 1) / page_size) * page_size;
  void* p = mmap(nullptr, map_len, PROT_READ | PROT_WRITE, MAP_SHARED, tref.fd, 0);
  if (p == MAP_FAILED) return false;
  *out_ptr = p;
  {
    std::lock_guard<std::mutex> lk(map_mtx_);
    mapped_registry_.emplace(p, p);
  }
  return true;
}

void DmabufAllocator::unregister_dmabuf(const TensorRef& tref, void* mapped_ptr) {
  if (!mapped_ptr) return;
  std::lock_guard<std::mutex> lk(map_mtx_);
  auto it = mapped_registry_.find(mapped_ptr);
  if (it != mapped_registry_.end()) {
    // best effort: unmap full page range
    munmap(it->first, 4096);
    mapped_registry_.erase(it);
  }
}

TensorCreateResult DmabufAllocator::create_tensor_from_ref(ProcessItem* item, const TensorRef& tref) {
  TensorCreateResult res{};
  if (tref.is_dmabuf && supports_zero_copy()) {
    void* mapped = nullptr;
    if (!register_dmabuf(tref, &mapped)) {
      if (copy_metric_cb_) copy_metric_cb_(1);
      return res;
    }

    try {
      Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      size_t elem_count = 1;
      for (auto d : tref.shape) elem_count *= (size_t)d;
      if (tref.dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        res.tensor = Ort::Value::CreateTensor<float>(mem_info, static_cast<float*>(mapped), elem_count, tref.shape.data(), tref.shape.size());
      } else {
        // fallback: create as byte blob
        res.tensor = Ort::Value::CreateTensor<uint8_t>(mem_info, static_cast<uint8_t*>(mapped), elem_count, tref.shape.data(), tref.shape.size());
      }
      res.zero_copy = true;
      res.mapped_ptr = mapped;
      if (zero_copy_metric_cb_) zero_copy_metric_cb_(1);
      return res;
    } catch (const std::exception& e) {
      unregister_dmabuf(tref, mapped);
      if (copy_metric_cb_) copy_metric_cb_(1);
      return res;
    }
  }

  // fallback copy path
  if (copy_metric_cb_) copy_metric_cb_(1);
  Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  size_t elem_count = 1;
  for (auto d : tref.shape) elem_count *= (size_t)d;
  if (tref.ptr == nullptr) {
    // allocate zeroed vector
    std::vector<float> tmp(elem_count, 0.0f);
    res.tensor = Ort::Value::CreateTensor<float>(mem_info, tmp.data(), elem_count, tref.shape.data(), tref.shape.size());
  } else {
    // copy the raw memory into a temporary vector (lifetime must outlive tensor creation)
    std::vector<float> tmp(elem_count);
    memcpy(tmp.data(), tref.ptr, elem_count * sizeof(float));
    res.tensor = Ort::Value::CreateTensor<float>(mem_info, tmp.data(), elem_count, tref.shape.data(), tref.shape.size());
  }
  res.zero_copy = false;
  return res;
}