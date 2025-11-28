#include "onnx_inferencer.h"
#include "onnx_dmabuf_allocator.h"
#include <onnxruntime_cxx_api.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <iostream>
#include <exception>

struct ONNXInferencer::Impl {
  std::unique_ptr<ONNXModelHandle> model;
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "onnx"};
  std::unique_ptr<Ort::Session> session;
  Ort::SessionOptions session_options;
  size_t max_batch;
  int max_latency_ms;

  std::unique_ptr<DmabufAllocator> dmabuf_allocator;

  std::mutex mtx;
  std::condition_variable cv;
  std::queue<std::pair<ProcessItem*, InferCallback>> q;
  bool stop = false;
  std::thread worker;

  Impl(std::unique_ptr<ONNXModelHandle> m, size_t mb, int ml)
    : model(std::move(m)), max_batch(mb), max_latency_ms(ml) {
    session_options.SetIntraOpNumThreads(1);
    if (model && !model->model_path.empty()) {
      try {
        session.reset(new Ort::Session(env, model->model_path.c_str(), session_options));
      } catch (...) {
        // ignore for stub
      }
    }
    dmabuf_allocator.reset(new DmabufAllocator(env, session_options));
    worker = std::thread([this]{ this->run_loop(); });
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lk(mtx);
      stop = true;
      cv.notify_all();
    }
    if (worker.joinable()) worker.join();
    dmabuf_allocator.reset();
  }

  void run_loop() {
    while (true) {
      std::unique_lock<std::mutex> lk(mtx);
      if (q.empty() && !stop) cv.wait_for(lk, std::chrono::milliseconds(max_latency_ms));
      if (stop && q.empty()) break;

      std::vector<std::pair<ProcessItem*, InferCallback>> batch;
      while (!q.empty() && batch.size() < max_batch) {
        batch.push_back(q.front()); q.pop();
      }
      lk.unlock();

      for (auto &p : batch) {
        ProcessItem* item = p.first;
        InferCallback cb = p.second;
        bool ok = false;
        std::vector<float> outputs;
        try {
          // In a real implementation, we would build Ort::Value inputs possibly using dmabuf_allocator
          outputs = {0.0f}; // dummy
          ok = true;
        } catch (...) {
          ok = false;
        }
        cb(ok, outputs);
      }
    }
  }
};

ONNXInferencer::ONNXInferencer(std::unique_ptr<ONNXModelHandle> model, size_t max_batch, int max_latency_ms)
  : impl_(new Impl(std::move(model), max_batch, max_latency_ms)) {}

ONNXInferencer::~ONNXInferencer() = default;

void ONNXInferencer::enqueue_inference(ProcessItem* item, InferCallback cb) {
  std::lock_guard<std::mutex> lk(impl_->mtx);
  impl_->q.emplace(item, cb);
  impl_->cv.notify_one();
}

void ONNXInferencer::reload_model(std::unique_ptr<ONNXModelHandle> new_model) {
  std::lock_guard<std::mutex> lk(impl_->mtx);
  impl_->model = std::move(new_model);
  // swap sessions in full implementation
}