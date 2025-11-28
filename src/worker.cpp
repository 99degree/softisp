#include "worker.hpp"

namespace softisp {

Worker::Worker(uint32_t id) : id_(id) {}

void Worker::enqueue_frame(std::shared_ptr<ProcessItem> item) {
    std::lock_guard<std::mutex> lk(mtx_);
    frameq_.push(item);
}

std::vector<std::shared_ptr<ProcessItem>> Worker::detach_frame_queue() {
    std::vector<std::shared_ptr<ProcessItem>> out;
    std::lock_guard<std::mutex> lk(mtx_);
    while (!frameq_.empty()) {
        out.push_back(frameq_.front());
        frameq_.pop();
    }
    return out;
}

} // namespace softisp
