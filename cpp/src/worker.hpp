#pragma once
#include "include/softisp.hpp"
#include <queue>
#include <mutex>
#include <vector>
#include <memory>

namespace softisp {

class Worker {
public:
    Worker(uint32_t id);
    // Enqueue a ProcessItem into frame queue
    void enqueue_frame(std::shared_ptr<ProcessItem> item);
    // Detach frame queue for handoff (atomic detach simplified)
    std::vector<std::shared_ptr<ProcessItem>> detach_frame_queue();

private:
    uint32_t id_;
    std::mutex mtx_;
    std::queue<std::shared_ptr<ProcessItem>> frameq_;
};

} // namespace softisp
