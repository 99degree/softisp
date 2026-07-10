#pragma once
#include "include/softisp.hpp"
#include "monitor.hpp"
#include "session_manager.hpp"
#include <vector>
#include <iostream>

namespace softisp {

class Scheduler {
public:
    Scheduler(SessionManager& sm, Monitor& monitor);
    // Handoff a detached batch from a worker
    void handoff_worker(uint32_t worker_id, std::vector<std::shared_ptr<ProcessItem>> detached_batch);

private:
    bool try_fast_forward_batch(uint32_t worker_id, const std::vector<std::shared_ptr<ProcessItem>>& batch);
    SessionManager& sm_;
    Monitor& monitor_;
};

} // namespace softisp
