#pragma once
#include "include/softisp.hpp"
#include "session_manager.hpp"
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <condition_variable>
#include <iostream>

namespace softisp {

struct DetachedBatch {
    uint32_t origin_worker_id;
    uint32_t reason;
    std::vector<std::shared_ptr<ProcessItem>> items;
};

class Monitor {
public:
    Monitor(SessionManager& sm);
    ~Monitor();

    void enqueue_batch(DetachedBatch batch);
    void start();
    void stop();

private:
    void loop();
    bool claim_token_with_retries(std::shared_ptr<ProcessItem> item, int max_retries = 3);

    SessionManager& sm_;
    std::queue<DetachedBatch> waitingq_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::thread th_;
    std::atomic<bool> running_{false};
};

} // namespace softisp
