#pragma once
#include "include/softisp.hpp"
#include <deque>
#include <mutex>
#include <condition_variable>
#include <iostream>

namespace softisp {

struct GlobalFIFOEntry {
    uint32_t slot_index;
    uint32_t epoch;
    uint64_t free_seq;
};

class SessionManager {
public:
    SessionManager();

    // Accept small control batch. Return true if accepted; false if busy.
    bool accept_batch(const std::vector<ControlItem>& batch);

    // Append a freed slot into the authoritative FIFO
    void append_slot_to_fifo(uint32_t slot_index, uint32_t epoch, uint32_t origin_worker_id, uint32_t reason);

    // Pop a batch from FIFO (allocator simulation)
    std::vector<GlobalFIFOEntry> pop_fifo_batch(size_t max_batch);

private:
    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<GlobalFIFOEntry> fifo_;
    std::atomic<uint64_t> free_seq_{0};
};

} // namespace softisp
