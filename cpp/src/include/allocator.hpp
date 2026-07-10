#pragma once
#include "softisp.hpp"
#include "session_manager.hpp"
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <deque>

namespace softisp {

class Allocator {
public:
    // pool_size: number of preallocated slots maintained by allocator
    Allocator(SessionManager& sm, size_t pool_size = 128);

    // Acquire a ProcessItem slot (blocking until available or timeout=-1 for indefinite)
    std::shared_ptr<ProcessItem> allocate(int timeout_ms = -1);

    // Release a slot back to SessionManager (returns to authoritative FIFO)
    void release(std::shared_ptr<ProcessItem> item);

    // For tests: get per-core cache size
    size_t pool_size() const { return pool_size_; }

private:
    void refill_from_session_manager();

    SessionManager& sm_;
    size_t pool_size_;

    // simple global free list used by allocate/release (protected by mutex for simplicity)
    std::mutex mtx_;
    std::deque<std::shared_ptr<ProcessItem>> free_list_;
};

} // namespace softisp
