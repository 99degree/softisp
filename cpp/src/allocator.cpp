#include "include/allocator.hpp"
#include <chrono>
#include <iostream>

namespace softisp {

Allocator::Allocator(SessionManager& sm, size_t pool_size)
    : sm_(sm), pool_size_(pool_size)
{
    // initialize empty free_list_; caller should append initial slots via SessionManager in tests
}

std::shared_ptr<ProcessItem> Allocator::allocate(int timeout_ms) {
    using namespace std::chrono;
    auto deadline = steady_clock::now() + milliseconds(timeout_ms < 0 ? 1000000000 : timeout_ms);

    while (true) {
        {
            std::lock_guard<std::mutex> lg(mtx_);
            if (!free_list_.empty()) {
                auto it = free_list_.front();
                free_list_.pop_front();
                return it;
            }
        }

        // Try to refill from SessionManager FIFO
        refill_from_session_manager();

        // wait a bit if still empty
        if (timeout_ms >= 0 && steady_clock::now() > deadline) return nullptr;
        std::this_thread::sleep_for( std::chrono::milliseconds(1) );
    }
}

void Allocator::release(std::shared_ptr<ProcessItem> item) {
    // For correctness, release means return to SessionManager's FIFO with epoch and metadata.
    // Here we forward the slot back to SessionManager.
    sm_.append_slot_to_fifo((uint32_t)item->slot_index, item->env.epoch, item->env.origin_worker_id, /*reason=*/0);
}

void Allocator::refill_from_session_manager() {
    // Drain a small batch from SessionManager FIFO and convert entries into ProcessItems
    auto batch = sm_.pop_fifo_batch(32);
    if (batch.empty()) return;

    std::lock_guard<std::mutex> lg(mtx_);
    for (auto& e : batch) {
        auto p = std::make_shared<ProcessItem>();
        p->slot_index = e.slot_index;
        p->env.epoch = e.epoch;
        p->env.publish_flag.store(0, std::memory_order_release);
        free_list_.push_back(std::move(p));
    }
    std::cout << "[Allocator] Refilled " << batch.size() << " slots\n";
}

} // namespace softisp
