#include "session_manager.hpp"

namespace softisp {

SessionManager::SessionManager() = default;

bool SessionManager::accept_batch(const std::vector<ControlItem>& batch) {
    // Simple implementation: always accept
    // In real implementation, validate idempotency and may return BUSY
    (void)batch;
    // For observability:
    std::cout << "[SessionManager] accept_batch n=" << batch.size() << "\n";
    return true;
}

void SessionManager::append_slot_to_fifo(uint32_t slot_index, uint32_t epoch, uint32_t origin_worker_id, uint32_t reason) {
    uint64_t seq = free_seq_.fetch_add(1, std::memory_order_acq_rel) + 1;
    GlobalFIFOEntry e{slot_index, epoch, seq};
    {
        std::lock_guard<std::mutex> lk(mtx_);
        fifo_.push_back(e);
    }
    cv_.notify_one();
    std::cout << "[SessionManager] PACKET_RETURNED_TO_FIFO slot=" << slot_index << " seq=" << seq << " origin=" << origin_worker_id << " reason=" << reason << "\n";
}

std::vector<GlobalFIFOEntry> SessionManager::pop_fifo_batch(size_t max_batch) {
    std::vector<GlobalFIFOEntry> out;
    std::lock_guard<std::mutex> lk(mtx_);
    while (!fifo_.empty() && out.size() < max_batch) {
        out.push_back(fifo_.front());
        fifo_.pop_front();
    }
    return out;
}

} // namespace softisp
