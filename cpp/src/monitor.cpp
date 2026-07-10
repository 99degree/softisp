#include "monitor.hpp"

using namespace std::chrono_literals;

namespace softisp {

Monitor::Monitor(SessionManager& sm) : sm_(sm) {}

Monitor::~Monitor() {
    stop();
}

void Monitor::start() {
    running_.store(true);
    th_ = std::thread(&Monitor::loop, this);
}

void Monitor::stop() {
    running_.store(false);
    cv_.notify_all();
    if (th_.joinable()) th_.join();
}

void Monitor::enqueue_batch(DetachedBatch batch) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        waitingq_.push(std::move(batch));
    }
    cv_.notify_one();
}

bool Monitor::claim_token_with_retries(std::shared_ptr<ProcessItem> item, int max_retries) {
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        OwnershipToken* tok = item->token.load(std::memory_order_acquire);
        if (!tok) return true; // nothing to claim
        uint8_t expected = 0; // WORKER
        // attempt to transfer WORKER -> MONITOR (1)
        if (tok->owner_enum.compare_exchange_strong(expected, 1, std::memory_order_acq_rel, std::memory_order_acquire)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5 * attempt));
    }
    return false;
}

void Monitor::loop() {
    while (running_.load()) {
        DetachedBatch batch;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait_for(lk, 50ms, [&]{ return !waitingq_.empty() || !running_.load(); });
            if (!running_.load() && waitingq_.empty()) return;
            if (waitingq_.empty()) continue;
            batch = std::move(waitingq_.front());
            waitingq_.pop();
        }

        std::cout << "[Monitor] Processing batch origin_worker=" << batch.origin_worker_id << " items=" << batch.items.size() << "\n";

        for (auto& item : batch.items) {
            // Validate publish flag
            if (!item->env.publish_flag.load(std::memory_order_acquire)) {
                std::cout << "[Monitor] PACKET_EPOCH_MISMATCH packet=" << item->env.packet_id << "\n";
                continue;
            }

            if (!claim_token_with_retries(item)) {
                std::cout << "[Monitor] TOKEN_TRANSFER_FAIL packet=" << item->env.packet_id << "\n";
                // move to quarantine or requeue - simplified: skip for now
                continue;
            }

            OwnershipToken* tok = item->token.load(std::memory_order_acquire);
            if (!tok) {
                // no token -> return slot to SessionManager
                sm_.append_slot_to_fifo((uint32_t)item->slot_index, item->env.epoch, batch.origin_worker_id, /*reason=*/1);
                continue;
            }

            if (tok->hw_in_use_flag) {
                // RECLAIMING path (simplified)
                std::cout << "[Monitor] FRAME_RECLAIM_TRIGGERED frame=" << tok->frame_id << " packet=" << item->env.packet_id << "\n";
                // In full implementation: register and wait for FRAME_HW_DONE
                continue;
            }

            // Convert to control item and forward
            ControlItem ctrl;
            ctrl.control_type = item->env.cmd_type;
            ctrl.origin_worker_id = item->env.origin_worker_id;
            ctrl.packet_id = item->env.packet_id;
            ctrl.packet_epoch = item->env.epoch;
            ctrl.diagnostics_idx = item->env.diagnostics_idx;
            ctrl.frame_id = item->env.frame_id;
            ctrl.alloc_ticket = item->env.alloc_ticket;
            ctrl.idempotency_key = item->env.idempotency_key;
            ctrl.reason = batch.reason;

            sm_.accept_batch({ctrl});
            std::cout << "[Monitor] PROCESSITEM_CONVERTED_TO_CONTROL packet=" << item->env.packet_id << "\n";
        }
    }
}

} // namespace softisp
