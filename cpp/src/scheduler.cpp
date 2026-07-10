#include "scheduler.hpp"

namespace softisp {

Scheduler::Scheduler(SessionManager& sm, Monitor& monitor) : sm_(sm), monitor_(monitor) {}

void Scheduler::handoff_worker(uint32_t worker_id, std::vector<std::shared_ptr<ProcessItem>> detached_batch) {
    if (detached_batch.empty()) return;

    // Drain small control items first (not modeled here)
    // Attempt fast-forward
    if (try_fast_forward_batch(worker_id, detached_batch)) {
        std::cout << "[Scheduler] HANDOFF_FAST_FORWARDED worker=" << worker_id << " count=" << detached_batch.size() << "\n";
        return;
    }

    // Fallback to Monitor
    DetachedBatch db;
    db.origin_worker_id = worker_id;
    db.reason = 2; // KILL_HANDOFF
    db.items = std::move(detached_batch);
    monitor_.enqueue_batch(std::move(db));
    std::cout << "[Scheduler] PACKET_MOVED_TO_MONITOR worker=" << worker_id << "\n";
}

bool Scheduler::try_fast_forward_batch(uint32_t worker_id, const std::vector<std::shared_ptr<ProcessItem>>& batch) {
    std::vector<OwnershipToken*> claimed;
    claimed.reserve(batch.size());

    for (auto& item : batch) {
        OwnershipToken* tok = item->token.load(std::memory_order_acquire);
        if (!tok) continue;
        uint8_t expected = 0; // WORKER
        if (!tok->owner_enum.compare_exchange_strong(expected, 2, std::memory_order_acq_rel, std::memory_order_acquire)) {
            // rollback claimed
            for (auto* t : claimed) {
                t->owner_enum.store(0, std::memory_order_release);
            }
            return false;
        }
        claimed.push_back(tok);
    }

    // convert to control items
    std::vector<ControlItem> ctrls;
    ctrls.reserve(batch.size());
    for (auto& item : batch) {
        ControlItem c;
        c.control_type = item->env.cmd_type;
        c.origin_worker_id = item->env.origin_worker_id;
        c.packet_id = item->env.packet_id;
        c.packet_epoch = item->env.epoch;
        c.diagnostics_idx = item->env.diagnostics_idx;
        c.frame_id = item->env.frame_id;
        c.alloc_ticket = item->env.alloc_ticket;
        c.idempotency_key = item->env.idempotency_key;
        c.reason = 3; // KILL_FAST_FORWARD
        ctrls.push_back(c);
    }

    bool ok = sm_.accept_batch(ctrls);
    if (!ok) {
        // rollback
        for (auto* t : claimed) t->owner_enum.store(0, std::memory_order_release);
    }
    return ok;
}

} // namespace softisp
