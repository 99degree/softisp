#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>
#include <string>

namespace softisp {

// Envelope: canonical metadata
struct Envelope {
    uint64_t packet_id = 0;
    uint32_t cmd_type = 0;
    uint32_t origin_worker_id = 0;
    uint64_t frame_id = 0;
    uint64_t alloc_ticket = 0;
    uint64_t idempotency_key = 0;
    uint32_t diagnostics_idx = 0;
    uint32_t epoch = 0;
    std::atomic<uint8_t> publish_flag{0};
};

// OwnershipToken: small atomic token
struct OwnershipToken {
    uint64_t token_id{0};
    uint64_t frame_id{0};
    uint8_t pool_id{0};
    uint8_t hw_in_use_flag{0};
    uint8_t reserved[6]{0};
    std::atomic<uint32_t> refcnt{0};
    std::atomic<uint8_t> owner_enum{0}; // 0=WORKER,1=MONITOR,2=SESSIONMANAGER
    OwnershipToken() = default;
};

// ProcessItem: unified item used across queues
struct ProcessItem {
    Envelope env;
    std::atomic<OwnershipToken*> token{nullptr}; // nullable
    void* worker_transient = nullptr;
    std::atomic<uint32_t> local_refcnt{0};
    std::atomic<uint8_t> state{0};
    uint64_t slot_index{0};
    uint64_t timestamps[2]{0,0};
    ProcessItem() = default;
};

struct ControlItem {
    uint32_t control_type{0};
    uint32_t origin_worker_id{0};
    uint64_t packet_id{0};
    uint32_t packet_epoch{0};
    uint32_t diagnostics_idx{0};
    uint64_t frame_id{0};
    uint64_t alloc_ticket{0};
    uint64_t idempotency_key{0};
    uint32_t reason{0};
};

} // namespace softisp
