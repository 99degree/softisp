#pragma once
#include "../include/control_item.h"
#include "../include/global_fifo.h"
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <memory>
#include "lf_queue.h"
#include <thread>

// Forward-declare a ControlBatch type for queueing
using ControlBatch = std::vector<ControlItem>;

struct FrameTableEntry {
  uint64_t frame_id;
  uint64_t alloc_ticket;
  bool hw_in_use;
  uint64_t slot_index;
  uint32_t epoch;
  uint64_t last_update_ts;
};

class SessionManager {
public:
  SessionManager(size_t fifo_capacity = 4096);
  ~SessionManager();

  // Accept control items (idempotent). Return false on transient BUSY.
  bool accept_batch(const ControlBatch& batch);

  // Append freed slot to authoritative FIFO
  bool append_slot_to_fifo(uint32_t slot_index, uint32_t epoch, uint32_t origin_worker_id, uint32_t reason);

  // Hardware completion notification
  void on_frame_hw_done(uint64_t frame_id, uint64_t alloc_ticket = 0);

  // Allocator API to pop a free slot
  bool pop_free_slot(GlobalFIFOEntry &out);

  uint64_t last_free_seq() const { return free_seq_.load(std::memory_order_acquire); }

private:
  std::atomic<uint64_t> free_seq_;

  // Lock-free queue for control batches (producer: Monitor, consumer: session.apply loop)
  std::unique_ptr<LFQueue<ControlBatch>> control_queue_;

  // Frame table
  std::mutex frame_table_mtx_;
  std::unordered_map<uint64_t, FrameTableEntry> frame_table_;

  // Idempotency store
  std::mutex idempotency_mtx_;
  std::unordered_map<uint64_t, uint64_t> idempotency_store_;

  // Local FIFO of freed slots (authoritative)
  std::mutex fifo_mtx_;
  std::vector<GlobalFIFOEntry> fifo_buffer_;
  size_t fifo_head_;

  void enqueue_fifo_entry(const GlobalFIFOEntry &entry);
  bool validate_control_item(const ControlItem &c);

  // Background apply loop (consumer of control_queue_)
  void apply_loop();
  std::atomic<bool> running_;
  std::thread apply_thread_;
};