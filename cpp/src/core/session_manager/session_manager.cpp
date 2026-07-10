#include "session_manager.h"
#include <chrono>
#include <iostream>
#include <thread>

SessionManager::SessionManager(size_t fifo_capacity)
  : free_seq_(0),
    control_queue_(new LFQueue<ControlBatch>(1024)),
    fifo_head_(0),
    running_(true),
    apply_thread_(&SessionManager::apply_loop, this) {
  (void)fifo_capacity;
}

SessionManager::~SessionManager() {
  running_.store(false);
  if (apply_thread_.joinable()) apply_thread_.join();
}

// accept_batch now enqueues into control_queue_, returns false if queue is full
bool SessionManager::accept_batch(const ControlBatch& batch) {
  if (batch.empty()) return true;
  if (!control_queue_->push(batch)) {
    return false; // BUSY
  }
  return true;
}

bool SessionManager::append_slot_to_fifo(uint32_t slot_index, uint32_t epoch, uint32_t origin_worker_id, uint32_t reason) {
  GlobalFIFOEntry e{slot_index, epoch, free_seq_.fetch_add(1, std::memory_order_acq_rel) + 1};
  enqueue_fifo_entry(e);
  return true;
}

void SessionManager::on_frame_hw_done(uint64_t frame_id, uint64_t alloc_ticket) {
  std::lock_guard<std::mutex> f(frame_table_mtx_);
  auto it = frame_table_.find(frame_id);
  if (it != frame_table_.end()) {
    it->second.hw_in_use = false;
    GlobalFIFOEntry e;
    e.slot_index = static_cast<uint32_t>(it->second.slot_index);
    e.epoch = it->second.epoch;
    e.free_seq = free_seq_.fetch_add(1, std::memory_order_acq_rel) + 1;
    enqueue_fifo_entry(e);
  } else {
    // Unknown frame - nothing to do for now
  }
}

bool SessionManager::pop_free_slot(GlobalFIFOEntry &out) {
  std::lock_guard<std::mutex> lk(fifo_mtx_);
  if (fifo_head_ >= fifo_buffer_.size()) return false;
  out = fifo_buffer_[fifo_head_++];
  if (fifo_head_ > 1024 && fifo_head_ * 2 > fifo_buffer_.size()) {
    fifo_buffer_.erase(fifo_buffer_.begin(), fifo_buffer_.begin() + fifo_head_);
    fifo_head_ = 0;
  }
  return true;
}

void SessionManager::enqueue_fifo_entry(const GlobalFIFOEntry &entry) {
  std::lock_guard<std::mutex> lk(fifo_mtx_);
  fifo_buffer_.push_back(entry);
  // In a full product, this would be persisted/replicated; here we just append.
  std::cerr << "[session_manager] appended slot=" << entry.slot_index << " free_seq=" << entry.free_seq << "\n";
}

bool SessionManager::validate_control_item(const ControlItem &c) {
  // Basic validation for stub
  return true;
}

void SessionManager::apply_loop() {
  while (running_.load()) {
    ControlBatch batch;
    if (control_queue_->pop(batch)) {
      // Apply each control item
      for (const auto &c : batch) {
        // Simple handling: RETURN_TO_POOL or RELEASE_FRAME produce slot entries
        if (c.control_type == RETURN_TO_POOL) {
          GlobalFIFOEntry e;
          e.slot_index = static_cast<uint32_t>(c.alloc_ticket);
          e.epoch = c.packet_epoch;
          e.free_seq = free_seq_.fetch_add(1, std::memory_order_acq_rel) + 1;
          enqueue_fifo_entry(e);
        } else if (c.control_type == RELEASE_FRAME) {
          std::lock_guard<std::mutex> f(frame_table_mtx_);
          FrameTableEntry fe;
          fe.frame_id = c.frame_id;
          fe.alloc_ticket = c.alloc_ticket;
          fe.hw_in_use = false;
          fe.slot_index = static_cast<uint32_t>(c.alloc_ticket);
          fe.epoch = c.packet_epoch;
          fe.last_update_ts = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now().time_since_epoch()).count());
          frame_table_[c.frame_id] = fe;
          // Immediately append freed slot for RELEASE_FRAME in this simplified model
          GlobalFIFOEntry e{static_cast<uint32_t>(fe.slot_index), fe.epoch, free_seq_.fetch_add(1, std::memory_order_acq_rel) + 1};
          enqueue_fifo_entry(e);
        } else {
          // other control types ignored for stub
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}