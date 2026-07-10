#pragma once
#include <atomic>
#include <cstddef>
#include <vector>
#include <cassert>

// Bounded lock-free MPMC queue based on circular buffer with sequence counters.
// Capacity must be power of two.
template<typename T>
class LFQueue {
public:
  explicit LFQueue(size_t capacity_pow2);
  ~LFQueue();

  bool push(const T& item);
  bool pop(T& out);
  size_t capacity() const { return capacity_; }

private:
  struct Node {
    std::atomic<size_t> seq;
    T data;
  };

  const size_t capacity_;
  const size_t mask_;
  std::vector<Node> buffer_;
  std::atomic<size_t> tail_;
  std::atomic<size_t> head_;
};

template<typename T>
LFQueue<T>::LFQueue(size_t capacity_pow2)
  : capacity_(capacity_pow2), mask_(capacity_pow2 - 1),
    buffer_(capacity_pow2), tail_(0), head_(0) {
  assert((capacity_pow2 & (capacity_pow2 - 1)) == 0 && "capacity must be power of two");
  for (size_t i = 0; i < capacity_; ++i) buffer_[i].seq.store(i, std::memory_order_relaxed);
}

template<typename T>
LFQueue<T>::~LFQueue() {}

template<typename T>
bool LFQueue<T>::push(const T& item) {
  Node* node;
  size_t pos = tail_.load(std::memory_order_relaxed);
  while (true) {
    node = &buffer_[pos & mask_];
    size_t seq = node->seq.load(std::memory_order_acquire);
    intptr_t dif = (intptr_t)seq - (intptr_t)pos;
    if (dif == 0) {
      if (tail_.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) break;
    } else if (dif < 0) {
      return false; // full
    } else {
      pos = tail_.load(std::memory_order_relaxed);
    }
  }
  node->data = item;
  node->seq.store(pos + 1, std::memory_order_release);
  return true;
}

template<typename T>
bool LFQueue<T>::pop(T& out) {
  Node* node;
  size_t pos = head_.load(std::memory_order_relaxed);
  while (true) {
    node = &buffer_[pos & mask_];
    size_t seq = node->seq.load(std::memory_order_acquire);
    intptr_t dif = (intptr_t)seq - (intptr_t)(pos + 1);
    if (dif == 0) {
      if (head_.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) break;
    } else if (dif < 0) {
      return false; // empty
    } else {
      pos = head_.load(std::memory_order_relaxed);
    }
  }
  out = node->data;
  node->seq.store(pos + capacity_, std::memory_order_release);
  return true;
}