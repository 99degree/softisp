#pragma once
#include <atomic>
#include <cstddef>
#include <vector>
#include <thread>
#include <chrono>
#include <cassert>

// Vyukov MPMC bounded queue (header-only)
// Non-copyable, non-movable. Provides try_enqueue / try_dequeue (lock-free)
// and blocking enqueue/dequeue wrappers that spin/yield with a small backoff.
template<typename T>
class MPMCBoundedQueue {
public:
    explicit MPMCBoundedQueue(size_t capacity)
        : capacity_(capacity), buffer_(nullptr), enqueue_pos_(0), dequeue_pos_(0)
    {
        assert(capacity >= 2 && "capacity must be >= 2");
        buffer_ = static_cast<Cell*>(operator new[](sizeof(Cell) * capacity_));
        for (size_t i = 0; i < capacity_; ++i) {
            new (&buffer_[i]) Cell();
            buffer_[i].seq.store(i, std::memory_order_relaxed);
        }
    }

    ~MPMCBoundedQueue() {
        for (size_t i = 0; i < capacity_; ++i) {
            buffer_[i].~Cell();
        }
        operator delete[](buffer_);
    }

    MPMCBoundedQueue(const MPMCBoundedQueue&) = delete;
    MPMCBoundedQueue& operator=(const MPMCBoundedQueue&) = delete;

    // Non-blocking enqueue. Returns false if the queue is full.
    bool try_enqueue(const T& value) {
        Cell* cell;
        size_t pos = enqueue_pos_.load(std::memory_order_relaxed);
        for (;;) {
            cell = &buffer_[pos % capacity_];
            size_t seq = cell->seq.load(std::memory_order_acquire);
            intptr_t dif = (intptr_t)seq - (intptr_t)pos;
            if (dif == 0) {
                if (enqueue_pos_.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
                    break;
                }
            } else if (dif < 0) {
                return false; // full
            } else {
                pos = enqueue_pos_.load(std::memory_order_relaxed);
            }
        }
        cell->data = value;
        cell->seq.store(pos + 1, std::memory_order_release);
        return true;
    }

    // Non-blocking dequeue. Returns false if the queue is empty.
    bool try_dequeue(T& out) {
        Cell* cell;
        size_t pos = dequeue_pos_.load(std::memory_order_relaxed);
        for (;;) {
            cell = &buffer_[pos % capacity_];
            size_t seq = cell->seq.load(std::memory_order_acquire);
            intptr_t dif = (intptr_t)seq - (intptr_t)(pos + 1);
            if (dif == 0) {
                if (dequeue_pos_.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
                    break;
                }
            } else if (dif < 0) {
                return false; // empty
            } else {
                pos = dequeue_pos_.load(std::memory_order_relaxed);
            }
        }
        out = std::move(cell->data);
        cell->seq.store(pos + capacity_, std::memory_order_release);
        return true;
    }

    // Blocking enqueue with light backoff
    void enqueue(const T& value) {
        while (!try_enqueue(value)) {
            std::this_thread::yield();
        }
    }

    // Blocking dequeue with light backoff
    T dequeue() {
        T out;
        while (!try_dequeue(out)) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
        return out;
    }

    size_t capacity() const noexcept { return capacity_; }

private:
    struct Cell {
        std::atomic<size_t> seq;
        T data;
        Cell() : seq(0), data() {}
        ~Cell() = default;
    };

    const size_t capacity_;
    Cell* buffer_;
    std::atomic<size_t> enqueue_pos_;
    std::atomic<size_t> dequeue_pos_;
};
