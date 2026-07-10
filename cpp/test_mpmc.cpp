#include "mpmc_bounded_queue.hpp"
#include <thread>
#include <vector>
#include <iostream>
#include <atomic>
#include <cassert>

int main() {
    const size_t capacity = 1024;
    MPMCBoundedQueue<int> q(capacity);

    const int producers = 4;
    const int consumers = 4;
    const int items_per_producer = 10000;

    std::atomic<int> produced{0};
    std::atomic<int> consumed{0};

    std::vector<std::thread> prod_ts;
    for (int p = 0; p < producers; ++p) {
        prod_ts.emplace_back([&](){
            for (int i = 0; i < items_per_producer; ++i) {
                q.enqueue(p * items_per_producer + i);
                produced.fetch_add(1, std::memory_order_relaxed);
            }
        });
    }

    std::vector<std::thread> cons_ts;
    for (int c = 0; c < consumers; ++c) {
        cons_ts.emplace_back([&](){
            int local_count = 0;
            while (consumed.load(std::memory_order_relaxed) < producers * items_per_producer) {
                int v;
                if (q.try_dequeue(v)) {
                    consumed.fetch_add(1, std::memory_order_relaxed);
                    ++local_count;
                } else {
                    std::this_thread::yield();
                }
            }
            // std::cout << "consumer done, got " << local_count << "\n";
        });
    }

    for (auto& t : prod_ts) t.join();
    for (auto& t : cons_ts) t.join();

    std::cout << "produced=" << produced.load() << " consumed=" << consumed.load() << "\n";
    assert(produced.load() == consumed.load());
    std::cout << "MPMC test PASSED\n";
    return 0;
}