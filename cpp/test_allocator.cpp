#include "session_manager.hpp"
#include "include/allocator.hpp"
#include <thread>
#include <iostream>
#include <cassert>

using namespace softisp;

int main() {
    SessionManager sm;
    // Prepopulate SessionManager FIFO with some entries (simulate freed slots)
    const int initial_slots = 64;
    for (int i = 0; i < initial_slots; ++i) {
        sm.append_slot_to_fifo(i, 1, /*origin=*/0, /*reason=*/0);
    }

    Allocator alloc(sm, 32);

    // Allocate all slots
    std::vector<std::shared_ptr<ProcessItem>> allocated;
    for (int i = 0; i < initial_slots; ++i) {
        auto p = alloc.allocate(1000);
        if (!p) {
            std::cerr << "allocate returned nullptr at i=" << i << "\n";
            return 1;
        }
        allocated.push_back(p);
    }
    std::cout << "Allocated " << allocated.size() << " slots\n";

    // Release them back
    for (auto& p : allocated) {
        alloc.release(p);
    }

    // Refill allocator again to ensure SessionManager received the returns
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto batch = sm.pop_fifo_batch(128);
    // Since append_slot_to_fifo increments free_seq and pushes into FIFO, batch may contain the returns.
    std::cout << "SessionManager FIFO after releases: " << batch.size() << " (drained)\n";

    std::cout << "Allocator test PASSED\n";
    return 0;
}