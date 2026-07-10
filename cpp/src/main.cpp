#include "session_manager.hpp"
#include "monitor.hpp"
#include "scheduler.hpp"
#include "worker.hpp"
#include "include/softisp.hpp"
#include <thread>
#include <chrono>
#include <iostream>

using namespace softisp;

int main() {
    SessionManager sm;
    Monitor monitor(sm);
    Scheduler sched(sm, monitor);

    monitor.start();

    Worker worker(1);

    // create a few ProcessItems and enqueue to worker
    for (int i = 0; i < 5; ++i) {
        auto item = std::make_shared<ProcessItem>();
        item->env.packet_id = 1000 + i;
        item->env.cmd_type = 1;
        item->env.origin_worker_id = 1;
        item->env.epoch = 1;
        item->env.publish_flag.store(1, std::memory_order_release);

        // attach a token for some items
        if (i % 2 == 0) {
            auto* tok = new OwnershipToken();
            tok->token_id = 500 + i;
            tok->frame_id = 200 + i;
            tok->owner_enum.store(0, std::memory_order_release); // WORKER
            item->token.store(tok, std::memory_order_release);
        }
        item->slot_index = i;
        worker.enqueue_frame(item);
    }

    // Simulate scheduler handoff for this worker
    auto detached = worker.detach_frame_queue();
    sched.handoff_worker(1, detached);

    // let monitor process
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // cleanup: in a full project we'd free tokens, stop monitor gracefully
    monitor.stop();
    std::cout << "exiting\n";
    return 0;
}
