# SoftISP — Project Tracking & Implementation Plan

This document sits with the code and tracks the implementation plan, milestones, tests, and telemetry derived from the architecture and design specification. License: GPL-3.0 (see LICENSE file).

## Project goals (short)
- Implement a resilient, observable ISP pipeline in C++ with:
  - unified ProcessItem format and Envelope
  - OwnershipToken semantics (atomic owner transitions)
  - Scheduler fast-forward handoff and Monitor fallback
  - SessionManager authoritative global FIFO for slot reuse
  - Observability and operational knobs

## Repo layout (initial)
- CMakeLists.txt
- LICENSE
- README.md
- TRACKING.md
- src/
  - include/softisp.hpp
  - main.cpp
  - session_manager.{cpp,hpp}
  - monitor.{cpp,hpp}
  - scheduler.{cpp,hpp}
  - worker.{cpp,hpp}

## Initial milestones (first sprint)
1. Drop-in scaffold (this commit)
   - basic types, skeleton modules, build
2. Implement thread-safe FIFO and simple allocator
3. Implement Monitor waitingq handler and token claim retries
4. Implement Scheduler detach + fast-forward attempt + fallback to Monitor
5. Add unit tests for token claim and epoch/publish fence
6. Add telemetry hooks and metrics counters
7. Stress tests and reclaim path simulation

## Tests (priority)
- Token transfer race stress test
- Epoch/publish fence fuzzing
- Zero-copy handoff pointer equality test
- FIFO fidelity test (strict-FIFO vs per-core caches)
- Reclaim correctness with simulated FRAME_HW_DONE

## Operational knobs (configurable)
- max_cmds_before_frame
- DRAIN_GRACE_PERIOD
- CANCEL_RETRY_COUNT
- DROP_THRESHOLD
- ESCALATION_TIMEOUT

## Observability & events
- waitingq_depth, token_transfer_fail_count
- HANDOFF_FAST_FORWARDED, PACKET_MOVED_TO_MONITOR
- PROCESSITEM_CONVERTED_TO_CONTROL, PACKET_RETURNED_TO_FIFO
- FRAME_RECLAIM_TRIGGERED, FRAME_RECLAIMED

## Next steps to get to a runnable canary
- Implement allocator and per-core caches
- Flesh out SessionManager to append to FIFO with free_seq
- Implement idempotency checks in SessionManager
- Integrate a simple telemetry emitter (prometheus or text metrics)
- Add CI that builds, runs unit tests, and runs token race stress test
