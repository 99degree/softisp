SoftISP - C++ scaffold (GPL-3.0)

This repository contains an initial scaffold implementing the SoftISP design:
- unified ProcessItem model
- OwnershipToken semantics
- Scheduler, Monitor, SessionManager skeletons

Build (requires CMake):
  mkdir build && cd build
  cmake ..
  cmake --build .

Run:
  ./softisp

This initial binary runs a small simulation of a worker and a scheduler handoff path.
