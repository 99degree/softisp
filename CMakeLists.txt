cmake_minimum_required(VERSION 3.10)

# Set project name
project(softisp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add source to this project
add_executable(softisp main.cpp src/session_manager.cpp src/monitor.cpp src/scheduler.cpp src/worker.cpp src/allocator.cpp)

# Include directories
include_directories(src/include)

# Add tests
enable_testing()
add_subdirectory(tests)
