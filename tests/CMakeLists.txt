cmake_minimum_required(VERSION 3.10)
project(softisp_tests)

add_executable(test_mpmc tests/test_mpmc.cpp)
add_executable(test_allocator tests/test_allocator.cpp src/session_manager.cpp src/allocator.cpp)

target_include_directories(test_mpmc PRIVATE ${CMAKE_SOURCE_DIR}/src/include)
target_include_directories(test_allocator PRIVATE ${CMAKE_SOURCE_DIR}/src/include)

# Link pthread for threading tests
find_package(Threads REQUIRED)
target_link_libraries(test_mpmc PRIVATE Threads::Threads)
target_link_libraries(test_allocator PRIVATE Threads::Threads)

add_test(NAME test_mpmc COMMAND test_mpmc)
add_test(NAME test_allocator COMMAND test_allocator)
