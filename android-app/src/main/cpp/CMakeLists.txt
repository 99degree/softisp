cmake_minimum_required(VERSION 3.22)
project(native)

add_library(native-lib SHARED native-lib.cpp)

target_include_directories(native-lib PRIVATE
  ${CMAKE_SOURCE_DIR}/../../cam-rust/cam-isp/mnn_sys
)

target_link_libraries(native-lib android log)
