cmake_minimum_required(VERSION 3.22.1)

project(softisp_jni LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(cam_isp SHARED
    native-lib.cpp
)

target_include_directories(cam_isp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/../../../../cam-rust/cam-isp/mnn_sys
)

find_library(ANDROID_LOG_LIB log)
target_link_libraries(cam_isp ${ANDROID_LOG_LIB} android)
