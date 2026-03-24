cmake_minimum_required(VERSION 3.16)
project(dm_socketcan_demo)
set(CMAKE_CXX_STANDARD 17)

find_package(Threads REQUIRED)

add_executable(dm_socketcan_demo src/dm_socketcan_demo.cpp)
target_compile_options(dm_socketcan_demo PRIVATE -Wall)
target_link_libraries(dm_socketcan_demo PRIVATE Threads::Threads)
# 无额外依赖：只用 Linux 内核头文件 linux/can.h
