cmake_minimum_required(VERSION 3.16)
project(dm_socketcan_demo)
set(CMAKE_CXX_STANDARD 17)

add_executable(dm_socketcan_demo src/dm_socketcan_demo.cpp)
target_compile_options(dm_socketcan_demo PRIVATE -Wall)
# 无额外依赖：只用 Linux 内核头文件 linux/can.h
