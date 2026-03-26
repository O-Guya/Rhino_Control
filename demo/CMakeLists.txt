cmake_minimum_required(VERSION 3.10)
project(dm_socketcan_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

file(GLOB DEMO_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

foreach(src ${DEMO_SOURCES})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    target_link_libraries(${name} PRIVATE Threads::Threads)
endforeach()