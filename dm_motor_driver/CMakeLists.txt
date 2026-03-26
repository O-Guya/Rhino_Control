cmake_minimum_required(VERSION 3.16)
project(dm_motor_driver VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# ── Library ──────────────────────────────────────────────────────────────────
add_library(dm_motor_driver
  src/dm_motor.cpp
  src/dm_protocol.cpp
  src/socketcan.cpp
)

target_include_directories(dm_motor_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(dm_motor_driver PRIVATE -Wall -Wextra -Wpedantic)

# ── Install ───────────────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS dm_motor_driver
  EXPORT dm_motor_driverTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(dm_motor_driverTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol dm_motor_driver)

  ament_add_gtest(test_socketcan_mock test/test_socketcan_mock.cpp)
  target_link_libraries(test_socketcan_mock dm_motor_driver)
endif()

ament_package()
