cmake_minimum_required(VERSION 3.16)
project(dm_gripper_action_server VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(dm_motor_driver REQUIRED)
find_package(dm_gripper_core REQUIRED)

# ── Executable ─────────────────────────────────────────────────────────────────
add_executable(gripper_action_server
  src/gripper_action_server.cpp
)

target_include_directories(gripper_action_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gripper_action_server
  rclcpp
  rclcpp_action
  control_msgs
)

target_link_libraries(gripper_action_server
  dm_motor_driver::dm_motor_driver
  dm_gripper_core::dm_gripper_core
)

target_compile_options(gripper_action_server PRIVATE -Wall -Wextra)

# ── Install ────────────────────────────────────────────────────────────────────
install(
  TARGETS gripper_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ── Tests ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_action_interface test/test_action_interface.cpp)
  ament_target_dependencies(test_action_interface rclcpp rclcpp_action control_msgs)
endif()

ament_package()
