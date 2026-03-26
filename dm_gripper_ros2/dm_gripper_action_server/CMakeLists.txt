cmake_minimum_required(VERSION 3.14)
project(dm_gripper_action_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(dm_gripper_core REQUIRED)

add_executable(gripper_action_server
  src/gripper_action_server.cpp
)

target_include_directories(gripper_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gripper_action_server
  rclcpp
  rclcpp_action
  control_msgs
  dm_gripper_core
)

install(TARGETS gripper_action_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_action_interface test/test_action_interface.cpp)
  ament_target_dependencies(test_action_interface rclcpp rclcpp_action control_msgs dm_gripper_core)
endif()

ament_package()
