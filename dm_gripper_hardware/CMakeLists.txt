cmake_minimum_required(VERSION 3.16)
project(dm_gripper_hardware VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(dm_motor_driver REQUIRED)
find_package(dm_gripper_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ── Plugin library ─────────────────────────────────────────────────────────────
add_library(dm_gripper_hardware SHARED
  src/dm_gripper_system.cpp
)

target_include_directories(dm_gripper_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dm_gripper_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(dm_gripper_hardware
  dm_motor_driver::dm_motor_driver
  dm_gripper_core::dm_gripper_core
)

target_compile_options(dm_gripper_hardware PRIVATE -Wall -Wextra)

# pluginlib export
pluginlib_export_plugin_description_file(hardware_interface dm_gripper_hardware.xml)

# ── Install ────────────────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS dm_gripper_hardware
  EXPORT dm_gripper_hardwareTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES dm_gripper_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(dm_gripper_hardwareTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle
                           dm_motor_driver dm_gripper_core)

# ── Tests ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_hardware_interface test/test_hardware_interface.cpp)
  ament_target_dependencies(test_hardware_interface hardware_interface)
  target_link_libraries(test_hardware_interface dm_gripper_hardware)
endif()

ament_package()
