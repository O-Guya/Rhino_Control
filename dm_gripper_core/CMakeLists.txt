cmake_minimum_required(VERSION 3.16)
project(dm_gripper_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(dm_motor_driver REQUIRED)

# ── Library ────────────────────────────────────────────────────────────────────
add_library(dm_gripper_core
  src/gripper_kinematics.cpp
  src/contact_detector.cpp
  src/gripper_state_machine.cpp
  src/impedance_controller.cpp
  src/force_position_controller.cpp
)

target_include_directories(dm_gripper_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(dm_gripper_core
  dm_motor_driver::dm_motor_driver
)

target_compile_options(dm_gripper_core PRIVATE -Wall -Wextra -Wpedantic)

# ── Install ────────────────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS dm_gripper_core
  EXPORT dm_gripper_coreTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(dm_gripper_coreTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(dm_motor_driver)

# ── Tests ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics dm_gripper_core)

  ament_add_gtest(test_contact_detector test/test_contact_detector.cpp)
  target_link_libraries(test_contact_detector dm_gripper_core)

  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_link_libraries(test_state_machine dm_gripper_core)

  ament_add_gtest(test_controllers test/test_controllers.cpp)
  target_link_libraries(test_controllers dm_gripper_core)
endif()

ament_package()
