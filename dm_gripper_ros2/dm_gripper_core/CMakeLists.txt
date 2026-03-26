cmake_minimum_required(VERSION 3.14)
project(dm_gripper_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(dm_motor_driver REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gripper_kinematics.cpp
  src/contact_detector.cpp
  src/gripper_state_machine.cpp
  src/impedance_controller.cpp
  src/force_position_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} dm_motor_driver)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(dm_motor_driver)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME})
  ament_add_gtest(test_contact_detector test/test_contact_detector.cpp)
  target_link_libraries(test_contact_detector ${PROJECT_NAME})
  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_link_libraries(test_state_machine ${PROJECT_NAME})
  ament_add_gtest(test_controllers test/test_controllers.cpp)
  target_link_libraries(test_controllers ${PROJECT_NAME})
endif()

ament_package()
