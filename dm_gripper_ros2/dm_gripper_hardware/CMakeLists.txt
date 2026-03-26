cmake_minimum_required(VERSION 3.14)
project(dm_gripper_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(dm_motor_driver REQUIRED)
find_package(dm_gripper_core REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dm_gripper_system.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  dm_motor_driver
  dm_gripper_core
)

pluginlib_export_plugin_description_file(hardware_interface dm_gripper_hardware_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(FILES dm_gripper_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hardware_interface test/test_hardware_interface.cpp)
  ament_target_dependencies(test_hardware_interface hardware_interface dm_motor_driver dm_gripper_core)
  target_link_libraries(test_hardware_interface ${PROJECT_NAME})
endif()

ament_package()
