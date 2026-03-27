cmake_minimum_required(VERSION 3.14)
project(dm_motor_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dm_protocol.cpp
  src/socketcan.cpp
  src/dm_motor.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} pthread)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# ──────────────────────────────────────────────────────────────
# 实验工具（Task 1 Phase 1）
# ──────────────────────────────────────────────────────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/tau_logger.cpp")
  add_executable(tau_logger tools/tau_logger.cpp)
  target_include_directories(tau_logger PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(tau_logger ${PROJECT_NAME} pthread)
  install(TARGETS tau_logger DESTINATION lib/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/smoke_test.cpp")
  add_executable(smoke_test tools/smoke_test.cpp)
  target_include_directories(smoke_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(smoke_test ${PROJECT_NAME} pthread)
  install(TARGETS smoke_test DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME})
  ament_add_gtest(test_socketcan_mock test/test_socketcan_mock.cpp)
  target_link_libraries(test_socketcan_mock ${PROJECT_NAME})
endif()

ament_package()