cmake_minimum_required(VERSION 3.14)
project(dm_gripper_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ament 依赖
find_package(ament_cmake REQUIRED)
find_package(dm_motor_driver REQUIRED)  # 提供 MotorState / MotorCommand 类型

# ---- 核心库 ----
add_library(dm_gripper_core
    src/gripper_kinematics.cpp
    src/contact_detector.cpp
    src/gripper_state_machine.cpp
    src/impedance_controller.cpp
    src/force_position_controller.cpp
)

target_include_directories(dm_gripper_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dm_gripper_core dm_motor_driver)

# ---- 测试 ----
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_gripper_kinematics    test/test_gripper_kinematics.cpp)
    ament_add_gtest(test_contact_detector      test/test_contact_detector.cpp)
    ament_add_gtest(test_gripper_state_machine test/test_gripper_state_machine.cpp)
    ament_add_gtest(test_impedance_controller  test/test_impedance_controller.cpp)
    ament_add_gtest(test_force_position_ctrl   test/test_force_position_controller.cpp)

    foreach(t
        test_gripper_kinematics
        test_contact_detector
        test_gripper_state_machine
        test_impedance_controller
        test_force_position_ctrl
    )
        target_link_libraries(${t} dm_gripper_core)
        target_include_directories(${t} PRIVATE include)
    endforeach()
endif()

# ---- 安装 ----
install(TARGETS dm_gripper_core
    EXPORT  dm_gripper_coreTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
