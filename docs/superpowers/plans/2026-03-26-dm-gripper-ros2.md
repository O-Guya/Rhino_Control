# DM Gripper ROS 2 Implementation Plan
**Date:** 2026-03-26

## Goal

基于达妙 DM4310 电机 + 丝杆传动，实现带阻抗控制和力位混合控制的 ROS 2 夹爪驱动包。

## Architecture

三层解耦设计：

```
dm_motor_driver       (零 ROS 依赖的纯 C++ CAN 通信库)
        ↓
dm_gripper_core       (夹爪运动学 + 控制状态机，零 ROS 依赖)
        ↓
dm_gripper_hardware / dm_gripper_action_server   (ROS 2 接口层)
```

控制模式通过策略模式在 core 层切换，硬件层不感知模式差异。

**Tech Stack:** C++17, Linux SocketCAN, ROS 2 Humble, ros2_control, control_msgs/action/GripperCommand

## Background Constraints

| Item | Status |
|------|--------|
| 传动 | 丝杆（q→线性位移）+ 连杆（线性位移→开口宽度） |
| 连杆 | 尚未到货，运动学用占位实现 |
| 传感器 | 仅电机自身 τ_feedback（12-bit 电流估算） |
| 接触检测 | 「位置卡死 + τ 上升」双重判据 |
| 当前硬件 | 裸电机可用 (can0, SocketCAN) |

## File Structure

```
dm_gripper_ros2/
├── dm_motor_driver/                    # 纯 C++ 库，零 ROS 依赖
│   ├── CMakeLists.txt
│   ├── include/dm_motor_driver/
│   │   ├── dm_protocol.hpp             # MIT 帧打包/解包，寄存器读写
│   │   ├── socketcan.hpp               # SocketCAN 收发封装（RAII）
│   │   └── dm_motor.hpp                # MotorState / MotorCommand 数据结构
│   ├── src/
│   │   ├── dm_protocol.cpp
│   │   ├── socketcan.cpp
│   │   └── dm_motor.cpp
│   └── test/
│       ├── test_protocol.cpp           # 打包/解包单元测试（不需要硬件）
│       └── test_socketcan_mock.cpp     # 收发逻辑测试（mock socket）
│
├── dm_gripper_core/                    # 夹爪业务逻辑，零 ROS 依赖
│   ├── CMakeLists.txt
│   ├── include/dm_gripper_core/
│   │   ├── gripper_kinematics.hpp      # q↔x↔w 传动映射，τ↔F 估算
│   │   ├── contact_detector.hpp        # 接触检测（滑窗滤波 + 位置卡死）
│   │   ├── gripper_state_machine.hpp   # 状态机：IDLE/APPROACHING/CONTACTING/GRASPING/RELEASING
│   │   ├── impedance_controller.hpp    # 模式 A：虚拟弹簧-阻尼
│   │   └── force_position_controller.hpp  # 模式 B：分阶段力位混合
│   ├── src/
│   │   ├── gripper_kinematics.cpp
│   │   ├── contact_detector.cpp
│   │   ├── gripper_state_machine.cpp
│   │   ├── impedance_controller.cpp
│   │   └── force_position_controller.cpp
│   └── test/
│       ├── test_kinematics.cpp
│       ├── test_contact_detector.cpp
│       ├── test_state_machine.cpp
│       └── test_controllers.cpp
│
├── dm_gripper_hardware/                # ros2_control hardware interface
│   ├── CMakeLists.txt
│   ├── dm_gripper_hardware.xml         # pluginlib manifest
│   ├── include/dm_gripper_hardware/
│   │   └── dm_gripper_system.hpp
│   ├── src/
│   │   └── dm_gripper_system.cpp
│   └── test/
│       └── test_hardware_interface.cpp
│
├── dm_gripper_action_server/           # ROS 2 对外接口节点
│   ├── CMakeLists.txt
│   ├── include/dm_gripper_action_server/
│   │   └── gripper_action_server.hpp
│   ├── src/
│   │   └── gripper_action_server.cpp
│   └── test/
│       └── test_action_interface.cpp
│
├── dm_gripper_bringup/
│   ├── config/
│   │   ├── gripper_params.yaml
│   │   ├── ros2_control.yaml
│   │   └── calibration.yaml
│   └── launch/
│       ├── gripper_standalone.launch.py
│       └── gripper_ros2control.launch.py
│
└── docs/
    ├── experiments/
    │   ├── exp1_tau_feedback_quality.md
    │   ├── exp2_impedance_tuning.md
    │   └── exp3_contact_detection.md
    └── superpowers/plans/
        └── 2026-03-26-dm-gripper-ros2.md
```

## Phase 0: Project Initialization (DONE)

- [x] Created workspace directory structure
- [x] Wrote CMakeLists.txt and package.xml for all packages
- [x] Created all header files (.hpp)
- [x] Created all source files (.cpp) — with placeholder implementations where hardware not yet available
- [x] Created GTest stubs for all test files
- [x] Created config YAMLs and Python launch files
- [x] Created docs/experiments/ placeholders
- [x] Committed and pushed to branch claude/organize-dm-gripper-ros2-13UHB

## Next Phases

### Phase 1: Bare motor validation
- Run demo/src/dm_socketcan_demo.cpp to confirm SocketCAN communication
- Measure τ_feedback quality (see docs/experiments/exp1_tau_feedback_quality.md)

### Phase 2: Impedance controller tuning
- Build and test dm_motor_driver + dm_gripper_core
- Tune k_spring/k_damper (see docs/experiments/exp2_impedance_tuning.md)

### Phase 3: Contact detection validation
- Validate dual-criterion detector (see docs/experiments/exp3_contact_detection.md)

### Phase 4: Linkage integration
- Hardware arrives → calibrate x↔w mapping
- Update gripper_kinematics.cpp (remove placeholder)
- Update calibration.yaml

### Phase 5: ros2_control integration
- Write URDF with <ros2_control> block
- Test DmGripperSystem with controller_manager
- Validate GripperActionController end-to-end
