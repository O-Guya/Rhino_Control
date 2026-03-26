# DM Gripper ROS 2 Implementation Plan

**Date**: 2026-03-26
**Status**: Skeleton created, implementation pending

## Goal

基于达妙 DM4310 电机 + 丝杆传动，实现带阻抗控制和力位混合控制的 ROS 2 夹爪驱动包。

## Architecture

三层解耦设计：

```
dm_motor_driver  →  dm_gripper_core  →  dm_gripper_hardware
(零 ROS 依赖)       (零 ROS 依赖)         (ros2_control 插件)
                                      →  dm_gripper_action_server
                                         (ROS 2 action server)
```

## Background Constraints

- 传动：丝杆（q→线性位移）+ 连杆（线性位移→开口宽度），连杆尚未到货
- 传感器：仅电机自身 τ_feedback（12-bit 电流估算，梯形螺纹反传效率低）
- 接触检测：依赖「位置卡死 + τ 上升」双重判据
- 当前硬件：裸电机可用，连杆待到货后补充标定

## Phases

### Phase 0: Project Skeleton (DONE)
- [x] 创建工作空间目录结构
- [x] 编写各包 CMakeLists.txt 和 package.xml
- [x] 骨架头文件和源文件占位

### Phase 1: dm_motor_driver Implementation
- [ ] dm_protocol: MIT 帧打包/解包（参考 demo/src/dm_socketcan_demo_can.cpp）
- [ ] socketcan: RAII SocketCAN 封装
- [ ] dm_motor: MotorState/MotorCommand 数据结构
- [ ] 单元测试

### Phase 2: dm_gripper_core Implementation
- [ ] gripper_kinematics: q↔x↔w 映射（连杆到货后补全）
- [ ] contact_detector: 滑窗滤波 + 位置卡死
- [ ] gripper_state_machine: 状态机
- [ ] impedance_controller: 虚拟弹簧-阻尼
- [ ] force_position_controller: 力位混合

### Phase 3: ROS 2 Integration
- [ ] dm_gripper_hardware: ros2_control SystemInterface
- [ ] dm_gripper_action_server: GripperCommand action server
- [ ] dm_gripper_bringup: launch files

### Phase 4: Experiments
- [ ] Exp 1: τ feedback quality
- [ ] Exp 2: impedance tuning
- [ ] Exp 3: contact detection validation
