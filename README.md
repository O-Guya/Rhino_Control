# DM Gripper ROS 2

基于**达妙 DM4310** 电机 + 丝杆传动的 ROS 2 夹爪驱动包，支持阻抗控制与力位混合控制。

## 架构

三层解耦设计，控制模式通过策略模式在 core 层切换，上层不感知模式差异：

```
dm_motor_driver          纯 C++，零 ROS 依赖
    ↓  SocketCAN MIT 帧
dm_gripper_core          纯 C++，零 ROS 依赖
    ↓  MotorState / MotorCommand
dm_gripper_hardware      ros2_control SystemInterface
dm_gripper_action_server GripperCommand action server
dm_gripper_bringup       启动文件 + 配置
```

## 包结构

```
Rhino_Control/
├── dm_motor_driver/             # CAN 通信库（无 ROS 依赖）
│   ├── dm_motor.hpp             # MotorState, MotorCommand, MotorLimits
│   ├── dm_protocol.hpp          # MIT 帧打包/解包，使能/失能命令
│   └── socketcan.hpp            # SocketCAN RAII 封装
├── dm_gripper_core/             # 夹爪业务逻辑（无 ROS 依赖）
│   ├── gripper_kinematics.hpp   # q ↔ x ↔ w 传动映射，τ ↔ F 估算
│   ├── contact_detector.hpp     # 双判据接触检测（位置卡死 + τ 上升）
│   ├── gripper_state_machine.hpp# 状态机 + 控制策略接口
│   ├── impedance_controller.hpp # 虚拟弹簧-阻尼控制
│   └── force_position_controller.hpp # 分阶段力位混合控制
├── dm_gripper_hardware/         # ros2_control 硬件接口
├── dm_gripper_action_server/    # GripperCommand action server 节点
├── dm_gripper_bringup/          # 启动文件 + 参数配置
├── demo/                        # 独立 SocketCAN 验证程序（无 ROS）
└── docs/
    ├── experiments/             # 实验记录模板
    └── damiao_control_motor.pdf # 达妙电机手册
```

## 硬件要求

| 项目 | 规格 |
|------|------|
| 电机 | 达妙 DM4310 |
| CAN 适配器 | gs_usb 固件（识别为 `can0`） |
| 传动 | 梯形丝杆（连杆待到货） |
| 系统 | Ubuntu 22.04 + ROS 2 Humble |

## 快速开始

### 1. 配置 CAN 接口

```bash
# 查看接口是否已识别
ip link show can0

# 启动 CAN FD 接口（1M 仲裁 / 5M 数据）
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

# 验证
candump can0
```

### 2. 裸电机测试（无 ROS）

```bash
cd demo
mkdir -p build && cd build
cmake .. && make -j4
sudo ./dm_socketcan_demo        # CAN FD 模式
# 或
sudo ./dm_socketcan_demo_can    # 经典 CAN 模式
```

### 3. 编译 ROS 2 包

```bash
# 在 Rhino_Control 目录作为 colcon 工作空间根
source /opt/ros/humble/setup.bash
colcon build --packages-select dm_motor_driver dm_gripper_core
colcon test  --packages-select dm_motor_driver dm_gripper_core
```

### 4. 启动夹爪（独立模式，不依赖 ros2_control）

```bash
source install/setup.bash
ros2 launch dm_gripper_bringup gripper_standalone.launch.py
```

### 5. 发送抓取指令

```bash
# position = 目标开口宽度 [m]，max_effort = 最大力矩 [Nm]
ros2 action send_goal /gripper_command \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.03, max_effort: 5.0}}"
```

### 6. 完整 ros2_control 启动

```bash
ros2 launch dm_gripper_bringup gripper_ros2control.launch.py
```

## 配置

主要参数在 `dm_gripper_bringup/config/gripper_params.yaml`：

```yaml
gripper_action_server:
  ros__parameters:
    can_interface: "can0"
    motor_can_id: 1          # 0x01
    use_canfd: true
    screw_lead_m: 7.957747e-4  # 丝杆导程（待标定）
    kp: 20.0                 # 虚拟弹簧刚度 [Nm/rad]
    kd: 0.5                  # 阻尼 [Nm·s/rad]
    tau_max: 5.0             # 力矩饱和 [Nm]
    contact_tau_threshold: 2.0
    contact_window_size: 20
```

## 控制策略

### 模式 A：阻抗控制（默认）

```
τ_cmd = k_spring × (q_target − q) − k_damper × dq
```

适合柔顺抓取，电机 MIT 帧以纯力矩前馈发送（kp=kd=0）。

### 模式 B：力位混合控制

- **接近阶段**：PD 位置控制驱动至目标宽度
- **抓握阶段**：恒力矩前馈维持夹持力

切换控制器：在 `gripper_action_server.cpp` 中将 `ImpedanceController` 替换为 `ForcePositionController`，状态机接口不变。

## 接触检测

双重判据，两者同时满足才触发：

1. **位置卡死**：滑窗内 |Δq| < `pos_stuck_threshold`（默认 0.001 rad）
2. **τ 上升**：滑窗均值 > `tau_threshold`（默认 2.0 Nm）

> 梯形螺纹反传效率低（η ≈ 0.30），外部接触力对电机侧有力矩放大效果，有利于检测。

## 运动学（占位实现）

连杆尚未到货，当前使用线性占位映射：

```
x [m]  = q [rad] × screw_lead_m     # 丝杆传动
w [m]  = −x                          # 占位（待连杆标定后替换）
F [N]  = τ × η / screw_lead_m       # 力估算，η = 0.30
```

连杆到货后修改 `dm_gripper_core/src/gripper_kinematics.cpp` 中的 `linear_to_width()` / `width_to_linear()`，并填写 `dm_gripper_bringup/config/calibration.yaml`。

## 实验计划

| 实验 | 目标 | 文档 |
|------|------|------|
| Exp 1 | τ_feedback 质量评估（噪声、延迟） | `docs/experiments/exp1_tau_feedback_quality.md` |
| Exp 2 | 阻抗控制器参数整定 | `docs/experiments/exp2_impedance_tuning.md` |
| Exp 3 | 接触检测验证 | `docs/experiments/exp3_contact_detection.md` |

## 协议参考

MIT 帧格式（8 字节，来源：`demo/src/dm_socketcan_demo.cpp`）：

```
[0]   q_uint[15:8]
[1]   q_uint[7:0]
[2]   dq_uint[11:4]
[3]   dq_uint[3:0] | kp_uint[11:8]
[4]   kp_uint[7:0]
[5]   kd_uint[11:4]
[6]   kd_uint[3:0] | tau_uint[11:8]
[7]   tau_uint[7:0]
```

DM4310 限位（CAN FD）：q ±12.5 rad，dq ±45 rad/s，τ ±18 Nm，Kp [0,500]，Kd [0,5]

特殊命令：

| 命令 | CAN ID | 数据 |
|------|--------|------|
| 使能 | 0x01 | `FF FF FF FF FF FF FF FC` |
| 失能 | 0x01 | `FF FF FF FF FF FF FF FD` |
| 切换 MIT 模式 | 0x7FF | `01 00 55 0A 01 00 00 00` |

## 开发分支

当前分支：`claude/organize-dm-gripper-ros2-13UHB`
