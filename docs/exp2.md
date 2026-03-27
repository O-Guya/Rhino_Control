实验1：用手扭纹丝不动
[INFO] 零点锁定：q_des = 0.021553 rad
  kp    | 位置误差 max(rad) | 速度 std(rad/s) | 振荡？
  ------+------------------+------------------+-------
      5 |        0.0003815 |         0.002368 | OK
     10 |        0.0003815 |                0 | OK
     20 |         0.003433 |         0.003184 | OK
     50 |         0.002289 |         0.004947 | OK
    100 |        0.0003815 |          0.00104 | OK
[CSV] 4945 records → csv/exp2_kp_sweep.csv
实验2：

[KD_SWEEP] kd 扫描（kp=20 固定）
  每个 kd 保持 5s，期间用手拨动轴一下，
  观察恢复速度和过冲量
[INFO] 零点：q_des = 0.00629425 rad
  kd    | 速度峰值(rad/s) | tau_fb std(Nm) | 手感描述
  ------+----------------+----------------+----------
    0.5 |              0 |         0.0118 | (手动记录)
      1 |              0 |          0.032 | (手动记录)
      2 |              0 |         0.0341 | (手动记录)
      5 |              0 |         0.0309 | (手动记录)
[CSV] 3956 records → csv/exp2_kd_sweep.csv
[KD_SWEEP] 完成
  临界阻尼估算：kd_critical ≈ 2 × sqrt(kp × J_eq)
  J_eq（空载约等于电机转动惯量）= 待确认
  → 将感觉最顺滑的 kd 填入 exp2_impedance_tuning.md
[OK] 安全退出
(Rhino_Control) franka@franka-ubuntu:~/Documents/Rhino_Control/dm_gripper_ros2/dm_motor_driver$ sudo ./impedance_sweep tau_limit --kp 20.0 --kd 2.0 --tau-max 2.0
===========================================
 DM4310 阻抗参数摸底工具 - Task 2
===========================================
  接口: can0  电机 ID: 0x1
  模式: tau_limit
[TAU_LIMIT] 力矩截断验证
  kp=20  kd=2  tau_max=2 Nm
  操作：用手阻拦电机轴，感受最大阻力（应等于 tau_max 对应的力）
  按 Ctrl+C 结束
[INFO] 零点：q_des = 0.00324249 rad
[INFO] 开始，实时显示中...
  q_err=0.003815 rad  tau_fb=0.05128 Nm  vel=-0.0073 rad/s^Cs
[TAU_LIMIT] 统计：
  tau_max 设定: 2 Nm
  tau_feedback 峰值: 0.13 Nm
  ✅ 力矩截断有效（tau_fb ≈ tau_max 量级）
[CSV] 5976 records → csv/exp2_tau_limit.csv
[OK] 安全退出
手感：几乎都扭不动