# 实验报告：τ_feedback 质量评估

**日期：** ___________  
**操作者：** ___________  
**电机型号：** DM4310  
**电机 CAN ID：** 0x___  
**工具：** `dm_motor_driver/tools/tau_logger`  

---

## 实验目的

回答一个核心问题：**电机自身的 τ_feedback（电流估算力矩）是否具备足够的信噪比，可用于夹爪控制的接触检测和力控闭环？**

该问题的答案决定后续：
- 接触检测策略（τ 阈值 OR 纯位置卡死 OR 双重判据）
- 力位混合控制器中 GRASPING 阶段的可靠性
- kp/kd 的推荐初始值范围

---

## 实验环境

```
CAN 接口初始化命令（运行前执行）：
  sudo ip link set can0 up type can bitrate 1000000

tau_logger 编译：
  cd dm_motor_driver && mkdir -p build && cd build
  g++ -std=c++17 -O2 -I../include ../tools/tau_logger.cpp -lpthread -o tau_logger
```

---

## 实验 1：空载力矩扫描（freerun）

**目的：** 量化 τ_feedback 的噪声基底，确认无外力时测量精度。

**运行命令：**
```bash
sudo ./tau_logger freerun --iface can0 --id 0x01 --out csv/exp1_freerun.csv
```

**实验条件：** 电机轴自由，无外力，轴上无负载。

### 结果记录

| tau_ff (Nm) | tau_feedback 均值 (Nm) | tau_feedback 标准差 (Nm) | 偏差 (cmd-fb) |
|-------------|------------------------|--------------------------|---------------|
|  0.0000     |        -0.0080         |         0.0018           |               |
| 0.1         |          0.0891        |         0.0065           |               |
| 0.5         |          0.1094        |         0.0222           |               |
| 1.0         |          0.1194        |         0.0125           |               |
| 2.0         |          0.1363        |         0.0147           |               |

> 工具运行完毕后会自动打印上表，直接填入即可。

**噪声基底（无外力时 std）：** ________ Nm

**结论：**
- [ ] 噪声 < 0.05 Nm → 优秀，可支持精细力控
- [ ] 噪声 0.05-0.2 Nm → 可用，接触检测阈值需 > 3σ
- [ ] 噪声 > 0.2 Nm → 较差，退化为位置卡死检测为主

---

## 实验 2：手动阻力实验（manual）

**目的：** 确认正向传动（电机主动输出）时，外力是否能在 τ_feedback 中反映出来。

**运行命令：**
```bash
sudo ./tau_logger manual --iface can0 --id 0x01 --tau 1.0 --out csv/exp1_manual_load.csv
```

**操作步骤：**
1. 启动程序，观察终端实时显示的 tau_feedback
2. 前 10 秒：手不接触电机轴（自由旋转）
3. 后段：用手缓慢握住并阻拦电机轴旋转
4. 感受阻力增大时 tau_feedback 的变化
5. Ctrl+C 结束

### 结果记录

自由段 tau_feedback：均值  0.11~0.14 Nm，std  0.022 Nm
施力段 tau_feedback：均值  ~0.42 Nm（峰值），std 估算 ~0.05 Nm
可检测差值：~0.28 Nm


**现象描述：**
```
  手动握住轴时 tau_feedback 从基线 ~0.13 上升至 ~0.42 Nm，
  变化清晰可见。因电机转矩较大，难以完全制停，
  测得值为部分阻力下的读数，实际夹持场景（低速/静止）信号会更稳定。
```

**结论：**
- [1] 变化 > 0.5 Nm 且清晰 → τ_feedback 可用于力控
- [1] 变化可检测但噪声大 → 需要低通滤波后使用
- [ ] 变化 < 噪声基底 → τ_feedback 不可靠，退化为位置控制

---

## 实验 3：摩擦力矩估算（friction）

**目的：** 通过位置往返，估算电机的动摩擦力矩量级。

**运行命令：**
```bash
sudo ./tau_logger friction --iface can0 --id 0x01 --pos 3.14 --out csv/exp1_friction.csv
```

**操作步骤：** 无需手动操作，程序自动往返运动。

### 结果记录

**正向到达 pos_b 时 τ_feedback：** _-0.1339_ Nm  
**逆向到达 pos_b 时 τ_feedback：** _0.1173_ Nm  
**差值（= 2 × 动摩擦估算）：** _0.2512_ Nm  
**单向动摩擦力矩估算：** _0.1256_ Nm  

**结论：**
- [ ] 摩擦 < 0.1 Nm → 可忽略，力控精度较高
- [ ] 摩擦 0.1-0.5 Nm → 需在控制律中加前馈补偿
- [ ] 摩擦 > 0.5 Nm → 摩擦主导，力分辨率受限

---

## 综合结论

### τ_feedback 可用性判断

| 指标 | 测量值 | 判断 |
|------|--------|------|
| 噪声标准差 (Nm) | | |
| 手动施力可检测信号 (Nm) | | |
| 动摩擦力矩估算 (Nm) | | |
| SNR（信号/噪声） | | |

### 控制策略建议（根据数据填写）

**接触检测阈值（推荐 > 3σ 噪声）：**
```yaml
# 填入 dm_gripper_bringup/config/gripper_params.yaml
contact_detector:
  tau_threshold: ______  # Nm
  q_error_threshold: ______  # rad（下一个实验确定）
  window_size: 10  # 帧数（约 50ms）
```

**夹持力控可靠性评估：**
- [ ] τ_feedback 可直接用于力矩闭环（SNR > 10）
- [ ] τ_feedback 仅用于接触检测，夹持靠位置+tau_ff 开环维持
- [ ] 退化方案：纯位置卡死检测 + tau_ff 开环夹持

### 附加观察（可选）

```
（记录任何异常现象，如：某个 tau_ff 档位有异响、
  tau_feedback 在某方向有明显偏置、
  温度上升速度等）
```

---

## 数据文件清单

| 文件 | 对应实验 | 行数 |
|------|----------|------|
| `csv/exp1_freerun.csv`      | 空载扫描 | |
| `csv/exp1_manual_load.csv`  | 手动阻力 | |
| `csv/exp1_friction.csv`     | 摩擦往返 | |

---

*报告完成后执行：*
```bash
git add docs/experiments/exp1_tau_feedback_quality.md csv/
git commit -m "exp: tau_feedback quality assessment results"
```