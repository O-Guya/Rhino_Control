#pragma once
#include "dm_gripper_core/gripper_state_machine.hpp"  // GripperMotorCmd

namespace dm_gripper_core {

/**
 * @brief 简单阻抗控制器（弹簧-阻尼）
 *
 * 输出力矩 = kp * (q_eq - q_actual) - kd * dq_actual
 * 通过 MIT 模式 tau_ff 发出，kp/kd 设为 0（纯前馈力矩控制）。
 *
 * 也可以直接用 kp/kd 字段发出（硬件内部 PD），取决于控制策略。
 * 此处采用混合方式：kp/kd 保留给硬件，tau_ff 补偿前馈。
 *
 * Phase 2 实验结论：
 *   - kp 扫至 100 无振荡（丝杆自锁）
 *   - 推荐初始值：kp=30, kd=1.0, tau_max=3.0 Nm
 */
class ImpedanceController {
public:
    struct Params {
        float kp      = 30.0f;  // 位置刚度（来自 Task2 实验）
        float kd      =  1.0f;  // 速度阻尼
        float tau_max =  3.0f;  // 输出力矩上限 (Nm)（安全上限）
    };

    explicit ImpedanceController(const Params& p);

    /**
     * @brief 计算阻抗控制指令
     * @param q_eq      平衡位置（期望角度）(rad)
     * @param q_actual  当前角度 (rad)
     * @param dq_actual 当前速度 (rad/s)
     * @return          电机指令（kp/kd 直接传给电机 MIT 模式）
     *
     * 输出 GripperMotorCmd：
     *   q_des = q_eq（让硬件 PD 也参与）
     *   kp / kd = 参数中的值
     *   tau_ff = 0（前馈由硬件 PD 完成）
     *   力矩 clamp：若 kp*(q_eq-q) + kd*(0-dq) 超过 tau_max，则缩放
     */
    GripperMotorCmd compute(float q_eq, float q_actual, float dq_actual) const;

    const Params& params() const { return p_; }

private:
    Params p_;
};

}  // namespace dm_gripper_core
