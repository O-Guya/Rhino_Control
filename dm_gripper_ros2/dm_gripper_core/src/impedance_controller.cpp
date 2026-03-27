#include "dm_gripper_core/impedance_controller.hpp"
#include <algorithm>  // std::clamp
#include <cmath>      // std::abs

namespace dm_gripper_core {

ImpedanceController::ImpedanceController(const Params& p) : p_(p) {}

GripperMotorCmd ImpedanceController::compute(
    float q_eq, float q_actual, float dq_actual) const
{
    GripperMotorCmd cmd;
    cmd.q_des  = q_eq;    // 期望位置传给硬件 PD
    cmd.dq_des = 0.0f;    // 期望速度为零
    cmd.kp     = p_.kp;
    cmd.kd     = p_.kd;
    cmd.tau_ff = 0.0f;    // 前馈力矩由硬件 PD 负责，不额外叠加

    // 软件侧估算实际会输出的力矩，用于 clamp 检查
    // 注：实际钳位在电机驱动侧由 MotorLimits.tau_max 保证，
    //     这里仅做软件层安全检查，必要时降低 kp（设为 tau_max / |pos_err|）
    float pos_err  = q_eq - q_actual;
    float estimated_tau = p_.kp * pos_err - p_.kd * dq_actual;

    if (std::abs(estimated_tau) > p_.tau_max) {
        // 超出 tau_max：按比例缩小 kp，保持方向不变
        // 等效：让 kp * |pos_err| = tau_max（忽略阻尼项简化处理）
        if (std::abs(pos_err) > 1e-6f) {
            float safe_kp = p_.tau_max / std::abs(pos_err);
            cmd.kp = std::min(cmd.kp, safe_kp);
        }
    }

    return cmd;
}

}  // namespace dm_gripper_core
