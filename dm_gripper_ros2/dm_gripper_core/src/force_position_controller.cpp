#include "dm_gripper_core/force_position_controller.hpp"
#include <algorithm>  // std::clamp

namespace dm_gripper_core {

ForcePositionController::ForcePositionController(const Params& p) : p_(p) {}

GripperMotorCmd ForcePositionController::compute(
    GripperPhase phase,
    const GripperMotorState& state,
    float q_target,
    float target_tau) const
{
    GripperMotorCmd cmd;
    cmd.dq_des = 0.0f;

    switch (phase) {

    case GripperPhase::IDLE:
        // 纯阻尼：不追踪目标，保持当前位置
        cmd.q_des  = state.pos;
        cmd.kp     = p_.idle_kp;
        cmd.kd     = p_.idle_kd;
        cmd.tau_ff = 0.0f;
        break;

    case GripperPhase::APPROACHING:
        // 位置控制：高刚度趋近目标
        cmd.q_des  = q_target;
        cmd.kp     = p_.approaching_kp;
        cmd.kd     = p_.approaching_kd;
        cmd.tau_ff = 0.0f;
        break;

    case GripperPhase::CONTACTING:
        // 低刚度轻推：降低冲击，等待力矩建立
        cmd.q_des  = q_target;
        cmd.kp     = p_.contacting_kp;
        cmd.kd     = p_.contacting_kd;
        cmd.tau_ff = 0.0f;
        break;

    case GripperPhase::GRASPING:
        // 纯力矩控制：kp=0，tau_ff 维持夹持力
        // target_tau 限幅到 tau_max，避免超压
        cmd.q_des  = state.pos;  // kp=0 时 q_des 无实际作用，设为当前位置
        cmd.kp     = p_.grasping_kp;
        cmd.kd     = p_.grasping_kd;
        cmd.tau_ff = std::clamp(target_tau, 0.0f, p_.tau_max);
        break;
    }

    return cmd;
}

}  // namespace dm_gripper_core
