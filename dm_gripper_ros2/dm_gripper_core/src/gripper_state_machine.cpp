#include "dm_gripper_core/gripper_state_machine.hpp"

namespace dm_gripper_core {

GripperStateMachine::GripperStateMachine(const Params& p)
    : p_(p), contact_(p.contact) {}

void GripperStateMachine::start_grasp(const Goal& goal) {
    current_goal_ = goal;
    enter_phase(GripperPhase::APPROACHING);
}

void GripperStateMachine::release() {
    enter_phase(GripperPhase::IDLE);
}

void GripperStateMachine::enter_phase(GripperPhase new_phase) {
    phase_        = new_phase;
    phase_time_ms_ = 0.0f;
    contact_.reset();  // 进入新阶段时重置接触检测
}

GripperMotorCmd GripperStateMachine::update(const GripperMotorState& state, float dt_ms) {
    phase_time_ms_ += dt_ms;

    // 更新接触检测（IDLE 时不检测）
    if (phase_ != GripperPhase::IDLE) {
        contact_.update(current_goal_.target_q_rad, state.pos, state.tau);
    }

    switch (phase_) {

    case GripperPhase::IDLE:
        // 纯阻尼保持当前位置，不主动运动
        return make_cmd(state.pos);

    case GripperPhase::APPROACHING:
        // 超时 → 回 IDLE
        if (p_.approaching.timeout_ms > 0 && phase_time_ms_ > p_.approaching.timeout_ms) {
            enter_phase(GripperPhase::IDLE);
            return make_cmd(state.pos);
        }
        // 检测到接触 → 进入 CONTACTING
        if (contact_.is_contact()) {
            enter_phase(GripperPhase::CONTACTING);
        }
        return make_cmd(current_goal_.target_q_rad);

    case GripperPhase::CONTACTING:
        // 超时 → 回 IDLE
        if (p_.contacting.timeout_ms > 0 && phase_time_ms_ > p_.contacting.timeout_ms) {
            enter_phase(GripperPhase::IDLE);
            return make_cmd(state.pos);
        }
        // 力矩建立（tau 持续超阈）→ 进入 GRASPING
        if (state.tau > p_.contact.tau_threshold) {
            enter_phase(GripperPhase::GRASPING);
        }
        return make_cmd(current_goal_.target_q_rad);

    case GripperPhase::GRASPING:
        // 超压保护：tau 超过 goal.max_tau → 释放
        if (over_torque(state.tau)) {
            enter_phase(GripperPhase::IDLE);
            return make_cmd(state.pos);
        }
        // GRASPING 阶段：kp=0，纯力矩维持
        // q_des 设为当前位置（kp=0 时无效，但保持字段合理）
        return make_cmd(state.pos);
    }

    // 不应到达这里
    return make_cmd(state.pos);
}

GripperMotorCmd GripperStateMachine::make_cmd(float q_des) const {
    GripperMotorCmd cmd;
    cmd.q_des  = q_des;
    cmd.dq_des = 0.0f;

    // 根据当前阶段选参数
    switch (phase_) {
    case GripperPhase::IDLE:
        cmd.kp = p_.idle.kp; cmd.kd = p_.idle.kd; cmd.tau_ff = p_.idle.tau_ff;
        break;
    case GripperPhase::APPROACHING:
        cmd.kp = p_.approaching.kp; cmd.kd = p_.approaching.kd; cmd.tau_ff = p_.approaching.tau_ff;
        break;
    case GripperPhase::CONTACTING:
        cmd.kp = p_.contacting.kp; cmd.kd = p_.contacting.kd; cmd.tau_ff = p_.contacting.tau_ff;
        break;
    case GripperPhase::GRASPING:
        cmd.kp = p_.grasping.kp; cmd.kd = p_.grasping.kd;
        // GRASPING 的 tau_ff 来自目标力矩（最大值的一定比例），此处用参数默认值
        cmd.tau_ff = p_.grasping.tau_ff;
        break;
    }

    return cmd;
}

bool GripperStateMachine::over_torque(float tau) const {
    return tau > current_goal_.max_tau;
}

}  // namespace dm_gripper_core
