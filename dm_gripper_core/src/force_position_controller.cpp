#include "dm_gripper_core/force_position_controller.hpp"

#include <algorithm>

namespace dm_gripper_core {

ForcePositionController::ForcePositionController(Params p)
    : p_(p)
{
    // Pre-compute grasp torque from force target
    // τ = F * lead / η
    grasp_tau_ = (p_.grasp_force_target * p_.screw_lead_m) / p_.screw_efficiency;
    grasp_tau_ = std::clamp(grasp_tau_, 0.0f, p_.tau_max);
}

dm_motor_driver::MotorCommand ForcePositionController::compute(
    const dm_motor_driver::MotorState& state,
    double target_pos)
{
    if (in_grasp_phase_) {
        // Phase B: constant torque to maintain grip force
        return dm_motor_driver::MotorCommand{
            .kp  = 0.0f,
            .kd  = 0.0f,
            .q   = state.pos,   // don't command position movement
            .dq  = 0.0f,
            .tau = grasp_tau_
        };
    }

    // Phase A: PD position control for approach
    float q_err = static_cast<float>(target_pos) - state.pos;
    float tau   = p_.approach_kp * q_err - p_.approach_kd * state.vel;
    tau         = std::clamp(tau, -p_.tau_max, p_.tau_max);

    return dm_motor_driver::MotorCommand{
        .kp  = 0.0f,
        .kd  = 0.0f,
        .q   = static_cast<float>(target_pos),
        .dq  = 0.0f,
        .tau = tau
    };
}

void ForcePositionController::start_grasp_phase()
{
    in_grasp_phase_ = true;
}

void ForcePositionController::stop_grasp_phase()
{
    in_grasp_phase_ = false;
}

} // namespace dm_gripper_core
