#include "dm_gripper_core/impedance_controller.hpp"

#include <algorithm>

namespace dm_gripper_core {

ImpedanceController::ImpedanceController(Params p)
    : p_(p)
{}

dm_motor_driver::MotorCommand ImpedanceController::compute(
    const dm_motor_driver::MotorState& state,
    double target_pos)
{
    float q_err = static_cast<float>(target_pos) - state.pos;
    float tau   = p_.k_spring * q_err - p_.k_damper * state.vel;

    // Saturate output
    tau = std::clamp(tau, -p_.tau_max, p_.tau_max);

    // Send as pure torque feed-forward; motor's internal PD disabled (kp=kd=0)
    return dm_motor_driver::MotorCommand{
        .kp  = 0.0f,
        .kd  = 0.0f,
        .q   = static_cast<float>(target_pos),
        .dq  = 0.0f,
        .tau = tau
    };
}

} // namespace dm_gripper_core
