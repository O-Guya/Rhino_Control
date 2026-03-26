#pragma once

#include "gripper_state_machine.hpp"
#include "dm_motor_driver/dm_motor.hpp"

namespace dm_gripper_core {

/// Impedance controller: virtual spring-damper in motor space.
///
/// τ_cmd = k_spring * (q_target - q) - k_damper * dq
///
/// Sends the command in MIT mode with kp/kd gains set to zero
/// (full torque feed-forward, motor's internal PD disabled).
class ImpedanceController : public IController {
public:
    struct Params {
        float k_spring;  // [Nm/rad]    virtual spring stiffness
        float k_damper;  // [Nm·s/rad]  virtual damper coefficient
        float tau_max;   // [Nm]        output saturation
    };

    explicit ImpedanceController(Params p);

    dm_motor_driver::MotorCommand compute(
        const dm_motor_driver::MotorState& state,
        double target_pos) override;

private:
    Params p_;
};

} // namespace dm_gripper_core
