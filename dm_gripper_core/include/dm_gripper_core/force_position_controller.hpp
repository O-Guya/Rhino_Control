#pragma once

#include "gripper_state_machine.hpp"
#include "dm_motor_driver/dm_motor.hpp"

namespace dm_gripper_core {

/// Force-position hybrid controller.
///
/// Phase A (approaching): PD position control to drive toward target.
/// Phase B (grasping):    Constant torque feed-forward to maintain grip force.
///
/// Phase switching is driven externally by the state machine (via target_pos
/// being set to the contact position when CONTACTING state is entered).
class ForcePositionController : public IController {
public:
    struct Params {
        float approach_kp;         // [Nm/rad]    position gain during approach
        float approach_kd;         // [Nm·s/rad]  velocity damping during approach
        float grasp_force_target;  // [N]         desired jaw force during grasp
        float tau_max;             // [Nm]        output saturation
        float screw_lead_m;        // [m/rad]     for force→torque conversion
        float screw_efficiency;    // []          η, typically 0.30
    };

    explicit ForcePositionController(Params p);

    dm_motor_driver::MotorCommand compute(
        const dm_motor_driver::MotorState& state,
        double target_pos) override;

    /// Call when transitioning to GRASPING phase to fix the grasp torque.
    void start_grasp_phase();
    void stop_grasp_phase();

private:
    Params p_;
    bool   in_grasp_phase_{false};
    float  grasp_tau_{0.0f};  // pre-computed from force target
};

} // namespace dm_gripper_core
