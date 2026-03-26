#pragma once

#include "contact_detector.hpp"
#include "gripper_kinematics.hpp"
#include "dm_motor_driver/dm_motor.hpp"

#include <memory>

namespace dm_gripper_core {

// ── Control strategy interface (Strategy pattern) ─────────────────────────────

class IController {
public:
    virtual ~IController() = default;

    /// Compute a motor command given the current motor state and desired position.
    /// @param state      latest feedback from the motor
    /// @param target_pos desired motor position [rad]
    virtual dm_motor_driver::MotorCommand compute(
        const dm_motor_driver::MotorState& state,
        double target_pos) = 0;
};

// ── Gripper states ────────────────────────────────────────────────────────────

enum class GripperState {
    IDLE,         ///< Motors disabled / holding still
    APPROACHING,  ///< Moving toward target width
    CONTACTING,   ///< Contact detected, switching to force control
    GRASPING,     ///< Holding object with force control
    RELEASING,    ///< Moving to open position
};

// ── State machine ─────────────────────────────────────────────────────────────

class GripperStateMachine {
public:
    GripperStateMachine(GripperKinematics kin, ContactDetector::Params contact_params);

    /// Replace the active control strategy (Strategy pattern).
    /// Takes ownership of the controller.
    void set_controller(std::unique_ptr<IController> ctrl);

    /// Command the gripper to move to target_width [m].
    /// Transitions from IDLE → APPROACHING (or RELEASING if opening).
    void command_width(double target_width_m);

    /// Command the gripper to open fully (go to max width).
    void command_open(double max_width_m);

    /// Run one control cycle. Call at control loop rate.
    /// @returns the motor command to apply this cycle.
    dm_motor_driver::MotorCommand update(const dm_motor_driver::MotorState& ms);

    GripperState state() const { return state_; }

    bool is_done() const {
        return state_ == GripperState::IDLE ||
               state_ == GripperState::GRASPING;
    }

private:
    void transition_to(GripperState next);

    GripperState          state_{GripperState::IDLE};
    GripperKinematics     kin_;
    ContactDetector       contact_detector_;
    std::unique_ptr<IController> controller_;

    double target_pos_rad_{0.0};   // in motor coordinates
    double open_pos_rad_{0.0};     // "fully open" motor position
};

} // namespace dm_gripper_core
