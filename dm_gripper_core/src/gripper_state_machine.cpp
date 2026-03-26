#include "dm_gripper_core/gripper_state_machine.hpp"

#include <cmath>
#include <stdexcept>

namespace dm_gripper_core {

GripperStateMachine::GripperStateMachine(GripperKinematics kin,
                                         ContactDetector::Params contact_params)
    : kin_(kin)
    , contact_detector_(contact_params)
{}

void GripperStateMachine::set_controller(std::unique_ptr<IController> ctrl)
{
    controller_ = std::move(ctrl);
}

void GripperStateMachine::command_width(double target_width_m)
{
    double target_linear = kin_.width_to_linear(target_width_m);
    target_pos_rad_ = kin_.linear_to_motor_pos(target_linear);
    contact_detector_.reset();
    transition_to(GripperState::APPROACHING);
}

void GripperStateMachine::command_open(double max_width_m)
{
    open_pos_rad_   = kin_.linear_to_motor_pos(kin_.width_to_linear(max_width_m));
    target_pos_rad_ = open_pos_rad_;
    contact_detector_.reset();
    transition_to(GripperState::RELEASING);
}

dm_motor_driver::MotorCommand GripperStateMachine::update(
    const dm_motor_driver::MotorState& ms)
{
    if (!controller_) {
        // Safety: no controller set → hold still with damping
        return dm_motor_driver::MotorCommand{
            .kp = 0.0f, .kd = 0.5f, .q = ms.pos, .dq = 0.0f, .tau = 0.0f
        };
    }

    switch (state_) {
        case GripperState::IDLE:
            // Hold current position passively
            return dm_motor_driver::MotorCommand{
                .kp = 0.0f, .kd = 0.3f, .q = ms.pos, .dq = 0.0f, .tau = 0.0f
            };

        case GripperState::APPROACHING: {
            bool contact = contact_detector_.update(ms.pos, ms.tau);
            if (contact) {
                transition_to(GripperState::CONTACTING);
            }
            return controller_->compute(ms, target_pos_rad_);
        }

        case GripperState::CONTACTING:
            // Hold at contact position — state machine caller decides
            // when to transition to GRASPING (e.g. after force ramp).
            transition_to(GripperState::GRASPING);
            return controller_->compute(ms, ms.pos);  // freeze at current pos

        case GripperState::GRASPING:
            // Maintain grasp force — controller handles this
            return controller_->compute(ms, target_pos_rad_);

        case GripperState::RELEASING: {
            // Move to open position; once arrived, go IDLE
            bool arrived = std::fabs(ms.pos - open_pos_rad_) < 0.05f;  // 0.05 rad tolerance
            if (arrived) {
                transition_to(GripperState::IDLE);
            }
            return controller_->compute(ms, open_pos_rad_);
        }
    }

    // Unreachable, but silence compiler
    return {};
}

void GripperStateMachine::transition_to(GripperState next)
{
    state_ = next;
}

} // namespace dm_gripper_core
