#include "dm_gripper_core/gripper_kinematics.hpp"

#include <cmath>
#include <stdexcept>

namespace dm_gripper_core {

GripperKinematics::GripperKinematics(double screw_lead_m)
    : screw_lead_(screw_lead_m)
{
    if (screw_lead_m <= 0.0)
        throw std::invalid_argument("screw_lead_m must be positive");
}

double GripperKinematics::motor_pos_to_linear(double q) const
{
    // x = q * lead    (positive q → positive linear displacement)
    return q * screw_lead_;
}

double GripperKinematics::linear_to_motor_pos(double x) const
{
    return x / screw_lead_;
}

double GripperKinematics::linear_to_width(double x) const
{
    // TODO: replace with calibrated linkage polynomial once linkage arrives.
    // Placeholder: width decreases as linear displacement increases (close direction).
    // w = w_max - 2 * x  (symmetric jaw, rough approximation)
    // For now, treat it as identity with sign flip so the structure is correct.
    return -x;
}

double GripperKinematics::width_to_linear(double w) const
{
    return -w;
}

double GripperKinematics::torque_to_force(double tau) const
{
    // F = τ * η / lead
    // η ≈ 0.30 for trapezoidal screw (conservative; actual back-drive efficiency lower)
    return (tau * kEta) / screw_lead_;
}

double GripperKinematics::force_to_torque(double force) const
{
    // τ = F * lead / η
    return (force * screw_lead_) / kEta;
}

} // namespace dm_gripper_core
