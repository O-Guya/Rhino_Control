#include <gtest/gtest.h>
#include "dm_gripper_core/gripper_kinematics.hpp"

using namespace dm_gripper_core;

static constexpr double kLead = 7.957747e-4;  // 5mm/rev in m/rad

TEST(Kinematics, RoundTripMotorToLinear)
{
    GripperKinematics kin(kLead);
    double q    = 1.5;  // rad
    double x    = kin.motor_pos_to_linear(q);
    double q_back = kin.linear_to_motor_pos(x);
    EXPECT_NEAR(q_back, q, 1e-9);
}

TEST(Kinematics, ZeroPos)
{
    GripperKinematics kin(kLead);
    EXPECT_NEAR(kin.motor_pos_to_linear(0.0), 0.0, 1e-12);
    EXPECT_NEAR(kin.linear_to_motor_pos(0.0), 0.0, 1e-12);
}

TEST(Kinematics, TorqueToForce)
{
    GripperKinematics kin(kLead);
    // F = τ * η / lead = 1.0 * 0.30 / 7.957747e-4 ≈ 377 N
    double F = kin.torque_to_force(1.0);
    EXPECT_NEAR(F, 0.30 / kLead, 0.01);
}

TEST(Kinematics, ForceToTorqueRoundTrip)
{
    GripperKinematics kin(kLead);
    double F  = 20.0;
    double tau = kin.force_to_torque(F);
    double F_back = kin.torque_to_force(tau);
    EXPECT_NEAR(F_back, F, 1e-6);
}

TEST(Kinematics, InvalidLeadThrows)
{
    EXPECT_THROW(GripperKinematics(0.0), std::invalid_argument);
    EXPECT_THROW(GripperKinematics(-1.0), std::invalid_argument);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
