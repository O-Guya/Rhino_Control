#include <gtest/gtest.h>
#include "dm_gripper_core/impedance_controller.hpp"
#include "dm_gripper_core/force_position_controller.hpp"

using namespace dm_gripper_core;
using namespace dm_motor_driver;

static MotorState make_state(float pos = 0.0f, float vel = 0.0f, float tau = 0.0f)
{
    MotorState s;
    s.pos   = pos;
    s.vel   = vel;
    s.tau   = tau;
    s.valid = true;
    return s;
}

// ── ImpedanceController ────────────────────────────────────────────────────────

TEST(ImpedanceController, ZeroErrorZeroTau)
{
    ImpedanceController ctrl({20.0f, 0.5f, 10.0f});
    MotorCommand cmd = ctrl.compute(make_state(1.0f), 1.0);
    EXPECT_NEAR(cmd.tau, 0.0f, 0.01f);
}

TEST(ImpedanceController, PositiveErrorPositiveTau)
{
    ImpedanceController ctrl({20.0f, 0.5f, 10.0f});
    // target > current → positive torque (close direction)
    MotorCommand cmd = ctrl.compute(make_state(0.0f, 0.0f), 1.0);
    EXPECT_GT(cmd.tau, 0.0f);
}

TEST(ImpedanceController, OutputSaturation)
{
    ImpedanceController ctrl({20.0f, 0.5f, 5.0f});  // tau_max = 5.0
    // Large error
    MotorCommand cmd = ctrl.compute(make_state(0.0f), 1000.0);
    EXPECT_LE(cmd.tau, 5.0f);
    EXPECT_GE(cmd.tau, -5.0f);
}

TEST(ImpedanceController, KpKdAreZero)
{
    // Impedance controller sends torque FF only; motor PD should be disabled
    ImpedanceController ctrl({20.0f, 0.5f, 10.0f});
    MotorCommand cmd = ctrl.compute(make_state(0.0f), 0.5);
    EXPECT_EQ(cmd.kp, 0.0f);
    EXPECT_EQ(cmd.kd, 0.0f);
}

// ── ForcePositionController ────────────────────────────────────────────────────

TEST(ForcePositionController, ApproachPhasePositiveError)
{
    ForcePositionController ctrl({
        .approach_kp          = 15.0f,
        .approach_kd          = 0.3f,
        .grasp_force_target   = 10.0f,
        .tau_max              = 8.0f,
        .screw_lead_m         = 7.957747e-4f,
        .screw_efficiency     = 0.30f
    });
    MotorCommand cmd = ctrl.compute(make_state(0.0f, 0.0f), 1.0);
    EXPECT_GT(cmd.tau, 0.0f);
}

TEST(ForcePositionController, GraspPhaseConstantTorque)
{
    ForcePositionController ctrl({
        .approach_kp          = 15.0f,
        .approach_kd          = 0.3f,
        .grasp_force_target   = 10.0f,
        .tau_max              = 8.0f,
        .screw_lead_m         = 7.957747e-4f,
        .screw_efficiency     = 0.30f
    });
    ctrl.start_grasp_phase();

    MotorCommand c1 = ctrl.compute(make_state(1.0f), 1.0);
    MotorCommand c2 = ctrl.compute(make_state(1.1f), 1.0);
    // Grasp torque should be constant regardless of position
    EXPECT_NEAR(c1.tau, c2.tau, 1e-6f);
    EXPECT_GT(c1.tau, 0.0f);
}

TEST(ForcePositionController, OutputSaturation)
{
    ForcePositionController ctrl({15.0f, 0.3f, 10.0f, 5.0f, 7.957747e-4f, 0.30f});
    MotorCommand cmd = ctrl.compute(make_state(0.0f), 1000.0);
    EXPECT_LE(cmd.tau,  5.0f);
    EXPECT_GE(cmd.tau, -5.0f);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
