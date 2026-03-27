#include <gtest/gtest.h>
#include "dm_gripper_core/force_position_controller.hpp"

using namespace dm_gripper_core;

static ForcePositionController::Params default_params() { return {}; }

static GripperMotorState make_state(float pos=0.0f, float vel=0.0f, float tau=0.0f) {
    return {pos, vel, tau};
}

TEST(ForcePositionController, IdleHoldsCurrentPos) {
    ForcePositionController ctrl(default_params());
    auto cmd = ctrl.compute(GripperPhase::IDLE, make_state(1.5f), 2.0f, 1.0f);

    EXPECT_NEAR(cmd.q_des, 1.5f, 1e-6f);  // 保持当前位置
    EXPECT_NEAR(cmd.kp, 0.0f, 1e-6f);     // kp=0
    EXPECT_GT(cmd.kd, 0.0f);              // 有阻尼
    EXPECT_NEAR(cmd.tau_ff, 0.0f, 1e-6f);
}

TEST(ForcePositionController, ApproachingTargetsGoal) {
    ForcePositionController ctrl(default_params());
    auto cmd = ctrl.compute(GripperPhase::APPROACHING, make_state(0.5f), 2.0f, 0.0f);

    EXPECT_NEAR(cmd.q_des, 2.0f, 1e-6f);   // 指向目标
    EXPECT_NEAR(cmd.kp, 30.0f, 1e-4f);
    EXPECT_NEAR(cmd.kd, 1.0f, 1e-4f);
    EXPECT_NEAR(cmd.tau_ff, 0.0f, 1e-6f);
}

TEST(ForcePositionController, ContactingLowStiffness) {
    ForcePositionController ctrl(default_params());
    auto cmd = ctrl.compute(GripperPhase::CONTACTING, make_state(0.5f), 2.0f, 0.0f);

    EXPECT_NEAR(cmd.kp, 5.0f, 1e-4f);
    EXPECT_NEAR(cmd.kd, 2.0f, 1e-4f);
    EXPECT_NEAR(cmd.tau_ff, 0.0f, 1e-6f);
}

TEST(ForcePositionController, GraspingPureTorque) {
    ForcePositionController ctrl(default_params());
    auto cmd = ctrl.compute(GripperPhase::GRASPING, make_state(1.0f), 2.0f, 1.5f);

    EXPECT_NEAR(cmd.kp, 0.0f, 1e-6f);       // kp=0
    EXPECT_NEAR(cmd.tau_ff, 1.5f, 1e-5f);   // 传入的 target_tau
}

TEST(ForcePositionController, GraspingTauClamped) {
    ForcePositionController ctrl(default_params());
    // target_tau=5.0 > tau_max=3.0 → 应 clamp 到 3.0
    auto cmd = ctrl.compute(GripperPhase::GRASPING, make_state(1.0f), 2.0f, 5.0f);
    EXPECT_NEAR(cmd.tau_ff, 3.0f, 1e-5f);
}

TEST(ForcePositionController, GraspingNegativeTauClampedZero) {
    ForcePositionController ctrl(default_params());
    // 负力矩无意义，应 clamp 到 0
    auto cmd = ctrl.compute(GripperPhase::GRASPING, make_state(1.0f), 2.0f, -1.0f);
    EXPECT_NEAR(cmd.tau_ff, 0.0f, 1e-5f);
}
