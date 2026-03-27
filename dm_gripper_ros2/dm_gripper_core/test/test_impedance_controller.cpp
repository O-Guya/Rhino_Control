#include <gtest/gtest.h>
#include "dm_gripper_core/impedance_controller.hpp"
#include <cmath>

using namespace dm_gripper_core;

static ImpedanceController::Params default_params() {
    return {};  // kp=30, kd=1.0, tau_max=3.0
}

TEST(ImpedanceController, ZeroErrorZeroOutput) {
    ImpedanceController ctrl(default_params());
    // q_eq == q_actual，dq=0 → 期望力矩为 0，kp 不应被 clamp
    auto cmd = ctrl.compute(1.0f, 1.0f, 0.0f);
    EXPECT_NEAR(cmd.kp, 30.0f, 1e-4f);
    EXPECT_NEAR(cmd.q_des, 1.0f, 1e-6f);
}

TEST(ImpedanceController, KpKdCorrect) {
    ImpedanceController ctrl(default_params());
    auto cmd = ctrl.compute(0.5f, 0.0f, 0.0f);  // pos_err=0.5, dq=0
    // estimated_tau = 30 * 0.5 = 15 > tau_max=3
    // safe_kp = 3.0 / 0.5 = 6.0 → kp 被限制为 6
    EXPECT_NEAR(cmd.kp, 6.0f, 1e-4f);
    EXPECT_NEAR(cmd.kd, 1.0f, 1e-4f);
}

TEST(ImpedanceController, NoClampWhenSmallError) {
    ImpedanceController ctrl(default_params());
    // pos_err=0.05 → estimated_tau = 30*0.05 = 1.5 < tau_max=3 → 不 clamp
    auto cmd = ctrl.compute(0.05f, 0.0f, 0.0f);
    EXPECT_NEAR(cmd.kp, 30.0f, 1e-4f);
}

TEST(ImpedanceController, TauMaxRespected) {
    ImpedanceController ctrl(default_params());
    // 大误差：pos_err=2.0 → estimated_tau = 60 > tau_max=3
    // safe_kp = 3/2 = 1.5
    auto cmd = ctrl.compute(2.0f, 0.0f, 0.0f);
    EXPECT_LE(cmd.kp, 1.5f + 1e-4f);
}

TEST(ImpedanceController, CustomTauMax) {
    ImpedanceController::Params p;
    p.kp = 50.0f; p.kd = 2.0f; p.tau_max = 5.0f;
    ImpedanceController ctrl(p);

    // pos_err=0.2 → estimated = 50*0.2 = 10 > 5 → safe_kp = 5/0.2 = 25
    auto cmd = ctrl.compute(0.2f, 0.0f, 0.0f);
    EXPECT_NEAR(cmd.kp, 25.0f, 1e-3f);
}

TEST(ImpedanceController, TauFFIsZero) {
    ImpedanceController ctrl(default_params());
    auto cmd = ctrl.compute(1.0f, 0.0f, 0.0f);
    EXPECT_NEAR(cmd.tau_ff, 0.0f, 1e-6f);
}
