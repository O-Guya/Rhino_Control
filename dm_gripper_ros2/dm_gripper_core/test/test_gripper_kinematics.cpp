#include <gtest/gtest.h>
#include "dm_gripper_core/gripper_kinematics.hpp"
#include <cmath>

using namespace dm_gripper_core;

static GripperKinematics make_kin() {
    return GripperKinematics{GripperKinematics::Params{}};
}

// ─── 零位验证 ─────────────────────────────────────────────────────────────

TEST(GripperKinematics, ZeroPosWidth70mm) {
    // q=0 时 STEP 实测接触面在 X=-35mm，双侧开口 = 70mm
    auto kin = make_kin();
    EXPECT_NEAR(kin.motor_pos_to_width(0.0f), 70.0f, 0.1f);
}

TEST(GripperKinematics, ZeroPosInverseIsZero) {
    auto kin = make_kin();
    float w0 = kin.motor_pos_to_width(0.0f);
    EXPECT_NEAR(kin.width_to_motor_pos(w0), 0.0f, 1e-3f);
}

// ─── 单调性 ───────────────────────────────────────────────────────────────

TEST(GripperKinematics, PositiveQClosesGripper) {
    // q 增大 → 收紧 → w 减小
    auto kin = make_kin();
    EXPECT_LT(kin.motor_pos_to_width(3.0f), kin.motor_pos_to_width(0.0f));
}

TEST(GripperKinematics, NegativeQOpensGripper) {
    // q 减小 → 张开 → w 增大
    auto kin = make_kin();
    EXPECT_GT(kin.motor_pos_to_width(-5.0f), kin.motor_pos_to_width(0.0f));
}

// ─── 往返精度（仅单调区间内）────────────────────────────────────────────

TEST(GripperKinematics, RoundTripAccuracy) {
    auto kin = make_kin();
    // 在 [q_min, q_max] 单调区间内测试往返精度 < 0.01 rad
    for (float q : {-10.0f, -5.0f, -2.0f, 0.0f, 2.0f, 5.0f, 7.0f}) {
        float w = kin.motor_pos_to_width(q);
        float q_back = kin.width_to_motor_pos(w);
        EXPECT_NEAR(q_back, q, 0.01f) << "往返误差超标，q=" << q;
    }
}

// ─── 宽度范围 ─────────────────────────────────────────────────────────────

TEST(GripperKinematics, WidthRangeValid) {
    auto kin = make_kin();
    auto [w_min, w_max] = kin.get_width_range();
    EXPECT_LT(w_min, w_max);
    EXPECT_GT(w_min, 50.0f);
    EXPECT_LT(w_max, 100.0f);
}

// ─── 边界不崩溃 ───────────────────────────────────────────────────────────

TEST(GripperKinematics, BoundaryNoCrash) {
    auto kin = make_kin();
    EXPECT_NO_THROW(kin.motor_pos_to_width(-12.5f));
    EXPECT_NO_THROW(kin.motor_pos_to_width(8.0f));
    EXPECT_NO_THROW(kin.motor_pos_to_width(-100.f));
    EXPECT_NO_THROW(kin.motor_pos_to_width(100.f));
}

TEST(GripperKinematics, WidthClampNoCrash) {
    auto kin = make_kin();
    auto [w_min, w_max] = kin.get_width_range();
    EXPECT_NO_THROW(kin.width_to_motor_pos(w_min - 5.0f));
    EXPECT_NO_THROW(kin.width_to_motor_pos(w_max + 5.0f));
}

// ─── 力矩→夹持力 ──────────────────────────────────────────────────────────

TEST(GripperKinematics, TorqueToForcePositive) {
    // 正 tau（收紧）→ 正夹持力
    auto kin = make_kin();
    float F = kin.torque_to_force(1.0f, 0.0f);
    EXPECT_GT(F, 0.0f) << "正力矩应产生正夹持力";
}

TEST(GripperKinematics, TorqueToForceProportion) {
    // 力矩翻倍 → 力翻倍
    auto kin = make_kin();
    float F1 = kin.torque_to_force(1.0f, 0.0f);
    float F2 = kin.torque_to_force(2.0f, 0.0f);
    EXPECT_NEAR(F2, 2.0f * F1, 1.0f);  // 精度 1N（浮点数值微分）
}

TEST(GripperKinematics, TorqueToForceReasonableMagnitude) {
    // 丝杆机构高机械增益：1Nm 应产生 >100N 夹持力
    auto kin = make_kin();
    float F = kin.torque_to_force(1.0f, 0.0f);
    EXPECT_GT(F, 100.0f) << "丝杆高机械增益，1Nm应产生>100N";
}

TEST(GripperKinematics, ZeroTauZeroForce) {
    auto kin = make_kin();
    EXPECT_NEAR(kin.torque_to_force(0.0f, 0.0f), 0.0f, 1e-4f);
}

// ─── 无效参数 ─────────────────────────────────────────────────────────────

TEST(GripperKinematics, InvalidPitchThrows) {
    GripperKinematics::Params p;
    p.pitch_mm = 0.0f;
    EXPECT_THROW(GripperKinematics{p}, std::invalid_argument);
}

TEST(GripperKinematics, NegativePitchThrows) {
    GripperKinematics::Params p;
    p.pitch_mm = -1.0f;
    EXPECT_THROW(GripperKinematics{p}, std::invalid_argument);
}
