#include <gtest/gtest.h>
#include "dm_gripper_core/gripper_state_machine.hpp"

using namespace dm_gripper_core;

// 构造一套测试用参数（超时设短，便于测试）
static GripperStateMachine::Params test_params() {
    GripperStateMachine::Params p;
    p.approaching.timeout_ms = 500.0f;   // 0.5s 超时
    p.contacting.timeout_ms  = 300.0f;   // 0.3s 超时
    p.contact.tau_threshold  = 0.24f;
    p.contact.window_size    = 3;        // 小窗口便于测试
    return p;
}

// 辅助：构造正常状态（无接触）
static GripperMotorState idle_state(float pos = 0.0f) {
    return {pos, 0.0f, 0.0f};
}

TEST(GripperStateMachine, InitialIdle) {
    GripperStateMachine sm(test_params());
    EXPECT_EQ(sm.current_phase(), GripperPhase::IDLE);
    EXPECT_FALSE(sm.is_grasping());
}

TEST(GripperStateMachine, StartGraspEntersApproaching) {
    GripperStateMachine sm(test_params());
    sm.start_grasp({1.0f, 3.0f});
    EXPECT_EQ(sm.current_phase(), GripperPhase::APPROACHING);
}

TEST(GripperStateMachine, ReleaseReturnsIdle) {
    GripperStateMachine sm(test_params());
    sm.start_grasp({1.0f, 3.0f});
    sm.release();
    EXPECT_EQ(sm.current_phase(), GripperPhase::IDLE);
}

TEST(GripperStateMachine, ApproachingTimeout) {
    GripperStateMachine sm(test_params());
    sm.start_grasp({1.0f, 3.0f});

    // 每帧 10ms，累计超过 500ms 应超时回 IDLE
    for (int i = 0; i < 55; ++i) {
        sm.update(idle_state(0.0f), 10.0f);
    }
    EXPECT_EQ(sm.current_phase(), GripperPhase::IDLE);
}

TEST(GripperStateMachine, TauContactTriggersContacting) {
    GripperStateMachine sm(test_params());
    sm.start_grasp({1.0f, 3.0f});

    // 发一帧高 tau → 接触检测触发 → 进入 CONTACTING
    GripperMotorState s = {0.5f, 0.0f, 0.30f};  // tau=0.30 > 0.24
    sm.update(s, 5.0f);
    EXPECT_EQ(sm.current_phase(), GripperPhase::CONTACTING);
}

TEST(GripperStateMachine, GraspingAfterContact) {
    GripperStateMachine sm(test_params());
    sm.start_grasp({1.0f, 3.0f});

    // 先进入 CONTACTING（tau 超阈）
    GripperMotorState contact_state = {0.5f, 0.0f, 0.30f};
    sm.update(contact_state, 5.0f);
    EXPECT_EQ(sm.current_phase(), GripperPhase::CONTACTING);

    // CONTACTING 下 tau 持续 > 阈值 → 进入 GRASPING
    sm.update(contact_state, 5.0f);
    EXPECT_EQ(sm.current_phase(), GripperPhase::GRASPING);
    EXPECT_TRUE(sm.is_grasping());
}

TEST(GripperStateMachine, OverTorqueProtection) {
    GripperStateMachine sm(test_params());
    sm.start_grasp({1.0f, 2.0f});  // max_tau = 2.0

    // 强制进入 GRASPING 阶段
    GripperMotorState s = {0.5f, 0.0f, 0.30f};
    sm.update(s, 5.0f);  // APPROACHING → CONTACTING
    sm.update(s, 5.0f);  // CONTACTING → GRASPING

    // 发送超压状态（tau=2.5 > max_tau=2.0）
    GripperMotorState overload = {0.5f, 0.0f, 2.5f};
    sm.update(overload, 5.0f);
    EXPECT_EQ(sm.current_phase(), GripperPhase::IDLE);
}

TEST(GripperStateMachine, IdleCmdKpZero) {
    GripperStateMachine sm(test_params());
    auto cmd = sm.update(idle_state(), 5.0f);
    // IDLE 阶段 kp=0，纯阻尼
    EXPECT_NEAR(cmd.kp, 0.0f, 1e-6f);
    EXPECT_GT(cmd.kd, 0.0f);
}

TEST(GripperStateMachine, ApproachingCmdKp30) {
    GripperStateMachine sm(test_params());
    sm.start_grasp({1.0f, 3.0f});
    auto cmd = sm.update(idle_state(), 5.0f);
    EXPECT_NEAR(cmd.kp, 30.0f, 1e-4f);
    EXPECT_NEAR(cmd.q_des, 1.0f, 1e-6f);  // 指向目标
}
