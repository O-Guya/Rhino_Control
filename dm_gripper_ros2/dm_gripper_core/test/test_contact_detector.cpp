#include <gtest/gtest.h>
#include "dm_gripper_core/contact_detector.hpp"

using namespace dm_gripper_core;

// 使用小窗口方便测试
static ContactDetector::Params test_params() {
    ContactDetector::Params p;
    p.tau_threshold     = 0.24f;
    p.delta_q_threshold = 0.02f;
    p.min_q_error       = 0.05f;
    p.window_size       = 5;  // 测试时用小窗口
    return p;
}

TEST(ContactDetector, InitialNoContact) {
    ContactDetector det(test_params());
    // 初始状态无接触
    EXPECT_FALSE(det.is_contact());
    EXPECT_EQ(det.last_result(), ContactResult::NONE);
}

TEST(ContactDetector, TauTrigger) {
    ContactDetector det(test_params());
    // tau 超阈，立刻触发 TAU_ONLY
    auto r = det.update(1.0f, 1.0f, 0.30f);  // q_error=0, tau=0.30 > 0.24
    EXPECT_EQ(r, ContactResult::TAU_ONLY);
    EXPECT_TRUE(det.is_contact());
}

TEST(ContactDetector, NoTriggerBelowThreshold) {
    ContactDetector det(test_params());
    auto r = det.update(1.0f, 1.0f, 0.10f);  // tau 低于阈值，q_error=0
    EXPECT_EQ(r, ContactResult::NONE);
    EXPECT_FALSE(det.is_contact());
}

TEST(ContactDetector, PosTriggerWhenStuck) {
    ContactDetector det(test_params());
    // 模拟夹爪卡住：q_error 保持 0.5 rad 不减小，tau 低于阈值
    // 填满窗口
    for (int i = 0; i < 5; ++i) {
        det.update(1.5f, 1.0f, 0.10f);  // q_error 固定 = 0.5，不收敛
    }
    auto r = det.last_result();
    // q_error=0.5 > min=0.05，窗口内 delta < delta_q_threshold → 触发 pos 判据
    EXPECT_EQ(r, ContactResult::POS_ONLY);
    EXPECT_TRUE(det.is_contact());
}

TEST(ContactDetector, NoPosIfConverging) {
    ContactDetector det(test_params());
    // 模拟正在收敛：q_error 从 0.5 线性减小到 0.3（收敛 delta > 0.02）
    float errors[] = {0.50f, 0.45f, 0.40f, 0.35f, 0.30f};
    ContactResult r = ContactResult::NONE;
    for (float e : errors) {
        r = det.update(e, 0.0f, 0.10f);  // q_actual=0, q_cmd=e → q_error=e
    }
    // delta = 0.50 - 0.30 = 0.20 > 0.02，认为在收敛，不触发 pos 判据
    EXPECT_EQ(r, ContactResult::NONE);
}

TEST(ContactDetector, BothTrigger) {
    ContactDetector det(test_params());
    // 填满窗口：q_error 卡住 + tau 超阈
    for (int i = 0; i < 5; ++i) {
        det.update(1.5f, 1.0f, 0.30f);
    }
    EXPECT_EQ(det.last_result(), ContactResult::BOTH);
}

TEST(ContactDetector, ResetClearsState) {
    ContactDetector det(test_params());
    det.update(1.5f, 1.0f, 0.30f);
    EXPECT_TRUE(det.is_contact());

    det.reset();
    EXPECT_FALSE(det.is_contact());
    EXPECT_EQ(det.last_result(), ContactResult::NONE);
}

TEST(ContactDetector, SmallQErrorNoPosTrigger) {
    ContactDetector det(test_params());
    // q_error < min_q_error=0.05：认为已到位，不触发 pos 判据
    for (int i = 0; i < 5; ++i) {
        det.update(1.02f, 1.0f, 0.10f);  // q_error = 0.02 < 0.05
    }
    EXPECT_EQ(det.last_result(), ContactResult::NONE);
}
