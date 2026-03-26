#include <gtest/gtest.h>
#include "dm_gripper_core/contact_detector.hpp"

using namespace dm_gripper_core;

static ContactDetector::Params make_params()
{
    return {
        .tau_threshold        = 2.0f,
        .pos_stuck_threshold  = 0.001f,
        .window_size          = 5
    };
}

TEST(ContactDetector, NoContactOnMotion)
{
    ContactDetector det(make_params());
    float pos = 0.0f;
    for (int i = 0; i < 20; i++) {
        pos += 0.01f;  // moving → not stuck
        bool c = det.update(pos, 0.0f);
        EXPECT_FALSE(c) << "i=" << i;
    }
}

TEST(ContactDetector, NoContactLowTorque)
{
    ContactDetector det(make_params());
    // Position stuck but torque below threshold
    for (int i = 0; i < 20; i++) {
        bool c = det.update(0.5f, 0.5f);  // stuck, but tau=0.5 < 2.0
        EXPECT_FALSE(c) << "i=" << i;
    }
}

TEST(ContactDetector, ContactOnStuckHighTorque)
{
    ContactDetector det(make_params());
    // Fill window with stuck + high-torque samples
    bool contact = false;
    for (int i = 0; i < 10; i++) {
        contact = det.update(1.0f, 3.0f);  // stuck, tau=3.0 > 2.0
    }
    EXPECT_TRUE(contact);
}

TEST(ContactDetector, ResetClearsContact)
{
    ContactDetector det(make_params());
    for (int i = 0; i < 10; i++) det.update(1.0f, 3.0f);
    EXPECT_TRUE(det.in_contact());
    det.reset();
    EXPECT_FALSE(det.in_contact());
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
