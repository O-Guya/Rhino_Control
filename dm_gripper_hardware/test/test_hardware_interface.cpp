#include <gtest/gtest.h>

// Structural compile-check for the hardware interface.
// Full integration tests require a live CAN bus and are run separately.

#include "dm_gripper_hardware/dm_gripper_system.hpp"

TEST(HardwareInterface, ClassInstantiates)
{
    // Verify the class can be constructed (pluginlib will do the same)
    dm_gripper_hardware::DmGripperSystem hw;
    (void)hw;
    SUCCEED();
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
