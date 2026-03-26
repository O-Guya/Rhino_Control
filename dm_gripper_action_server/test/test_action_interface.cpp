#include <gtest/gtest.h>

// Structural compile-check only.
// End-to-end action tests require a live ROS 2 environment and are run
// separately as integration tests.

TEST(ActionInterface, CompileCheck)
{
    // If this compiles and links, the test passes.
    SUCCEED();
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
