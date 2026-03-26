#include <gtest/gtest.h>
// 打包/解包单元测试（不需要硬件）
// TODO: add tests

TEST(DmProtocolTest, placeholder) { SUCCEED(); }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
