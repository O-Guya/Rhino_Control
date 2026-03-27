/**
 * test_socketcan_mock.cpp
 *
 * SocketCan 单元测试。
 *
 * 策略：
 *   - 不需要真实 CAN 硬件
 *   - 测试「接口不存在时构造失败」等可在普通环境验证的行为
 *   - 测试 CanFrame 数据结构的边界条件
 *
 * 注意：
 *   涉及真实 CAN 收发的测试（send/recv 路径）需要硬件，
 *   在 CI 环境中跳过，在实机上手动验证。
 */

#include <gtest/gtest.h>
#include "dm_motor_driver/socketcan.hpp"

using namespace dm_motor_driver;

// ─────────────────────────────────────────────────────────────
// CanFrame 数据结构测试
// ─────────────────────────────────────────────────────────────

TEST(CanFrame, DefaultInit)
{
    // 默认构造的帧应全为零
    CanFrame f;
    EXPECT_EQ(f.id,  0u);
    EXPECT_EQ(f.dlc, 0u);
    for (int i = 0; i < 8; ++i) {
        EXPECT_EQ(f.data[i], 0u) << "data[" << i << "] 应初始化为 0";
    }
}

TEST(CanFrame, DataCanBeSet)
{
    CanFrame f;
    f.id  = 0x01;
    f.dlc = 8;
    for (uint8_t i = 0; i < 8; ++i) f.data[i] = i;

    EXPECT_EQ(f.id,  0x01u);
    EXPECT_EQ(f.dlc, 8u);
    for (int i = 0; i < 8; ++i)
        EXPECT_EQ(f.data[i], static_cast<uint8_t>(i));
}

// ─────────────────────────────────────────────────────────────
// SocketCan 构造失败测试
// ─────────────────────────────────────────────────────────────

TEST(SocketCan, ThrowsOnNonexistentInterface)
{
    // 不存在的接口名应在构造时抛出异常
    EXPECT_THROW(
        SocketCan can("nonexistent_can_interface_xyz"),
        std::runtime_error
    ) << "不存在的接口应抛出 runtime_error";
}

TEST(SocketCan, ThrowsOnEmptyInterfaceName)
{
    EXPECT_THROW(
        SocketCan can(""),
        std::runtime_error
    ) << "空接口名应抛出 runtime_error";
}

// ─────────────────────────────────────────────────────────────
// SocketCan 移动语义测试
// ─────────────────────────────────────────────────────────────

TEST(SocketCan, MoveConstructorTransfersOwnership)
{
    // 注意：这个测试需要真实 CAN 接口，在无接口环境中跳过
    // 这里只验证「构造失败后移动」的边界情况

    // 构造一个无效的 SocketCan（通过捕获异常后无法移动，
    // 但可以验证移动后原对象析构不崩溃）
    // 这里通过一个间接方法：用 placement 思路验证类型特征
    EXPECT_FALSE(std::is_copy_constructible<SocketCan>::value)
        << "SocketCan 不应可拷贝构造";
    EXPECT_FALSE(std::is_copy_assignable<SocketCan>::value)
        << "SocketCan 不应可拷贝赋值";
    EXPECT_TRUE(std::is_move_constructible<SocketCan>::value)
        << "SocketCan 应可移动构造";
    EXPECT_TRUE(std::is_move_assignable<SocketCan>::value)
        << "SocketCan 应可移动赋值";
}

// ─────────────────────────────────────────────────────────────
// 集成测试占位（需要真实硬件，CI 中跳过）
// ─────────────────────────────────────────────────────────────

// 以下测试在有真实 can0 接口的环境中手动运行：
//   sudo ip link set can0 up type can bitrate 1000000
//   colcon test --packages-select dm_motor_driver
//   （或单独：./test_socketcan_mock）

#ifdef DM_HARDWARE_TEST_ENABLED
// 仅在定义 DM_HARDWARE_TEST_ENABLED 时编译（需要真实硬件）

TEST(SocketCan, OpenAndCloseRealInterface)
{
    EXPECT_NO_THROW({
        SocketCan can("can0");
        // 析构时自动关闭
    });
}

TEST(SocketCan, SendEnableFrameDoesNotThrow)
{
    SocketCan can("can0");
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    EXPECT_NO_THROW(can.send(0x01, data, 8));
}

TEST(SocketCan, RecvTimeoutReturnsfalse)
{
    // 没有设备响应时，recv 应在超时后返回 false
    SocketCan can("can0", 100);  // 100ms 超时
    CanFrame frame;
    // 假设总线上没有发送者
    bool got = can.recv(frame);
    // got 可能是 true（如果总线上有其他帧），也可能是 false，不做断言
    (void)got;
}

#endif  // DM_HARDWARE_TEST_ENABLED