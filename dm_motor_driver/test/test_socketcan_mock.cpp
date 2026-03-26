#include <gtest/gtest.h>
#include "dm_motor_driver/socketcan.hpp"

using namespace dm_motor_driver;

// These tests do NOT require hardware. They verify the class compiles and
// that error paths work correctly when no CAN interface is available.

TEST(SocketCAN, DefaultNotOpen)
{
    SocketCAN can("can0_nonexistent", true);
    EXPECT_FALSE(can.is_open());
}

TEST(SocketCAN, OpenFailsOnNonexistentIface)
{
    SocketCAN can("can_does_not_exist_xyz", true);
    // open() should fail gracefully, not crash
    bool result = can.open();
    EXPECT_FALSE(result);
    EXPECT_FALSE(can.is_open());
}

TEST(SocketCAN, SendFailsWhenNotOpen)
{
    SocketCAN can("can0", true);
    // Don't call open() — send should return false
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    bool ok = can.send(0x01, data, 8);
    EXPECT_FALSE(ok);
}

TEST(SocketCAN, RecvFailsWhenNotOpen)
{
    SocketCAN can("can0", true);
    uint32_t id  = 0;
    uint8_t  buf[64] = {};
    uint8_t  len = 0;
    bool ok = can.recv(id, buf, len, 0);
    EXPECT_FALSE(ok);
}

TEST(SocketCAN, CloseIdempotent)
{
    SocketCAN can("can0", true);
    // Closing without opening should not crash
    can.close();
    can.close();
    EXPECT_FALSE(can.is_open());
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
