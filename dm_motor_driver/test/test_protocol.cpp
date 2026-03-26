#include <gtest/gtest.h>
#include "dm_motor_driver/dm_protocol.hpp"
#include "dm_motor_driver/dm_motor.hpp"

using namespace dm_motor_driver;

// DM4310 limits used throughout tests
static const MotorLimits kLim = get_limits(MotorType::DM4310);

// ── float_to_uint / uint_to_float round-trip ─────────────────────────────────

TEST(Protocol, QuantizationMidpoint)
{
    // Midpoint should survive round-trip with minimal error
    float  x     = 0.0f;
    float  xmin  = -12.5f;
    float  xmax  =  12.5f;
    auto   u     = float_to_uint(x, xmin, xmax, 16);
    float  back  = uint_to_float(u, xmin, xmax, 16);
    EXPECT_NEAR(back, x, 0.001f);
}

TEST(Protocol, QuantizationMax)
{
    float xmax = 12.5f;
    auto  u    = float_to_uint(xmax, -xmax, xmax, 16);
    float back = uint_to_float(u, -xmax, xmax, 16);
    EXPECT_NEAR(back, xmax, 0.01f);
}

TEST(Protocol, QuantizationMin)
{
    float xmax = 12.5f;
    auto  u    = float_to_uint(-xmax, -xmax, xmax, 16);
    float back = uint_to_float(u, -xmax, xmax, 16);
    EXPECT_NEAR(back, -xmax, 0.01f);
}

// ── MIT frame pack / decode round-trip ───────────────────────────────────────

TEST(Protocol, MitFrameRoundTrip_Zero)
{
    MotorCommand cmd{};  // all zeros
    uint8_t buf[8] = {};
    pack_mit_frame(cmd, kLim, buf);

    MotorState state{};
    bool ok = decode_feedback(buf, 8, kLim, state);
    // decode_feedback treats buf as feedback (different layout), so this is
    // a structural test only — check that it doesn't crash and returns true.
    EXPECT_TRUE(ok);
}

TEST(Protocol, PackMitFrame_KnownValues)
{
    // Encode q=0, dq=0, kp=0, kd=0.5, tau=0
    MotorCommand cmd;
    cmd.q   = 0.0f;
    cmd.dq  = 0.0f;
    cmd.kp  = 0.0f;
    cmd.kd  = 0.5f;
    cmd.tau = 0.0f;

    uint8_t buf[8] = {};
    pack_mit_frame(cmd, kLim, buf);

    // q=0 → midpoint of 16-bit range → 0x7FFF → bytes [0x7F, 0xFF]
    EXPECT_EQ(buf[0], 0x7F);
    EXPECT_EQ(buf[1], 0xFF);
}

TEST(Protocol, DecodeFeedbackTooShort)
{
    uint8_t payload[5] = {};
    MotorState state{};
    EXPECT_FALSE(decode_feedback(payload, 5, kLim, state));
    EXPECT_FALSE(state.valid);
}

// ── Special command builders ──────────────────────────────────────────────────

TEST(Protocol, EnableCmd)
{
    uint8_t buf[8] = {};
    build_enable_cmd(buf);
    // Last byte must be 0xFC
    EXPECT_EQ(buf[7], 0xFC);
    for (int i = 0; i < 7; i++) EXPECT_EQ(buf[i], 0xFF);
}

TEST(Protocol, DisableCmd)
{
    uint8_t buf[8] = {};
    build_disable_cmd(buf);
    EXPECT_EQ(buf[7], 0xFD);
}

TEST(Protocol, SwitchToMitCmd)
{
    uint8_t buf[8] = {};
    build_switch_to_mit_cmd(0x01, buf);
    EXPECT_EQ(buf[0], 0x01);  // motor_can_id low byte
    EXPECT_EQ(buf[1], 0x00);  // motor_can_id high byte
    EXPECT_EQ(buf[2], 0x55);  // magic
    EXPECT_EQ(buf[3], 0x0A);  // CTRL_MODE register (RID=10)
    EXPECT_EQ(buf[4], 0x01);  // MIT mode value
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
