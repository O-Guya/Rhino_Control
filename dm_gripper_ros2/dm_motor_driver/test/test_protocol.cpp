/**
 * test_protocol.cpp
 *
 * dm_protocol 单元测试。
 * 不需要真实硬件，纯数学验证。
 *
 * 测试策略：
 *   1. 往返精度：float → pack → unpack → float，误差 ≤ 量化分辨率
 *   2. 边界值：±max 不溢出，超出范围被钳位
 *   3. 特殊命令帧：字节内容正确
 *   4. 寄存器读写帧：格式正确
 */

#include <gtest/gtest.h>
#include "dm_motor_driver/dm_protocol.hpp"

#include <cmath>
#include <cstring>

using namespace dm_motor_driver;

// ─────────────────────────────────────────────────────────────
// 测试辅助
// ─────────────────────────────────────────────────────────────

// 计算 n-bit 量化的理论分辨率（一个 LSB 对应的物理量）
static float resolution(float range_min, float range_max, int bits)
{
    return (range_max - range_min) / static_cast<float>((1 << bits) - 1);
}

// ─────────────────────────────────────────────────────────────
// 量化函数测试
// ─────────────────────────────────────────────────────────────

TEST(FloatUintConversion, RoundTrip16bit)
{
    // 16-bit 位置量化往返精度
    const float xmin = -12.5f, xmax = 12.5f;
    const int   bits = 16;
    const float res  = resolution(xmin, xmax, bits);

    for (float x : {-12.5f, -6.0f, 0.0f, 3.14f, 6.0f, 12.5f}) {
        uint16_t u = float_to_uint(x, xmin, xmax, bits);
        float    r = uint_to_float(u, xmin, xmax, bits);
        // 往返误差应在 1 个 LSB 以内
        EXPECT_NEAR(r, x, res * 1.01f)
            << "x=" << x << " u=" << u << " r=" << r;
    }
}

TEST(FloatUintConversion, RoundTrip12bit)
{
    // 12-bit 速度/力矩量化往返精度
    const float xmin = -30.0f, xmax = 30.0f;
    const int   bits = 12;
    const float res  = resolution(xmin, xmax, bits);

    for (float x : {-30.0f, -15.0f, 0.0f, 10.0f, 30.0f}) {
        uint16_t u = float_to_uint(x, xmin, xmax, bits);
        float    r = uint_to_float(u, xmin, xmax, bits);
        EXPECT_NEAR(r, x, res * 1.01f)
            << "x=" << x << " u=" << u << " r=" << r;
    }
}

TEST(FloatUintConversion, Clamping)
{
    // 超出范围时钳位，不应溢出
    const float xmin = -10.0f, xmax = 10.0f;
    const int   bits = 12;
    const uint16_t max_val = (1 << bits) - 1;

    EXPECT_EQ(float_to_uint(20.0f, xmin, xmax, bits), max_val)  // 正向超出
        << "正向超范围应钳位到最大值";
    EXPECT_EQ(float_to_uint(-20.0f, xmin, xmax, bits), 0)        // 负向超出
        << "负向超范围应钳位到 0";
}

TEST(FloatUintConversion, ZeroMapsToMidpoint)
{
    // 对称范围时，0.0 应映射到接近中间值
    const float xmin = -12.5f, xmax = 12.5f;
    uint16_t u = float_to_uint(0.0f, xmin, xmax, 16);
    // 中间值 = 32767 或 32768（因为 (2^16-1) = 65535 是奇数）
    EXPECT_NEAR(static_cast<float>(u), 65535.0f / 2.0f, 1.0f);
}

// ─────────────────────────────────────────────────────────────
// MIT 帧打包测试
// ─────────────────────────────────────────────────────────────

TEST(PackMitFrame, ZeroCommandProducesValidBytes)
{
    // 零命令（kp=0, kd=0, q=0, dq=0, tau=0）时帧应有确定值
    MotorCommand cmd;  // 全零（kd 默认 0.5，这里明确设为 0）
    cmd.kd = 0.0f;
    MotorLimits  limits;
    uint8_t      out[8] = {};

    pack_mit_frame(cmd, limits, out);

    // q=0 在 [-12.5, 12.5] 中映射到约 32767（16-bit 中点）
    uint16_t q_u = (static_cast<uint16_t>(out[0]) << 8) | out[1];
    EXPECT_NEAR(static_cast<float>(q_u), 32767.0f, 2.0f)
        << "q=0 应映射到 16-bit 中点";
}

TEST(PackMitFrame, RoundTripAllFields)
{
    // 打包后解包，所有字段往返误差应在量化精度内
    MotorLimits limits;
    MotorCommand cmd;
    cmd.q_des  = 3.0f;
    cmd.dq_des = 5.0f;
    cmd.kp     = 30.0f;
    cmd.kd     = 1.5f;
    cmd.tau_ff = 2.0f;

    uint8_t out[8] = {};
    pack_mit_frame(cmd, limits, out);

    // 手动解包（与 unpack_feedback 逻辑不同，这里解包命令帧）
    uint16_t q_u   = (static_cast<uint16_t>(out[0]) << 8) | out[1];
    uint16_t dq_u  = (static_cast<uint16_t>(out[2]) << 4) | (out[3] >> 4);
    uint16_t kp_u  = (static_cast<uint16_t>(out[3] & 0x0F) << 8) | out[4];
    uint16_t kd_u  = (static_cast<uint16_t>(out[5]) << 4) | (out[6] >> 4);
    uint16_t tau_u = (static_cast<uint16_t>(out[6] & 0x0F) << 8) | out[7];

    float q_r   = uint_to_float(q_u,   -limits.q_max,   limits.q_max,   16);
    float dq_r  = uint_to_float(dq_u,  -limits.dq_max,  limits.dq_max,  12);
    float kp_r  = uint_to_float(kp_u,   MotorLimits::kp_min, MotorLimits::kp_max, 12);
    float kd_r  = uint_to_float(kd_u,   MotorLimits::kd_min, MotorLimits::kd_max, 12);
    float tau_r = uint_to_float(tau_u, -limits.tau_max, limits.tau_max,  12);

    EXPECT_NEAR(q_r,   cmd.q_des,  resolution(-limits.q_max,   limits.q_max,   16) * 1.1f);
    EXPECT_NEAR(dq_r,  cmd.dq_des, resolution(-limits.dq_max,  limits.dq_max,  12) * 1.1f);
    EXPECT_NEAR(kp_r,  cmd.kp,     resolution(0.0f, 500.0f, 12) * 1.1f);
    EXPECT_NEAR(kd_r,  cmd.kd,     resolution(0.0f, 5.0f,   12) * 1.1f);
    EXPECT_NEAR(tau_r, cmd.tau_ff, resolution(-limits.tau_max, limits.tau_max,  12) * 1.1f);
}

// ─────────────────────────────────────────────────────────────
// 反馈帧解析测试
// ─────────────────────────────────────────────────────────────

TEST(UnpackFeedback, KnownBytesProduceCorrectValues)
{
    // 构造一个已知内容的反馈帧，验证解析结果
    MotorLimits limits;

    // 手动构造：id=1, err=1(使能), pos=0, vel=0, tau=0, Tmos=30, Tcoil=25
    uint16_t q_u   = float_to_uint(0.0f, -limits.q_max,   limits.q_max,   16);
    uint16_t dq_u  = float_to_uint(0.0f, -limits.dq_max,  limits.dq_max,  12);
    uint16_t tau_u = float_to_uint(0.0f, -limits.tau_max,  limits.tau_max, 12);

    uint8_t data[8] = {};
    data[0] = (1 << 4) | 1;  // err=1(使能), id=1
    data[1] = (q_u >> 8) & 0xFF;
    data[2] =  q_u       & 0xFF;
    data[3] =  dq_u >> 4;
    data[4] = ((dq_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F);
    data[5] =  tau_u & 0xFF;
    data[6] = 30;   // Tmos
    data[7] = 25;   // Tcoil

    MotorState state = unpack_feedback(data, limits);

    EXPECT_TRUE(state.valid);
    EXPECT_EQ(state.motor_id, 1);
    EXPECT_EQ(state.err_code, 1);
    EXPECT_TRUE(state.is_enabled());
    EXPECT_FALSE(state.has_error());
    EXPECT_NEAR(state.pos,   0.0f, 0.01f);
    EXPECT_NEAR(state.vel,   0.0f, 0.1f);
    EXPECT_NEAR(state.tau,   0.0f, 0.1f);
    EXPECT_FLOAT_EQ(state.Tmos,  30.0f);
    EXPECT_FLOAT_EQ(state.Tcoil, 25.0f);
}

TEST(UnpackFeedback, PositivePosition)
{
    // 正位置值解析
    MotorLimits limits;
    float target_pos = 3.14f;

    uint16_t q_u = float_to_uint(target_pos, -limits.q_max, limits.q_max, 16);

    uint8_t data[8] = {};
    data[0] = (1 << 4) | 1;  // 使能
    data[1] = (q_u >> 8) & 0xFF;
    data[2] =  q_u       & 0xFF;
    // 其余字段为 0

    MotorState state = unpack_feedback(data, limits);
    float res = resolution(-limits.q_max, limits.q_max, 16);
    EXPECT_NEAR(state.pos, target_pos, res * 1.1f);
}

// ─────────────────────────────────────────────────────────────
// 特殊命令帧测试
// ─────────────────────────────────────────────────────────────

TEST(SpecialPayloads, EnablePayload)
{
    uint8_t out[8] = {};
    make_enable_payload(out);

    for (int i = 0; i < 7; ++i)
        EXPECT_EQ(out[i], 0xFF) << "使能帧前7字节应为 0xFF，index=" << i;
    EXPECT_EQ(out[7], 0xFC) << "使能帧最后一字节应为 0xFC";
}

TEST(SpecialPayloads, DisablePayload)
{
    uint8_t out[8] = {};
    make_disable_payload(out);

    for (int i = 0; i < 7; ++i)
        EXPECT_EQ(out[i], 0xFF) << "失能帧前7字节应为 0xFF，index=" << i;
    EXPECT_EQ(out[7], 0xFD) << "失能帧最后一字节应为 0xFD";
}

TEST(SpecialPayloads, SetZeroPayload)
{
    uint8_t out[8] = {};
    make_set_zero_payload(out);
    EXPECT_EQ(out[7], 0xFE);
}

TEST(SpecialPayloads, ClearErrPayload)
{
    uint8_t out[8] = {};
    make_clear_err_payload(out);
    EXPECT_EQ(out[7], 0xFB);
}

// ─────────────────────────────────────────────────────────────
// 寄存器读写帧测试
// ─────────────────────────────────────────────────────────────

TEST(RegisterPayload, WriteRegFormat)
{
    uint8_t out[8] = {};
    // 对 ID=1 的电机写入寄存器 0x0A（控制模式），值 = 1（MIT 模式）
    make_write_reg_payload(0x0001, 0x0A, 1u, out);

    EXPECT_EQ(out[0], 0x01) << "CANID_L 应为 0x01";
    EXPECT_EQ(out[1], 0x00) << "CANID_H 应为 0x00（标准帧）";
    EXPECT_EQ(out[2], 0x55) << "写寄存器指令标识应为 0x55";
    EXPECT_EQ(out[3], 0x0A) << "寄存器地址 RID 应为 0x0A";
    EXPECT_EQ(out[4], 0x01) << "值低字节应为 0x01";
    EXPECT_EQ(out[5], 0x00);
    EXPECT_EQ(out[6], 0x00);
    EXPECT_EQ(out[7], 0x00);
}

TEST(RegisterPayload, ReadRegFormat)
{
    uint8_t out[4] = {};
    make_read_reg_payload(0x0001, 0x15, out);  // 读 PMAX 寄存器

    EXPECT_EQ(out[0], 0x01);
    EXPECT_EQ(out[1], 0x00);
    EXPECT_EQ(out[2], 0x33) << "读寄存器指令标识应为 0x33";
    EXPECT_EQ(out[3], 0x15) << "寄存器地址应为 0x15";
}

// ─────────────────────────────────────────────────────────────
// MotorState 辅助函数测试
// ─────────────────────────────────────────────────────────────

TEST(MotorState, ErrorStringMapping)
{
    MotorState s;
    s.err_code = 0;  EXPECT_EQ(s.err_str(), "DISABLED");
    s.err_code = 1;  EXPECT_EQ(s.err_str(), "ENABLED");
    s.err_code = 9;  EXPECT_EQ(s.err_str(), "UNDER_VOLTAGE");
    s.err_code = 0xE; EXPECT_EQ(s.err_str(), "OVERLOAD");
}

TEST(MotorState, EnabledAndErrorFlags)
{
    MotorState s;
    s.err_code = 1;
    EXPECT_TRUE(s.is_enabled());
    EXPECT_FALSE(s.has_error());

    s.err_code = 0xA;
    EXPECT_FALSE(s.is_enabled());
    EXPECT_TRUE(s.has_error());
}