/**
 * dm_protocol.cpp
 *
 * 达妙电机 MIT 协议的打包/解包实现。
 * 所有函数均为纯函数（无全局状态），可独立单元测试。
 */

#include "dm_motor_driver/dm_protocol.hpp"

#include <cstring>
#include <cmath>

namespace dm_motor_driver {

// ─────────────────────────────────────────────────────────────
// 量化辅助
// ─────────────────────────────────────────────────────────────

uint16_t float_to_uint(float x, float xmin, float xmax, int bits)
{
    float span = xmax - xmin;
    float norm = (x - xmin) / span;

    // 钳位到 [0, 1]，防止溢出
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    // 映射到 [0, 2^bits - 1]
    return static_cast<uint16_t>(norm * static_cast<float>((1 << bits) - 1));
}

float uint_to_float(uint16_t x, float xmin, float xmax, int bits)
{
    float span = xmax - xmin;
    float norm = static_cast<float>(x) /
                 static_cast<float>((1 << bits) - 1);
    return norm * span + xmin;
}

// ─────────────────────────────────────────────────────────────
// MIT 帧打包
// ─────────────────────────────────────────────────────────────

void pack_mit_frame(const MotorCommand& cmd,
                    const MotorLimits& limits,
                    uint8_t out[8])
{
    // 将各物理量量化为定点整数
    //   q_des  → 16-bit（精度最高，位置控制需要）
    //   其余   → 12-bit
    uint16_t q_u   = float_to_uint(cmd.q_des,
                                   -limits.q_max,   limits.q_max,   16);
    uint16_t dq_u  = float_to_uint(cmd.dq_des,
                                   -limits.dq_max,  limits.dq_max,  12);
    uint16_t kp_u  = float_to_uint(cmd.kp,
                                   MotorLimits::kp_min, MotorLimits::kp_max, 12);
    uint16_t kd_u  = float_to_uint(cmd.kd,
                                   MotorLimits::kd_min, MotorLimits::kd_max, 12);
    uint16_t tau_u = float_to_uint(cmd.tau_ff,
                                   -limits.tau_max, limits.tau_max, 12);

    // 按位打包进 8 字节
    // 布局见头文件注释，每个字段的位按大端序排列
    out[0] = (q_u >> 8) & 0xFF;                          // q[15:8]
    out[1] =  q_u       & 0xFF;                          // q[7:0]
    out[2] =  dq_u >> 4;                                 // dq[11:4]
    out[3] = ((dq_u  & 0x0F) << 4) | ((kp_u >> 8) & 0x0F); // dq[3:0] | kp[11:8]
    out[4] =  kp_u  & 0xFF;                              // kp[7:0]
    out[5] =  kd_u  >> 4;                                // kd[11:4]
    out[6] = ((kd_u  & 0x0F) << 4) | ((tau_u >> 8) & 0x0F); // kd[3:0] | tau[11:8]
    out[7] =  tau_u & 0xFF;                              // tau[7:0]
}

// ─────────────────────────────────────────────────────────────
// 反馈帧解析
// ─────────────────────────────────────────────────────────────

MotorState unpack_feedback(const uint8_t data[8], const MotorLimits& limits)
{
    MotorState state;

    // D[0]：低 4 位 = 电机 ID，高 4 位 = 错误码
    state.motor_id = data[0] & 0x0F;
    state.err_code = (data[0] >> 4) & 0x0F;

    // D[1:2]：位置，16-bit 大端
    uint16_t q_u = (static_cast<uint16_t>(data[1]) << 8) | data[2];

    // D[3] 高4位 + D[4] 高4位 = 速度，12-bit
    uint16_t dq_u = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);

    // D[4] 低4位 + D[5] = 力矩，12-bit
    uint16_t tau_u = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];

    // 反量化
    state.pos = uint_to_float(q_u,   -limits.q_max,  limits.q_max,  16);
    state.vel = uint_to_float(dq_u,  -limits.dq_max, limits.dq_max, 12);
    state.tau = uint_to_float(tau_u, -limits.tau_max, limits.tau_max, 12);

    // D[6:7]：温度，直接用整数值（单位 °C）
    state.Tmos  = static_cast<float>(data[6]);
    state.Tcoil = static_cast<float>(data[7]);

    state.valid = true;
    return state;
}

// ─────────────────────────────────────────────────────────────
// 特殊命令载荷生成
// ─────────────────────────────────────────────────────────────

// 辅助：填充全 FF 前缀
static void fill_ff(uint8_t out[8])
{
    for (int i = 0; i < 7; ++i) out[i] = 0xFF;
}

void make_enable_payload(uint8_t out[8])
{
    fill_ff(out);
    out[7] = 0xFC;  // 使能标志
}

void make_disable_payload(uint8_t out[8])
{
    fill_ff(out);
    out[7] = 0xFD;  // 失能标志
}

void make_set_zero_payload(uint8_t out[8])
{
    fill_ff(out);
    out[7] = 0xFE;  // 设置零点标志
}

void make_clear_err_payload(uint8_t out[8])
{
    fill_ff(out);
    out[7] = 0xFB;  // 清除错误标志
}

// ─────────────────────────────────────────────────────────────
// 寄存器读写载荷
// ─────────────────────────────────────────────────────────────

void make_write_reg_payload(uint16_t motor_can_id, uint8_t rid,
                            uint32_t value, uint8_t out[8])
{
    // 格式：[CANID_L, CANID_H, 0x55, RID, d0, d1, d2, d3]
    // CANID_L/H：目标电机 CAN ID 拆分，让总线上其他电机忽略此帧
    out[0] = static_cast<uint8_t>(motor_can_id & 0xFF);         // CANID 低 8 位
    out[1] = static_cast<uint8_t>((motor_can_id >> 8) & 0x07);  // CANID 高 3 位
    out[2] = 0x55;  // 写寄存器指令标识
    out[3] = rid;   // 目标寄存器地址

    // 数据低位在前（小端序）
    out[4] = static_cast<uint8_t>(value & 0xFF);
    out[5] = static_cast<uint8_t>((value >>  8) & 0xFF);
    out[6] = static_cast<uint8_t>((value >> 16) & 0xFF);
    out[7] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

void make_read_reg_payload(uint16_t motor_can_id, uint8_t rid,
                           uint8_t out[4])
{
    // 格式：[CANID_L, CANID_H, 0x33, RID]
    out[0] = static_cast<uint8_t>(motor_can_id & 0xFF);
    out[1] = static_cast<uint8_t>((motor_can_id >> 8) & 0x07);
    out[2] = 0x33;  // 读寄存器指令标识
    out[3] = rid;
}

// ─────────────────────────────────────────────────────────────
// MotorState 辅助
// ─────────────────────────────────────────────────────────────

std::string MotorState::err_str() const
{
    switch (err_code) {
        case 0:  return "DISABLED";
        case 1:  return "ENABLED";
        case 2:  return "ENCODER_NOT_CALIBRATED";
        case 8:  return "OVER_VOLTAGE";
        case 9:  return "UNDER_VOLTAGE";
        case 0xA: return "OVER_CURRENT";
        case 0xB: return "MOS_OVER_TEMP";
        case 0xC: return "COIL_OVER_TEMP";
        case 0xD: return "COMM_LOST";
        case 0xE: return "OVERLOAD";
        default:  return "UNKNOWN(" + std::to_string(err_code) + ")";
    }
}

}  // namespace dm_motor_driver