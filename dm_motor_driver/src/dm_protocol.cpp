#include "dm_motor_driver/dm_protocol.hpp"

#include <cstring>

namespace dm_motor_driver {

// ── Quantization helpers ──────────────────────────────────────────────────────
// Source: demo/src/dm_socketcan_demo.cpp (float_to_uint / uint_to_float)

uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bits)
{
    float span = xmax - xmin;
    float norm = (x - xmin) / span;
    return static_cast<uint16_t>(norm * static_cast<float>((1 << bits) - 1));
}

float uint_to_float(uint16_t x, float xmin, float xmax, uint8_t bits)
{
    float span = xmax - xmin;
    float norm = static_cast<float>(x) / static_cast<float>((1 << bits) - 1);
    return norm * span + xmin;
}

// ── MIT frame pack ────────────────────────────────────────────────────────────
// Source: demo/src/dm_socketcan_demo.cpp (pack_mit_frame)

void pack_mit_frame(const MotorCommand& cmd, const MotorLimits& lim, uint8_t out[8])
{
    uint16_t q_uint   = float_to_uint(cmd.q,   -lim.q_max,   lim.q_max,   16);
    uint16_t dq_uint  = float_to_uint(cmd.dq,  -lim.dq_max,  lim.dq_max,  12);
    uint16_t kp_uint  = float_to_uint(cmd.kp,   0.0f,         lim.kp_max,  12);
    uint16_t kd_uint  = float_to_uint(cmd.kd,   0.0f,         lim.kd_max,  12);
    uint16_t tau_uint = float_to_uint(cmd.tau, -lim.tau_max,  lim.tau_max, 12);

    out[0] = (q_uint >> 8) & 0xFF;
    out[1] =  q_uint       & 0xFF;
    out[2] =  dq_uint >> 4;
    out[3] = static_cast<uint8_t>(((dq_uint  & 0xF) << 4) | ((kp_uint >> 8) & 0xF));
    out[4] =  kp_uint  & 0xFF;
    out[5] =  kd_uint  >> 4;
    out[6] = static_cast<uint8_t>(((kd_uint  & 0xF) << 4) | ((tau_uint >> 8) & 0xF));
    out[7] =  tau_uint & 0xFF;
}

// ── Feedback decode ───────────────────────────────────────────────────────────
// Source: demo/src/dm_socketcan_demo.cpp (decode_feedback) and main.cpp

bool decode_feedback(const uint8_t* payload, size_t len,
                     const MotorLimits& lim, MotorState& state)
{
    if (len < 8) return false;

    // byte[0] is status/error — ignored for now
    uint16_t q_uint   = (static_cast<uint16_t>(payload[1]) << 8) | payload[2];
    uint16_t dq_uint  = (static_cast<uint16_t>(payload[3]) << 4) | (payload[4] >> 4);
    uint16_t tau_uint = (static_cast<uint16_t>(payload[4] & 0xF) << 8) | payload[5];

    state.pos       = uint_to_float(q_uint,   -lim.q_max,   lim.q_max,   16);
    state.vel       = uint_to_float(dq_uint,  -lim.dq_max,  lim.dq_max,  12);
    state.tau       = uint_to_float(tau_uint, -lim.tau_max, lim.tau_max, 12);
    state.mos_temp  = static_cast<float>(payload[6]);
    state.rotor_temp= static_cast<float>(payload[7]);
    state.valid     = true;
    return true;
}

// ── Special command builders ───────────────────────────────────────────────────
// Source: demo/src/dm_socketcan_demo.cpp (send_enable / send_disable / send_switch_to_mit)

void build_enable_cmd(uint8_t out[8])
{
    // 0xFC trailer: enable command
    const uint8_t cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    memcpy(out, cmd, 8);
}

void build_disable_cmd(uint8_t out[8])
{
    // 0xFD trailer: disable command
    const uint8_t cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    memcpy(out, cmd, 8);
}

void build_switch_to_mit_cmd(uint32_t motor_can_id, uint8_t out[8])
{
    // Write register RID=10 (CTRL_MODE), value=1 (MIT), send on CAN ID 0x7FF
    // Source: damiao.cpp:471-492 and demo/src/dm_socketcan_demo.cpp
    out[0] = static_cast<uint8_t>(motor_can_id & 0xFF);
    out[1] = static_cast<uint8_t>((motor_can_id >> 8) & 0xFF);
    out[2] = 0x55;  // magic marker
    out[3] = 0x0A;  // RID = 10 (CTRL_MODE register)
    out[4] = 0x01;  // value = 1 = MIT_MODE
    out[5] = 0x00;
    out[6] = 0x00;
    out[7] = 0x00;
}

} // namespace dm_motor_driver
