#pragma once

#include "dm_motor.hpp"
#include <cstdint>
#include <cstddef>

namespace dm_motor_driver {

// ── Quantization helpers ──────────────────────────────────────────────────────

/// Linear float → unsigned fixed-point (bits wide).
uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bits);

/// Linear unsigned fixed-point → float.
float uint_to_float(uint16_t x, float xmin, float xmax, uint8_t bits);

// ── MIT-mode frame encode/decode ───────────────────────────────────────────────

/// Pack a MotorCommand into the 8-byte MIT-mode CAN payload.
/// out must point to at least 8 bytes.
///
/// Layout (from Damiao protocol):
///   [0]   q_uint[15:8]
///   [1]   q_uint[7:0]
///   [2]   dq_uint[11:4]
///   [3]   dq_uint[3:0] | kp_uint[11:8]
///   [4]   kp_uint[7:0]
///   [5]   kd_uint[11:4]
///   [6]   kd_uint[3:0] | tau_uint[11:8]
///   [7]   tau_uint[7:0]
void pack_mit_frame(const MotorCommand& cmd, const MotorLimits& lim, uint8_t out[8]);

/// Decode an 8-byte MIT-mode feedback payload into MotorState.
/// Returns false if payload is too short (< 8 bytes).
///
/// Feedback layout (from Damiao protocol):
///   [0]   status/error byte (ignored)
///   [1-2] position (16-bit)
///   [3-4] velocity (12-bit, spanning bytes)
///   [4-5] torque   (12-bit, spanning bytes)
///   [6]   MOS temperature [°C]
///   [7]   rotor temperature [°C]
bool decode_feedback(const uint8_t* payload, size_t len,
                     const MotorLimits& lim, MotorState& state);

// ── Special command builders ───────────────────────────────────────────────────

/// Fill out[8] with the motor enable command (0xFC trailer).
void build_enable_cmd(uint8_t out[8]);

/// Fill out[8] with the motor disable command (0xFD trailer).
void build_disable_cmd(uint8_t out[8]);

/// Fill out[8] with the register-write payload to switch motor to MIT mode.
/// Send on CAN ID 0x7FF.
void build_switch_to_mit_cmd(uint32_t motor_can_id, uint8_t out[8]);

} // namespace dm_motor_driver
