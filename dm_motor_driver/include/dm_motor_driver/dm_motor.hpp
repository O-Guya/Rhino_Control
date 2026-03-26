#pragma once

#include <cstdint>

namespace dm_motor_driver {

// ── Motor type ────────────────────────────────────────────────────────────────

enum class MotorType {
    DM4310,       // 12.5 rad, 45 rad/s (CAN FD), 18 Nm
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
};

// ── Parameter limits (per motor type) ─────────────────────────────────────────

struct MotorLimits {
    float q_max;    // [rad]   position limit (±)
    float dq_max;   // [rad/s] velocity limit (±)
    float tau_max;  // [Nm]    torque limit (±)
    float kp_max;   // []      Kp upper bound for MIT mode
    float kd_max;   // [Nm·s/rad] Kd upper bound for MIT mode
};

/// Returns hard-coded limits for the given motor type.
MotorLimits get_limits(MotorType type);

// ── State (feedback from motor) ────────────────────────────────────────────────

struct MotorState {
    float    pos       = 0.0f;   // [rad]
    float    vel       = 0.0f;   // [rad/s]
    float    tau       = 0.0f;   // [Nm]
    float    mos_temp  = 0.0f;   // [°C]
    float    rotor_temp= 0.0f;   // [°C]
    bool     valid     = false;  // true after first successful feedback decode
    uint64_t stamp_us  = 0;      // monotonic timestamp [µs]
};

// ── Command (sent to motor) ────────────────────────────────────────────────────

struct MotorCommand {
    float kp  = 0.0f;   // position gain  [0, kp_max]
    float kd  = 0.5f;   // velocity gain  [0, kd_max]
    float q   = 0.0f;   // target position [rad]
    float dq  = 0.0f;   // target velocity [rad/s]
    float tau = 0.0f;   // feed-forward torque [Nm]
};

} // namespace dm_motor_driver
