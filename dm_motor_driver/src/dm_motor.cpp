#include "dm_motor_driver/dm_motor.hpp"

namespace dm_motor_driver {

MotorLimits get_limits(MotorType type)
{
    switch (type) {
        case MotorType::DM4310:
            // Source: demo/src/dm_socketcan_demo.cpp and Damiao datasheet
            return { .q_max = 12.5f, .dq_max = 45.0f, .tau_max = 18.0f,
                     .kp_max = 500.0f, .kd_max = 5.0f };

        case MotorType::DM4310_48V:
            return { .q_max = 12.5f, .dq_max = 50.0f, .tau_max = 18.0f,
                     .kp_max = 500.0f, .kd_max = 5.0f };

        case MotorType::DM4340:
            return { .q_max = 12.5f, .dq_max = 10.0f, .tau_max = 28.0f,
                     .kp_max = 500.0f, .kd_max = 5.0f };

        case MotorType::DM4340_48V:
            return { .q_max = 12.5f, .dq_max = 10.0f, .tau_max = 28.0f,
                     .kp_max = 500.0f, .kd_max = 5.0f };

        case MotorType::DM6006:
            return { .q_max = 12.5f, .dq_max = 45.0f, .tau_max = 20.0f,
                     .kp_max = 500.0f, .kd_max = 5.0f };

        case MotorType::DM8006:
            return { .q_max = 12.5f, .dq_max = 45.0f, .tau_max = 54.0f,
                     .kp_max = 500.0f, .kd_max = 5.0f };

        case MotorType::DM8009:
            return { .q_max = 12.5f, .dq_max = 45.0f, .tau_max = 54.0f,
                     .kp_max = 500.0f, .kd_max = 5.0f };

        default:
            // Safe fallback — conservative limits
            return { .q_max = 12.5f, .dq_max = 10.0f, .tau_max = 5.0f,
                     .kp_max = 500.0f, .kd_max = 5.0f };
    }
}

} // namespace dm_motor_driver
