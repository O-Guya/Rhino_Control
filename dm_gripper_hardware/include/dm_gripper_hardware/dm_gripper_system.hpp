#pragma once

#include "dm_motor_driver/dm_motor.hpp"
#include "dm_motor_driver/socketcan.hpp"
#include "dm_motor_driver/dm_protocol.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <memory>
#include <string>
#include <vector>

namespace dm_gripper_hardware {

/// ros2_control SystemInterface for the DM4310 screw-drive gripper.
///
/// Exposes:
///   State interfaces:  position [rad], effort [Nm]
///   Command interfaces: position [rad]
///
/// The hardware parameters are read from the URDF <hardware> tag:
///   <param name="can_interface">can0</param>
///   <param name="motor_can_id">1</param>
///   <param name="motor_mst_id">0</param>
///   <param name="use_canfd">true</param>
class DmGripperSystem : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DmGripperSystem)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Parameters from URDF
    std::string can_iface_{"can0"};
    uint32_t    motor_can_id_{0x01};
    uint32_t    motor_mst_id_{0x00};
    bool        use_canfd_{true};

    // Hardware
    std::unique_ptr<dm_motor_driver::SocketCAN> can_;
    dm_motor_driver::MotorLimits                limits_;
    dm_motor_driver::MotorState                 motor_state_;

    // ros2_control interface buffers
    double hw_pos_cmd_   {0.0};
    double hw_pos_state_ {0.0};
    double hw_eff_state_ {0.0};
};

} // namespace dm_gripper_hardware
