#include "dm_gripper_hardware/dm_gripper_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdlib>
#include <chrono>
#include <thread>

namespace dm_gripper_hardware {

static const char* kLogger = "DmGripperSystem";

hardware_interface::CallbackReturn DmGripperSystem::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Read parameters from URDF <hardware> block
    try {
        can_iface_     = info.hardware_parameters.at("can_interface");
        motor_can_id_  = static_cast<uint32_t>(
                             std::stoul(info.hardware_parameters.at("motor_can_id"), nullptr, 0));
        motor_mst_id_  = static_cast<uint32_t>(
                             std::stoul(info.hardware_parameters.at("motor_mst_id"), nullptr, 0));
        use_canfd_     = (info.hardware_parameters.at("use_canfd") == "true");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                     "Missing hardware parameter: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    limits_ = dm_motor_driver::get_limits(dm_motor_driver::MotorType::DM4310);

    RCLCPP_INFO(rclcpp::get_logger(kLogger),
                "Initialized: iface=%s can_id=0x%02X mst_id=0x%02X canfd=%d",
                can_iface_.c_str(), motor_can_id_, motor_mst_id_, use_canfd_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmGripperSystem::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    can_ = std::make_unique<dm_motor_driver::SocketCAN>(can_iface_, use_canfd_);
    if (!can_->open()) {
        RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                     "Failed to open SocketCAN interface: %s", can_iface_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Switch motor to MIT mode
    uint8_t mit_cmd[8];
    dm_motor_driver::build_switch_to_mit_cmd(motor_can_id_, mit_cmd);
    can_->send(0x7FF, mit_cmd, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmGripperSystem::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    uint8_t en_cmd[8];
    dm_motor_driver::build_enable_cmd(en_cmd);
    for (int i = 0; i < 5; i++) {
        can_->send(motor_can_id_, en_cmd, 8);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "Motor enabled.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmGripperSystem::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    uint8_t dis_cmd[8];
    dm_motor_driver::build_disable_cmd(dis_cmd);
    for (int i = 0; i < 5; i++) {
        can_->send(motor_can_id_, dis_cmd, 8);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    RCLCPP_INFO(rclcpp::get_logger(kLogger), "Motor disabled.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DmGripperSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Joint name comes from the URDF joint element
    const std::string joint_name = info_.joints[0].name;

    state_interfaces.emplace_back(
        joint_name, hardware_interface::HW_IF_POSITION, &hw_pos_state_);
    state_interfaces.emplace_back(
        joint_name, hardware_interface::HW_IF_EFFORT, &hw_eff_state_);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DmGripperSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    const std::string joint_name = info_.joints[0].name;

    command_interfaces.emplace_back(
        joint_name, hardware_interface::HW_IF_POSITION, &hw_pos_cmd_);

    return command_interfaces;
}

hardware_interface::return_type DmGripperSystem::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Non-blocking: try to drain one feedback frame
    uint32_t rx_id = 0;
    uint8_t  data[64] = {};
    uint8_t  len  = 0;

    if (can_->recv(rx_id, data, len, 0 /*non-blocking*/)) {
        if (rx_id == motor_mst_id_ && len >= 8) {
            dm_motor_driver::decode_feedback(data, len, limits_, motor_state_);
        }
    }

    if (motor_state_.valid) {
        hw_pos_state_ = static_cast<double>(motor_state_.pos);
        hw_eff_state_ = static_cast<double>(motor_state_.tau);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DmGripperSystem::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // Simple position-hold: kp=20, kd=0.5, tau_ff=0
    dm_motor_driver::MotorCommand cmd;
    cmd.kp  = 20.0f;
    cmd.kd  = 0.5f;
    cmd.q   = static_cast<float>(hw_pos_cmd_);
    cmd.dq  = 0.0f;
    cmd.tau = 0.0f;

    uint8_t payload[8];
    dm_motor_driver::pack_mit_frame(cmd, limits_, payload);
    can_->send(motor_can_id_, payload, 8);

    return hardware_interface::return_type::OK;
}

} // namespace dm_gripper_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    dm_gripper_hardware::DmGripperSystem,
    hardware_interface::SystemInterface)
