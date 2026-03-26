#pragma once

#include "dm_motor_driver/socketcan.hpp"
#include "dm_motor_driver/dm_motor.hpp"
#include "dm_motor_driver/dm_protocol.hpp"
#include "dm_gripper_core/gripper_state_machine.hpp"
#include "dm_gripper_core/gripper_kinematics.hpp"
#include "dm_gripper_core/contact_detector.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"

#include <memory>
#include <thread>
#include <atomic>

namespace dm_gripper_action_server {

/// ROS 2 node that exposes a GripperCommand action server.
///
/// The action server drives the GripperStateMachine at a fixed control rate
/// (default 200 Hz) and publishes feedback until the gripper reaches its
/// goal state (IDLE or GRASPING).
class GripperActionServer : public rclcpp::Node {
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandle     = rclcpp_action::ServerGoalHandle<GripperCommand>;

    explicit GripperActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~GripperActionServer();

private:
    // ── Action callbacks ───────────────────────────────────────────────────────
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const GripperCommand::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    // ── Control loop ───────────────────────────────────────────────────────────
    void control_loop(std::shared_ptr<GoalHandle> goal_handle);

    // ── Members ────────────────────────────────────────────────────────────────
    rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;

    std::unique_ptr<dm_motor_driver::SocketCAN>        can_;
    dm_motor_driver::MotorLimits                       limits_;
    dm_motor_driver::MotorState                        motor_state_;
    std::unique_ptr<dm_gripper_core::GripperStateMachine> state_machine_;

    std::atomic<bool>  control_active_{false};
    std::thread        recv_thread_;

    // Parameters
    std::string can_iface_;
    uint32_t    motor_can_id_{0x01};
    uint32_t    motor_mst_id_{0x00};
    bool        use_canfd_{true};
    double      control_rate_hz_{200.0};
    double      max_width_m_{0.10};
};

} // namespace dm_gripper_action_server
