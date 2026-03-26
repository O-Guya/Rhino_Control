#include "dm_gripper_action_server/gripper_action_server.hpp"

#include "dm_gripper_core/impedance_controller.hpp"

#include <chrono>
#include <thread>

namespace dm_gripper_action_server {

GripperActionServer::GripperActionServer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("gripper_action_server", options)
{
    // Declare and read parameters
    can_iface_       = declare_parameter<std::string>("can_interface", "can0");
    motor_can_id_    = static_cast<uint32_t>(declare_parameter<int>("motor_can_id", 0x01));
    motor_mst_id_    = static_cast<uint32_t>(declare_parameter<int>("motor_mst_id", 0x00));
    use_canfd_       = declare_parameter<bool>("use_canfd", true);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 200.0);
    max_width_m_     = declare_parameter<double>("max_width_m", 0.10);

    limits_ = dm_motor_driver::get_limits(dm_motor_driver::MotorType::DM4310);

    // Build state machine
    dm_gripper_core::GripperKinematics kin(
        declare_parameter<double>("screw_lead_m", 7.957747e-4));

    dm_gripper_core::ContactDetector::Params cp;
    cp.tau_threshold       = static_cast<float>(declare_parameter<double>("contact_tau_threshold", 2.0));
    cp.pos_stuck_threshold = static_cast<float>(declare_parameter<double>("contact_pos_stuck_threshold", 0.001));
    cp.window_size         = declare_parameter<int>("contact_window_size", 20);

    state_machine_ = std::make_unique<dm_gripper_core::GripperStateMachine>(kin, cp);

    // Default controller: impedance
    dm_gripper_core::ImpedanceController::Params ip;
    ip.k_spring  = static_cast<float>(declare_parameter<double>("kp", 20.0));
    ip.k_damper  = static_cast<float>(declare_parameter<double>("kd", 0.5));
    ip.tau_max   = static_cast<float>(declare_parameter<double>("tau_max", 5.0));
    state_machine_->set_controller(
        std::make_unique<dm_gripper_core::ImpedanceController>(ip));

    // Open CAN
    can_ = std::make_unique<dm_motor_driver::SocketCAN>(can_iface_, use_canfd_);
    if (!can_->open()) {
        RCLCPP_WARN(get_logger(), "Could not open CAN interface %s — running in simulation mode",
                    can_iface_.c_str());
    } else {
        // Switch to MIT and enable
        uint8_t mit[8];
        dm_motor_driver::build_switch_to_mit_cmd(motor_can_id_, mit);
        can_->send(0x7FF, mit, 8);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        uint8_t en[8];
        dm_motor_driver::build_enable_cmd(en);
        for (int i = 0; i < 5; i++) {
            can_->send(motor_can_id_, en, 8);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    // Create action server
    action_server_ = rclcpp_action::create_server<GripperCommand>(
        this,
        "gripper_command",
        std::bind(&GripperActionServer::handle_goal,     this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&GripperActionServer::handle_cancel,   this, std::placeholders::_1),
        std::bind(&GripperActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "GripperActionServer ready (iface=%s, %.0f Hz)",
                can_iface_.c_str(), control_rate_hz_);
}

GripperActionServer::~GripperActionServer()
{
    control_active_.store(false);
    if (recv_thread_.joinable()) recv_thread_.join();

    if (can_ && can_->is_open()) {
        uint8_t dis[8];
        dm_motor_driver::build_disable_cmd(dis);
        for (int i = 0; i < 5; i++) {
            can_->send(motor_can_id_, dis, 8);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

rclcpp_action::GoalResponse GripperActionServer::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const GripperCommand::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal: position=%.4f max_effort=%.2f",
                goal->command.position, goal->command.max_effort);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperActionServer::handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
    RCLCPP_INFO(get_logger(), "Goal cancelled — releasing gripper");
    control_active_.store(false);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperActionServer::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    // Run control loop in a detached thread to avoid blocking the executor
    std::thread([this, goal_handle]() { control_loop(goal_handle); }).detach();
}

void GripperActionServer::control_loop(std::shared_ptr<GoalHandle> goal_handle)
{
    const auto goal    = goal_handle->get_goal();
    const double width = goal->command.position;  // GripperCommand uses position as opening width

    control_active_.store(true);
    state_machine_->command_width(width);

    using clock    = std::chrono::steady_clock;
    using duration = std::chrono::duration<double>;
    const duration cycle(1.0 / control_rate_hz_);

    auto feedback = std::make_shared<GripperCommand::Feedback>();
    auto result   = std::make_shared<GripperCommand::Result>();

    while (rclcpp::ok() && control_active_.load() && !state_machine_->is_done()) {
        auto t0 = clock::now();

        // Receive motor feedback (non-blocking)
        if (can_ && can_->is_open()) {
            uint32_t rx_id = 0;
            uint8_t  data[64] = {};
            uint8_t  len  = 0;
            if (can_->recv(rx_id, data, len, 0)) {
                if (rx_id == motor_mst_id_ && len >= 8) {
                    dm_motor_driver::decode_feedback(data, len, limits_, motor_state_);
                }
            }
        }

        // Run state machine
        dm_motor_driver::MotorCommand cmd = state_machine_->update(motor_state_);

        // Send motor command
        if (can_ && can_->is_open()) {
            uint8_t payload[8];
            dm_motor_driver::pack_mit_frame(cmd, limits_, payload);
            can_->send(motor_can_id_, payload, 8);
        }

        // Publish feedback
        feedback->feedback.position    = motor_state_.pos;
        feedback->feedback.effort      = motor_state_.tau;
        feedback->feedback.stalled     = false;
        feedback->feedback.reached_goal= false;
        goal_handle->publish_feedback(feedback);

        // Check if cancelled
        if (goal_handle->is_canceling()) {
            state_machine_->command_open(max_width_m_);
            result->reached_goal = false;
            goal_handle->canceled(result);
            control_active_.store(false);
            return;
        }

        std::this_thread::sleep_until(
            t0 + std::chrono::duration_cast<clock::duration>(cycle));
    }

    result->reached_goal = state_machine_->is_done();
    result->position     = motor_state_.pos;
    result->effort       = motor_state_.tau;
    result->stalled      = false;

    if (goal_handle->is_active()) {
        goal_handle->succeed(result);
    }
    control_active_.store(false);
}

} // namespace dm_gripper_action_server

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dm_gripper_action_server::GripperActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
