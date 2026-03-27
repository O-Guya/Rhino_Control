#pragma once
#include "dm_gripper_core/contact_detector.hpp"

namespace dm_gripper_core {

/**
 * @brief 夹爪运动阶段
 *
 * IDLE → APPROACHING → CONTACTING → GRASPING
 *            ↓                            ↓
 *           IDLE（超时/中止）           IDLE（release/超压）
 */
enum class GripperPhase {
    IDLE,        // 静止：纯阻尼保持位置
    APPROACHING, // 趋近：位置控制向目标宽度移动
    CONTACTING,  // 接触推进：低刚度轻推，等待力矩建立
    GRASPING,    // 抓持：纯力矩维持夹紧力
};

/**
 * @brief MotorCommand 简化版（dm_motor_driver 未在 core 中依赖时使用）
 *
 * 字段含义与 dm_protocol.hpp 中 MotorCommand 一致：
 *   kp * (q_des - q) + kd * (0 - dq) + tau_ff = 输出力矩
 */
struct GripperMotorCmd {
    float q_des  = 0.0f;  // 期望位置 (rad)
    float dq_des = 0.0f;  // 期望速度（通常为 0）
    float kp     = 0.0f;  // 位置刚度
    float kd     = 0.5f;  // 速度阻尼
    float tau_ff = 0.0f;  // 前馈力矩 (Nm)
};

/**
 * @brief 简化电机状态（仅状态机需要的字段）
 */
struct GripperMotorState {
    float pos = 0.0f;  // 当前角度 (rad)
    float vel = 0.0f;  // 当前速度 (rad/s)
    float tau = 0.0f;  // 反馈力矩 (Nm)
};

/**
 * @brief 夹爪状态机
 *
 * 负责管理 IDLE/APPROACHING/CONTACTING/GRASPING 四个阶段的切换，
 * 每帧根据电机状态和接触检测结果输出对应的 MotorCommand 参数。
 *
 * 使用方式：
 *   1. start_grasp(goal) 开始抓取
 *   2. 每帧调用 update(state, q_cmd) 获取本帧指令
 *   3. 完成后调用 release() 回到 IDLE
 */
class GripperStateMachine {
public:
    /** 抓取目标 */
    struct Goal {
        float target_q_rad = 0.0f;  // 目标电机角度 (rad)（由 GripperKinematics 转换而来）
        float max_tau      = 3.0f;  // 最大允许力矩 (Nm)
    };

    /** 各阶段固定参数 */
    struct PhaseParams {
        float kp         = 0.0f;
        float kd         = 0.5f;
        float tau_ff     = 0.0f;
        float timeout_ms = 10000.0f;  // 超时阈值（ms）
    };

    /** 状态机配置 */
    struct Params {
        PhaseParams idle        = {0.0f,  0.5f, 0.0f, 0.0f};      // 无超时
        PhaseParams approaching = {30.0f, 1.0f, 0.0f, 10000.0f};  // 10s 超时
        PhaseParams contacting  = {5.0f,  2.0f, 0.0f, 3000.0f};   // 3s 超时
        PhaseParams grasping    = {0.0f,  0.5f, 1.0f, 0.0f};      // 无超时（靠超压保护）

        // 接触检测参数
        ContactDetector::Params contact = {};
    };

    explicit GripperStateMachine(const Params& p);

    /**
     * @brief 开始一次抓取动作（从任意状态可调用，会重置内部计时器）
     */
    void start_grasp(const Goal& goal);

    /**
     * @brief 释放夹爪，回到 IDLE
     */
    void release();

    /**
     * @brief 每帧更新（200Hz 调用）
     * @param state   当前电机状态
     * @param dt_ms   距上次调用的时间 (ms)
     * @return        本帧应发送的电机指令
     */
    GripperMotorCmd update(const GripperMotorState& state, float dt_ms);

    GripperPhase current_phase() const { return phase_; }
    bool is_grasping() const { return phase_ == GripperPhase::GRASPING; }

private:
    Params p_;
    GripperPhase phase_  = GripperPhase::IDLE;
    Goal current_goal_   = {};
    float phase_time_ms_ = 0.0f;  // 当前阶段已过时间

    ContactDetector contact_;

    // 切换到新阶段，重置计时器和接触检测
    void enter_phase(GripperPhase new_phase);

    // 根据当前阶段填充 kp/kd/tau_ff（q_des 由调用方填入）
    GripperMotorCmd make_cmd(float q_des) const;

    // 超压保护：检查是否超出 goal.max_tau
    bool over_torque(float tau) const;
};

}  // namespace dm_gripper_core
