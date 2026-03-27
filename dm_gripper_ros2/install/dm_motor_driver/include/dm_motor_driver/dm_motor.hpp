#pragma once

/**
 * dm_motor.hpp
 *
 * 达妙电机高层封装。
 *
 * 职责：
 *   - 持有 SocketCan 引用，提供语义化的电机控制接口
 *   - 在后台接收线程中持续读取反馈帧，更新最新状态
 *   - 提供看门狗机制：若超时未收到反馈则标记状态为 stale
 *   - 管理使能/失能生命周期
 *
 * 线程模型：
 *   ┌──────────────────────────────────────────────────┐
 *   │  调用者线程（控制循环）                             │
 *   │    send_command(cmd)  →  SocketCan::send()       │
 *   │    get_state()        →  读取 latest_state_      │
 *   └──────────────────────────────────────────────────┘
 *               ↑ mutex 保护 latest_state_
 *   ┌──────────────────────────────────────────────────┐
 *   │  接收线程（内部）                                  │
 *   │    SocketCan::recv()  →  unpack  →  latest_state_│
 *   └──────────────────────────────────────────────────┘
 *
 * 典型用法：
 *   SocketCan can("can0");
 *   MotorLimits limits;
 *   DmMotor motor(can, 0x01, 0x00, limits);
 *
 *   motor.enable();
 *
 *   MotorCommand cmd;
 *   cmd.kp = 30.0f; cmd.kd = 1.0f; cmd.q_des = 1.57f;
 *   motor.send_command(cmd);
 *
 *   MotorState state = motor.get_state();
 *   std::cout << state.pos << "\n";
 *
 *   motor.disable();
 *   // 析构时自动停止接收线程
 */

#include "dm_motor_driver/dm_protocol.hpp"
#include "dm_motor_driver/socketcan.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

namespace dm_motor_driver {

/**
 * 达妙电机高层驱动类。
 *
 * 非拷贝，可移动（但移动后原对象不可用）。
 */
class DmMotor {
public:
    /**
     * 构造函数：绑定到指定 CAN 接口和电机 ID。
     *
     * 构造时自动启动后台接收线程。
     *
     * @param can           已打开的 SocketCan 引用（调用者负责生命周期）
     * @param can_id        电机接收命令的 CAN ID（例如 0x01）
     * @param mst_id        电机发送反馈的 CAN ID（master id，默认 0x00）
     * @param limits        量化参数（必须与电机内部寄存器设置一致）
     * @param stale_timeout 超过此时长未收到反馈则标记 stale（毫秒，默认 500ms）
     */
    DmMotor(SocketCan& can,
            uint32_t can_id,
            uint32_t mst_id,
            const MotorLimits& limits,
            int stale_timeout_ms = 500);

    /**
     * 析构：先发送失能命令，再停止接收线程。
     */
    ~DmMotor();

    // 禁止拷贝
    DmMotor(const DmMotor&)            = delete;
    DmMotor& operator=(const DmMotor&) = delete;

    // ─────────────────────────────────────────────
    // 控制接口
    // ─────────────────────────────────────────────

    /**
     * 发送使能命令（重复发送 repeat 次，建议 3~5 次）。
     *
     * 使能成功后电机进入 MIT 模式待命状态。
     * 每次使能之间间隔约 2ms（与官方例程一致）。
     *
     * @param repeat  重发次数（默认 5）
     */
    void enable(int repeat = 5);

    /**
     * 发送失能命令（重复发送 repeat 次）。
     *
     * 失能后电机自由转动（无阻力），注意夹爪可能松开。
     */
    void disable(int repeat = 5);

    /**
     * 发送 MIT 控制命令。
     *
     * 线程安全：可从控制循环线程调用。
     *
     * @param cmd  MIT 控制命令（kp/kd/q_des/dq_des/tau_ff）
     * @return true  发送成功
     * @return false TX 队列满，当前帧被丢弃（下一周期重试）
     */
    bool send_command(const MotorCommand& cmd);

    /**
     * 发送设置零点命令。
     * ⚠️ 会修改电机绝对零点，谨慎使用。
     */
    void set_zero();

    /**
     * 发送清除错误命令。
     * 用于清除 err_code 非 0/1 的错误状态。
     */
    void clear_error();

    /**
     * 写入电机寄存器（发送到广播 ID 0x7FF）。
     *
     * 写入立即生效但掉电丢失，需调用 save_register() 持久化。
     *
     * @param rid    寄存器地址（见 dm_protocol.hpp reg 命名空间）
     * @param value  写入值（uint32 原始字节，由调用者负责类型转换）
     */
    void write_register(uint8_t rid, uint32_t value);

    // ─────────────────────────────────────────────
    // 状态查询
    // ─────────────────────────────────────────────

    /**
     * 获取最新电机状态（线程安全）。
     *
     * 返回接收线程最近一次更新的状态快照。
     * 若 stale_timeout 内未收到反馈，state.valid 仍为 true
     * 但 is_stale() 返回 true。
     */
    MotorState get_state() const;

    /**
     * 返回当前状态是否过期（超过 stale_timeout 未更新）。
     */
    bool is_stale() const;

    /**
     * 返回自上次收到反馈以来经过的毫秒数。
     */
    int64_t ms_since_last_feedback() const;

    /**
     * 返回是否已使能。
     */
    bool is_enabled() const { return enabled_.load(); }

    /**
     * 返回电机 CAN ID。
     */
    uint32_t can_id() const { return can_id_; }

    /**
     * 注册反馈回调（每收到一帧反馈时调用）。
     *
     * 回调在接收线程中执行，应尽快返回，不要在回调中调用 send_command()。
     *
     * @param cb  回调函数，参数为最新 MotorState
     */
    void set_feedback_callback(std::function<void(const MotorState&)> cb);

private:
    // ─────────────────────────────────────────────
    // 接收线程
    // ─────────────────────────────────────────────

    /**
     * 后台接收线程函数。
     * 持续调用 can_.recv()，过滤出属于本电机的反馈帧，解析后更新状态。
     */
    void recv_loop();

    // ─────────────────────────────────────────────
    // 成员变量
    // ─────────────────────────────────────────────

    SocketCan&   can_;       ///< CAN 接口引用（不拥有生命周期）
    uint32_t     can_id_;    ///< 电机命令 ID
    uint32_t     mst_id_;    ///< 电机反馈 ID
    MotorLimits  limits_;    ///< 量化参数

    int stale_timeout_ms_;   ///< 看门狗超时阈值（毫秒）

    // 线程安全的最新状态
    mutable std::mutex state_mutex_;
    MotorState         latest_state_;

    // 最后一次收到反馈的时间戳
    mutable std::mutex              time_mutex_;
    std::chrono::steady_clock::time_point last_recv_time_;
    bool                            ever_received_ = false;

    // 接收线程控制
    std::atomic<bool>  running_;
    std::thread        recv_thread_;

    // 使能状态标志
    std::atomic<bool>  enabled_{false};

    // 可选反馈回调
    mutable std::mutex                       cb_mutex_;
    std::function<void(const MotorState&)>   feedback_cb_;
};

}  // namespace dm_motor_driver