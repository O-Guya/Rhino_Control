/**
 * dm_motor.cpp
 *
 * DmMotor 高层封装实现。
 * 管理接收线程、状态更新、看门狗和生命周期。
 */

#include "dm_motor_driver/dm_motor.hpp"
#include "dm_motor_driver/dm_protocol.hpp"

#include <unistd.h>  // usleep

namespace dm_motor_driver {

// ─────────────────────────────────────────────────────────────
// 构造 / 析构
// ─────────────────────────────────────────────────────────────

DmMotor::DmMotor(SocketCan& can,
                 uint32_t can_id,
                 uint32_t mst_id,
                 const MotorLimits& limits,
                 int stale_timeout_ms)
    : can_(can)
    , can_id_(can_id)
    , mst_id_(mst_id)
    , limits_(limits)
    , stale_timeout_ms_(stale_timeout_ms)
    , running_(true)
{
    // 启动后台接收线程
    // 使用 lambda 避免 std::bind 的冗余，同时捕获 this
    recv_thread_ = std::thread([this]() { recv_loop(); });
}

DmMotor::~DmMotor()
{
    // 1. 若当前使能，先发送失能（安全停止）
    if (enabled_.load()) {
        disable();
    }

    // 2. 通知接收线程退出
    running_.store(false);

    // 3. 等待线程结束
    //    接收线程在 recv() 超时后会检查 running_，最多等待一个超时周期
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

// ─────────────────────────────────────────────────────────────
// 使能 / 失能
// ─────────────────────────────────────────────────────────────

void DmMotor::enable(int repeat)
{
    uint8_t payload[8];
    make_enable_payload(payload);

    for (int i = 0; i < repeat; ++i) {
        can_.send(can_id_, payload, 8);
        ::usleep(2000);  // 2ms 间隔（与官方例程一致）
    }
    enabled_.store(true);
}

void DmMotor::disable(int repeat)
{
    uint8_t payload[8];
    make_disable_payload(payload);

    for (int i = 0; i < repeat; ++i) {
        can_.send(can_id_, payload, 8);
        ::usleep(2000);
    }
    enabled_.store(false);
}

// ─────────────────────────────────────────────────────────────
// 命令发送
// ─────────────────────────────────────────────────────────────

bool DmMotor::send_command(const MotorCommand& cmd)
{
    uint8_t payload[8];
    pack_mit_frame(cmd, limits_, payload);
    return can_.send(can_id_, payload, 8);
}

void DmMotor::set_zero()
{
    uint8_t payload[8];
    make_set_zero_payload(payload);
    can_.send(can_id_, payload, 8);
}

void DmMotor::clear_error()
{
    uint8_t payload[8];
    make_clear_err_payload(payload);
    can_.send(can_id_, payload, 8);
}

void DmMotor::write_register(uint8_t rid, uint32_t value)
{
    uint8_t payload[8];
    make_write_reg_payload(
        static_cast<uint16_t>(can_id_), rid, value, payload);
    // 寄存器读写命令发送到广播 ID 0x7FF
    can_.send(0x7FF, payload, 8);
}

// ─────────────────────────────────────────────────────────────
// 状态查询
// ─────────────────────────────────────────────────────────────

MotorState DmMotor::get_state() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

bool DmMotor::is_stale() const
{
    return ms_since_last_feedback() > stale_timeout_ms_;
}

int64_t DmMotor::ms_since_last_feedback() const
{
    std::lock_guard<std::mutex> lock(time_mutex_);
    if (!ever_received_) {
        // 还没收到过任何反馈，返回一个很大的值
        return stale_timeout_ms_ * 10;
    }
    auto now  = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_recv_time_);
    return diff.count();
}

void DmMotor::set_feedback_callback(
    std::function<void(const MotorState&)> cb)
{
    std::lock_guard<std::mutex> lock(cb_mutex_);
    feedback_cb_ = std::move(cb);
}

// ─────────────────────────────────────────────────────────────
// 接收线程
// ─────────────────────────────────────────────────────────────

void DmMotor::recv_loop()
{
    CanFrame frame;

    while (running_.load()) {
        // recv() 会阻塞至多 recv_timeout_ms（SocketCan 构造时设置）
        // 超时后返回 false，此时检查 running_ 标志
        bool got_frame = false;
        try {
            got_frame = can_.recv(frame);
        } catch (const std::exception&) {
            // socket 被关闭或出错，退出线程
            break;
        }

        if (!got_frame) {
            // 超时或信号，继续循环（会检查 running_）
            continue;
        }

        // 只处理属于本电机的反馈帧
        // 注意：CAN 总线上所有设备的帧都会被收到，需要过滤
        if (frame.id != mst_id_) {
            continue;
        }

        // 反馈帧至少需要 8 字节（含温度数据）
        if (frame.dlc < 8) {
            continue;
        }

        // 解析反馈帧
        MotorState new_state = unpack_feedback(frame.data, limits_);

        // 更新最新状态（加锁保护）
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = new_state;
        }

        // 更新看门狗时间戳
        {
            std::lock_guard<std::mutex> lock(time_mutex_);
            last_recv_time_ = std::chrono::steady_clock::now();
            ever_received_  = true;
        }

        // 触发回调（如果已注册）
        {
            std::lock_guard<std::mutex> lock(cb_mutex_);
            if (feedback_cb_) {
                feedback_cb_(new_state);
            }
        }
    }
}

}  // namespace dm_motor_driver