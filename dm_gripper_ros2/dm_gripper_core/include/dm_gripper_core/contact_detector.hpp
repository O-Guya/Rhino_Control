#pragma once
#include <deque>
#include <cstdint>

namespace dm_gripper_core {

/**
 * @brief 接触检测结果
 */
enum class ContactResult {
    NONE     = 0,  // 无接触
    TAU_ONLY = 1,  // 仅力矩超阈
    POS_ONLY = 2,  // 仅位置卡住
    BOTH     = 3,  // 两者同时触发
};

/**
 * @brief 夹爪接触检测器（无硬件依赖，纯算法）
 *
 * 双判据：
 *   1. 力矩判据（主）：tau_fb > tau_threshold
 *   2. 位置判据（辅）：q_error 停止收敛（"卡住"检测）
 *      - |q_error| > min_q_error（排除已到位情况）
 *      - 最近 window_size 帧内 q_error 减小量 < delta_q_threshold
 *
 * 注：pos 判据是"卡住"检测，不是绝对阈值
 */
class ContactDetector {
public:
    struct Params {
        float tau_threshold     = 0.24f;  // Nm（来自 Task1 实验，手动施力可检测）
        float delta_q_threshold = 0.02f;  // rad，q_error 收敛判断灵敏度
        float min_q_error       = 0.05f;  // rad，小于此值认为已到位，不触发 pos 判据
        int   window_size       = 10;     // 帧数（200Hz 下 ≈ 50ms）
    };

    explicit ContactDetector(const Params& p);

    /**
     * @brief 每帧调用，更新接触状态
     * @param q_cmd     期望角度（位置指令）(rad)
     * @param q_actual  当前实际角度 (rad)
     * @param tau_fb    电机反馈力矩 (Nm)
     * @return          本帧接触检测结果
     */
    ContactResult update(float q_cmd, float q_actual, float tau_fb);

    /**
     * @brief 是否检测到接触（TAU_ONLY / POS_ONLY / BOTH 均返回 true）
     */
    bool is_contact() const;

    /** 重置内部状态（状态机切换时调用） */
    void reset();

    /** 读取当前结果（不更新状态） */
    ContactResult last_result() const { return last_result_; }

private:
    Params p_;

    // 滑动窗口：存储最近 window_size 帧的 |q_error|
    std::deque<float> q_error_window_;

    ContactResult last_result_ = ContactResult::NONE;
};

}  // namespace dm_gripper_core
