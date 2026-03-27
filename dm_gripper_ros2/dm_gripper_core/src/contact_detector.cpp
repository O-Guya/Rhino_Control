#include "dm_gripper_core/contact_detector.hpp"
#include <cmath>  // std::abs

namespace dm_gripper_core {

ContactDetector::ContactDetector(const Params& p) : p_(p) {}

ContactResult ContactDetector::update(float q_cmd, float q_actual, float tau_fb) {
    // 1. 计算本帧误差
    float q_error = std::abs(q_cmd - q_actual);

    // 更新滑动窗口
    q_error_window_.push_back(q_error);
    if (static_cast<int>(q_error_window_.size()) > p_.window_size) {
        q_error_window_.pop_front();
    }

    // 2. 力矩判据：tau_fb 超阈视为接触
    bool tau_triggered = (tau_fb > p_.tau_threshold);

    // 3. 位置判据（"卡住"检测）
    //    条件 A：|q_error| 足够大（还没到位）
    //    条件 B：窗口内 q_error 没有明显减小
    bool pos_triggered = false;
    if (q_error > p_.min_q_error &&
        static_cast<int>(q_error_window_.size()) == p_.window_size) {
        // 比较窗口最新值与最旧值的差
        // 若差值 < delta_q_threshold → q_error 基本没减小 → 卡住
        float oldest = q_error_window_.front();
        float newest = q_error_window_.back();
        float delta  = oldest - newest;  // 正值 = q_error 在减小（正在收敛）

        // delta < delta_q_threshold：收敛速度不足，认为卡住
        pos_triggered = (delta < p_.delta_q_threshold);
    }

    // 4. 组合结果
    if (tau_triggered && pos_triggered) {
        last_result_ = ContactResult::BOTH;
    } else if (tau_triggered) {
        last_result_ = ContactResult::TAU_ONLY;
    } else if (pos_triggered) {
        last_result_ = ContactResult::POS_ONLY;
    } else {
        last_result_ = ContactResult::NONE;
    }

    return last_result_;
}

bool ContactDetector::is_contact() const {
    return last_result_ != ContactResult::NONE;
}

void ContactDetector::reset() {
    q_error_window_.clear();
    last_result_ = ContactResult::NONE;
}

}  // namespace dm_gripper_core
