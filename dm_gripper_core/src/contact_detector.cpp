#include "dm_gripper_core/contact_detector.hpp"

#include <cmath>
#include <numeric>

namespace dm_gripper_core {

ContactDetector::ContactDetector(Params p)
    : params_(p)
{
    reset();
}

bool ContactDetector::update(float pos, float tau)
{
    // 1. Update sliding window of |τ|
    tau_window_.push_back(std::fabs(tau));
    while (static_cast<int>(tau_window_.size()) > params_.window_size) {
        tau_window_.pop_front();
    }

    float tau_mean = 0.0f;
    if (!tau_window_.empty()) {
        tau_mean = std::accumulate(tau_window_.begin(), tau_window_.end(), 0.0f)
                   / static_cast<float>(tau_window_.size());
    }

    // 2. Position-stuck detection
    float delta_pos = std::fabs(pos - prev_pos_);
    prev_pos_ = pos;
    if (delta_pos < params_.pos_stuck_threshold) {
        stuck_count_++;
    } else {
        stuck_count_ = 0;
    }

    // 3. Dual criterion: torque threshold AND position stuck for full window
    bool torque_high  = (tau_mean >= params_.tau_threshold);
    bool pos_stuck    = (stuck_count_ >= params_.window_size);

    in_contact_ = torque_high && pos_stuck;
    return in_contact_;
}

void ContactDetector::reset()
{
    tau_window_.clear();
    prev_pos_    = 0.0f;
    stuck_count_ = 0;
    in_contact_  = false;
}

} // namespace dm_gripper_core
