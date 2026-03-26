#pragma once

#include <deque>

namespace dm_gripper_core {

/// Dual-criterion contact detector for a screw-drive gripper.
///
/// Contact is declared when BOTH:
///   1. The sliding-window mean of |τ| exceeds tau_threshold, AND
///   2. The position has been "stuck" (Δpos < pos_stuck_threshold) for
///      at least window_size consecutive samples.
///
/// This avoids false positives from transient torque spikes during motion.
class ContactDetector {
public:
    struct Params {
        float tau_threshold;       // [Nm]  torque window mean threshold
        float pos_stuck_threshold; // [rad] max Δpos considered "stuck"
        int   window_size;         // number of samples in sliding window
    };

    explicit ContactDetector(Params p);

    /// Feed one sample. Returns true if contact is currently detected.
    /// @param pos  motor position [rad]
    /// @param tau  motor torque   [Nm]
    bool update(float pos, float tau);

    /// Reset detector state (e.g. after gripper releases).
    void reset();

    bool in_contact() const { return in_contact_; }

private:
    Params             params_;
    std::deque<float>  tau_window_;      // sliding window of |τ|
    float              prev_pos_{0.0f};
    int                stuck_count_{0};
    bool               in_contact_{false};
};

} // namespace dm_gripper_core
