#pragma once

namespace dm_gripper_core {

/// Gripper transmission kinematics for DM4310 + screw drive + (future) linkage.
///
/// Coordinate conventions:
///   q  [rad]  — motor position (positive = close direction)
///   x  [m]    — linear displacement of the screw nut (positive = close)
///   w  [m]    — jaw opening width (positive = open)
///
/// The linkage mapping (x↔w) is a placeholder until the physical linkage
/// arrives and is calibrated. Until then, linear_to_width / width_to_linear
/// use a 1:1 identity with a sign flip.
class GripperKinematics {
public:
    /// @param screw_lead_m  Lead [m/rev] of the trapezoidal screw, expressed
    ///                      in metres per radian (lead_mm_per_rev / (1000 * 2π)).
    ///                      Placeholder default: 5 mm/rev → ≈ 7.96e-4 m/rad.
    explicit GripperKinematics(double screw_lead_m = 7.957747e-4);

    // ── Transmission mappings ─────────────────────────────────────────────────

    /// Motor position → linear screw displacement.
    double motor_pos_to_linear(double q) const;

    /// Linear displacement → motor position.
    double linear_to_motor_pos(double x) const;

    /// Linear displacement → jaw opening width.
    /// TODO: replace with calibrated linkage polynomial once linkage arrives.
    double linear_to_width(double x) const;

    /// Jaw opening width → linear displacement.
    double width_to_linear(double w) const;

    // ── Force/torque mapping ──────────────────────────────────────────────────

    /// Estimated jaw force from motor torque.
    /// Uses: F = τ * η / (screw_lead_m)
    /// η ≈ 0.30 for trapezoidal screw (conservative — feedback only, no back-drive).
    double torque_to_force(double tau) const;

    /// Desired jaw force → required motor torque.
    double force_to_torque(double force) const;

    // ── Accessors ─────────────────────────────────────────────────────────────
    double screw_lead() const { return screw_lead_; }

private:
    double screw_lead_;          // [m/rad]
    static constexpr double kEta = 0.30;  // screw efficiency
};

} // namespace dm_gripper_core
