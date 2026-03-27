#include "dm_gripper_core/gripper_kinematics.hpp"
#include <cmath>
#include <algorithm>  // std::clamp, std::swap
#include <limits>

namespace dm_gripper_core {

// ─── 辅助：双圆交点 ────────────────────────────────────────────────────────

/**
 * @brief 求两圆的两个交点
 *
 * 圆1：圆心 (c1x,c1y)，半径 r1
 * 圆2：圆心 (c2x,c2y)，半径 r2
 *
 * 返回 false 表示无实数解（两圆不相交）。
 * 通过 p1/p2 返回两个候选交点（不排序，调用方选根）。
 */
static bool circle_intersect_both(
    float c1x, float c1y, float r1,
    float c2x, float c2y, float r2,
    float& p1x, float& p1y,
    float& p2x, float& p2y)
{
    float dx = c2x - c1x;
    float dy = c2y - c1y;
    float d  = std::sqrt(dx*dx + dy*dy);

    if (d > r1 + r2 + 1e-6f || d < std::abs(r1 - r2) - 1e-6f || d < 1e-9f) {
        return false;
    }

    float a   = (r1*r1 - r2*r2 + d*d) / (2.0f * d);
    float h_sq = r1*r1 - a*a;
    if (h_sq < 0.0f) h_sq = 0.0f;
    float h = std::sqrt(h_sq);

    float mx = c1x + a * dx / d;
    float my = c1y + a * dy / d;

    // 垂直方向分量（沿 (dy/d, -dx/d) 方向偏移）
    p1x = mx + h * dy / d;
    p1y = my - h * dx / d;
    p2x = mx - h * dy / d;
    p2y = my + h * dx / d;

    return true;
}

// ─── GripperKinematics ─────────────────────────────────────────────────────

GripperKinematics::GripperKinematics(const Params& p) : p_(p) {
    if (p_.pitch_mm <= 0.0f) {
        throw std::invalid_argument("pitch_mm 必须大于零");
    }
    rad2mm_ = p_.pitch_mm / (2.0f * static_cast<float>(M_PI));
}

float GripperKinematics::_fwd_x_contact(float q) const {
    const float NaN = std::numeric_limits<float>::quiet_NaN();

    // ── Step A：螺母 Y 坐标 ──────────────────────────────────────────────
    float Y_nut = p_.Y_nut_0 + q * rad2mm_;

    // ── Step B：求 J_A ───────────────────────────────────────────────────
    // J_A 在圆(J_B, L2) 与 圆(P_nut=(0,Y_nut), L1) 的交点上
    // 正确根：J_A 在两根中 y 较大的那个（机构上方）
    float p1x, p1y, p2x, p2y;
    if (!circle_intersect_both(
            p_.J_B_x, p_.J_B_y, p_.L2,
            0.0f,     Y_nut,    p_.L1,
            p1x, p1y, p2x, p2y)) {
        return NaN;
    }
    // 选 y 更大的根（J_A 在 J_B 上方，靠近机构运动侧）
    float JAx = (p1y > p2y) ? p1x : p2x;
    float JAy = (p1y > p2y) ? p1y : p2y;

    // ── Step C：主动杆当前角度 ──────────────────────────────────────────
    float theta2 = std::atan2(JAy - p_.J_B_y, JAx - p_.J_B_x);

    // ── Step D：欠驱动杆角度（刚性耦合，固定角差 δ）────────────────────
    float theta3 = theta2 + p_.delta;

    // ── Step E：J_C 位置 ────────────────────────────────────────────────
    float JCx = p_.J_B_x + p_.L3 * std::cos(theta3);
    float JCy = p_.J_B_y + p_.L3 * std::sin(theta3);

    // ── Step F：求 J_D ───────────────────────────────────────────────────
    // J_D 在圆(J_C, L4) 与 圆(F_pivot, L5) 的交点上
    // 正确根：J_D.y < F_pivot.y（初始构型验证，J_D 在 F_pivot 下方）
    if (!circle_intersect_both(
            JCx,          JCy,          p_.L4,
            p_.F_pivot_x, p_.F_pivot_y, p_.L5,
            p1x, p1y, p2x, p2y)) {
        return NaN;
    }
    // 选 y 较小的根（J_D 在 F_pivot 下方）
    float JDx = (p1y < p2y) ? p1x : p2x;
    float JDy = (p1y < p2y) ? p1y : p2y;

    // ── Step G：接触面 X 坐标 ───────────────────────────────────────────
    float theta_rocker  = std::atan2(JDy - p_.F_pivot_y, JDx - p_.F_pivot_x);
    float theta_contact = theta_rocker + p_.phi_arm;
    return p_.F_pivot_x + p_.L_arm * std::cos(theta_contact);
}

float GripperKinematics::_jacobian(float q) const {
    constexpr float EPS = 1e-4f;
    float x1 = _fwd_x_contact(q + EPS);
    float x2 = _fwd_x_contact(q - EPS);
    if (std::isnan(x1) || std::isnan(x2)) return 0.0f;
    // w = 2|x_contact|，x_contact < 0 → w = -2*x_contact
    // dw/dq = -2 * (x1 - x2) / (2*EPS)
    return -2.0f * (x1 - x2) / (2.0f * EPS);
}

float GripperKinematics::motor_pos_to_width(float q) const {
    float q_clamped = std::clamp(q, p_.q_min, p_.q_max);
    float x = _fwd_x_contact(q_clamped);
    if (std::isnan(x)) {
        // 奇异点保护：退回到安全边界
        x = _fwd_x_contact(p_.q_max - 0.1f);
    }
    return 2.0f * std::abs(x);
}

float GripperKinematics::width_to_motor_pos(float w) const {
    auto [w_min, w_max] = get_width_range();
    float w_clamped = std::clamp(w, w_min, w_max);

    // 确定单调方向
    float q_lo = p_.q_min, q_hi = p_.q_max;
    float w_lo = motor_pos_to_width(q_lo);
    float w_hi = motor_pos_to_width(q_hi);
    if (w_lo > w_hi) { std::swap(q_lo, q_hi); std::swap(w_lo, w_hi); }

    // 二分搜索（50次迭代）
    for (int i = 0; i < 50; ++i) {
        float q_mid = 0.5f * (q_lo + q_hi);
        float w_mid = motor_pos_to_width(q_mid);
        if (w_mid < w_clamped) q_lo = q_mid;
        else                   q_hi = q_mid;
    }
    return 0.5f * (q_lo + q_hi);
}

float GripperKinematics::torque_to_force(float tau, float q) const {
    // 虚功原理：tau [Nm] × dq [rad] = F_grip [N] × d(w/2) [m]
    // → F_grip [N] = tau [Nm] × 1000 [mm/m] / |dw/dq [mm/rad]|
    // 注：dw/dq < 0（q增大=收紧=w减小），取绝对值后 F 与 tau 同号
    // 正 tau = 收紧 = 正夹持力
    float dwdq = _jacobian(q);  // mm/rad
    if (std::abs(dwdq) < 1e-6f) return 0.0f;
    // 1000 是 mm→m 换算；负号使符号与物理意义一致（dw/dq<0时取正F）
    return -tau * 1000.0f / dwdq;
}

std::pair<float, float> GripperKinematics::get_width_range() const {
    float wa = motor_pos_to_width(p_.q_min);
    float wb = motor_pos_to_width(p_.q_max);
    return {std::min(wa, wb), std::max(wa, wb)};
}

}  // namespace dm_gripper_core
