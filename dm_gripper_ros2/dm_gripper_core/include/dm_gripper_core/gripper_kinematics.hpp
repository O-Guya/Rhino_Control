#pragma once
#include <utility>   // std::pair
#include <stdexcept>

namespace dm_gripper_core {

/**
 * @brief 夹爪运动学：电机角度 ↔ 开口宽度，力矩 → 夹持力
 *
 * ===================== 机构拓扑 =====================
 *
 *  DM4310(q) → 丝杆(pitch=2mm/rev) → 螺母(滑块, X=0, Y沿丝杆轴变化)
 *                 ↓ 螺母连杆 L1=13.203mm
 *             J_A (浮动铰点)
 *                 ↓ 主动杆 L2=49.300mm
 *             J_B = (-62, 74.5) [固定枢轴, 机架]
 *                 ↓ 欠驱动杆 L3=50.754mm  (与主动杆刚性耦合, 角差δ固定)
 *             J_C (浮动)
 *                 ↓ 手指连杆 L4=18.000mm
 *             J_D (浮动)
 *               ↙ 摇杆 L5=15.063mm
 *           F_pivot = (-47, 146.352) [固定, M5螺栓]
 *               ↓ 手指臂 L_arm=20.000mm (固定角差 φ_arm)
 *           接触面 (左侧 X<0, 开口宽度 w = 2|x_contact|)
 *
 * 左右完全镜像，开口宽度 w = 2 × |x_contact_left|
 *
 * 所有几何参数从 RhinoV2.0_Assembly.STEP 自动提取 + 用户实测接触面坐标确认。
 * =====================================================
 *
 * 重要约束：
 *   - 主正运动学有奇异点：q ≈ +11.0 rad（4杆锁死）
 *   - 有效范围：q_min ≈ -12.5 rad (w≈75.7mm) 到 q_max ≈ +11.0 rad (w≈69.8mm)
 *   - 逆运动学用二分法（单调函数，50次迭代精度 <1e-5 rad）
 */
class GripperKinematics {
public:
    struct Params {
        // 丝杆参数
        float pitch_mm = 2.0f;          // 丝杆导程 (mm/转)

        // 固定枢轴坐标（从STEP提取，单位 mm）
        float J_B_x = -62.0f;           // 主动杆+欠驱动杆固定枢轴 X
        float J_B_y =  74.5f;           // 主动杆+欠驱动杆固定枢轴 Y
        float F_pivot_x = -47.0f;       // 手指固定枢轴 X（M5螺栓）
        float F_pivot_y = 146.352f;     // 手指固定枢轴 Y

        // 连杆长度（从STEP提取，单位 mm）
        float L1 = 13.203f;             // 螺母连杆（nut→J_A）
        float L2 = 49.300f;             // 主动杆（J_B→J_A）
        float L3 = 50.754f;             // 欠驱动杆（J_B→J_C）
        float L4 = 18.000f;             // 手指连杆（J_C→J_D）
        float L5 = 15.063f;             // 摇杆（F_pivot→J_D）
        float L_arm = 20.000f;          // 手指接触臂（F_pivot→接触面）

        // 装配零位参数（从STEP提取）
        float Y_nut_0 = 78.904f;        // q=0 时螺母 Y 坐标

        // 预计算角度偏置（从STEP零位几何推导）
        float delta    = 1.445601f;     // 主动杆→欠驱动杆固定角差 (rad，约82.83°)
        float phi_arm  = 2.370682f;     // 接触臂→摇杆固定角差 (rad，约135.83°)

        // 安全范围
        float q_min = -12.5f;           // 电机角度下限 (rad)
        float q_max =  8.0f;           // 单调区间上限 (rad，q>8.3后w不再单调)           // 电机角度上限 (rad，留0.5rad余量避免奇异点)
    };

    explicit GripperKinematics(const Params& p);

    /**
     * @brief 电机角度 → 夹爪开口宽度 (mm)
     * @param q  电机位置 (rad)
     * @return   开口宽度 w (mm)，若超出运动学有效范围则 clamp 到边界
     *
     * 正运动学：五步链式计算（滑块曲柄 + 4连杆）
     */
    float motor_pos_to_width(float q) const;

    /**
     * @brief 夹爪开口宽度 (mm) → 电机角度 (rad)
     * @param w  目标开口宽度 (mm)
     * @return   电机角度 q (rad)
     *
     * 逆运动学：二分法数值求逆（50次迭代，精度 <1e-5 rad）
     */
    float width_to_motor_pos(float w) const;

    /**
     * @brief 电机力矩 → 单侧夹持力
     * @param tau  电机输出力矩 (Nm)
     * @param q    当前电机角度 (rad)（用于计算当前雅可比）
     * @return     单侧夹持力 F (N)
     *
     * 虚功原理：F = tau / (dw/dq)，dw/dq 由数值微分计算
     * 注：丝杆自锁，外力不会反传到 tau；此函数仅在主动施力时有意义
     */
    float torque_to_force(float tau, float q) const;

    /** 合法宽度范围 (w_min_mm, w_max_mm) */
    std::pair<float, float> get_width_range() const;

    const Params& params() const { return p_; }

private:
    Params p_;

    // 预计算常量
    float rad2mm_;      // pitch_mm / (2π)，rad → mm 系数

    /**
     * @brief 内部正运动学核心（返回接触面 x 坐标，失败返回 NAN）
     * 解两次双圆交点：nut→J_A（滑块曲柄段）+ J_C→J_D（4连杆段）
     */
    float _fwd_x_contact(float q) const;

    /**
     * @brief 数值微分 dw/dq（用于雅可比和逆运动学）
     * eps = 1e-4 rad，精度足够
     */
    float _jacobian(float q) const;
};

}  // namespace dm_gripper_core
