#pragma once
#include "dm_gripper_core/gripper_state_machine.hpp"  // GripperPhase, GripperMotorCmd, GripperMotorState

namespace dm_gripper_core {

/**
 * @brief 力位混合控制器
 *
 * 根据当前 GripperPhase 选择控制策略：
 *
 *   IDLE:        kp=0,  kd=0.5  → 纯阻尼，保持当前位置
 *   APPROACHING: kp=30, kd=1.0  → 位置控制趋近目标
 *   CONTACTING:  kp=5,  kd=2.0  → 低刚度轻推，减少冲击
 *   GRASPING:    kp=0,  kd=0.5, tau_ff=target_tau → 纯力矩维持
 *
 * 与 GripperStateMachine 的关系：
 *   状态机负责阶段切换逻辑，本控制器负责根据阶段计算具体指令。
 *   两者可以分离使用（便于单独测试）。
 */
class ForcePositionController {
public:
    struct Params {
        // 各阶段 kp/kd（来自 gripper_params.yaml）
        float approaching_kp = 30.0f;
        float approaching_kd =  1.0f;
        float contacting_kp  =  5.0f;
        float contacting_kd  =  2.0f;
        float grasping_kp    =  0.0f;
        float grasping_kd    =  0.5f;
        float idle_kp        =  0.0f;
        float idle_kd        =  0.5f;

        float tau_max        =  3.0f;  // 全局力矩上限 (Nm)
    };

    explicit ForcePositionController(const Params& p);

    /**
     * @brief 计算力位混合控制指令
     * @param phase       当前夹爪阶段
     * @param state       当前电机状态
     * @param q_target    目标电机角度 (rad)
     * @param target_tau  目标夹持力矩 (Nm)（仅 GRASPING 阶段使用）
     * @return            电机指令
     */
    GripperMotorCmd compute(GripperPhase phase,
                            const GripperMotorState& state,
                            float q_target,
                            float target_tau) const;

    const Params& params() const { return p_; }

private:
    Params p_;
};

}  // namespace dm_gripper_core
