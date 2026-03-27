#pragma once

/**
 * dm_protocol.hpp
 *
 * 达妙电机 CAN 通信协议定义。
 *
 * 包含：
 *   1. 数据结构：MotorState（反馈）、MotorCommand（MIT 控制命令）
 *   2. 量化参数：DM4310 的物理量范围
 *   3. 协议函数：MIT 帧打包、反馈帧解析、特殊命令生成
 *
 * 协议参考：
 *   达妙上手文档 §7 电机底层控制 API
 *
 * 设计原则：
 *   - 纯函数，无状态，无副作用
 *   - 不依赖 ROS 或任何外部库
 *   - 所有打包/解包操作均为可单元测试的纯数学变换
 */

#include <cstdint>
#include <string>

namespace dm_motor_driver {

// ─────────────────────────────────────────────────────────────
// 量化参数（需与电机内部寄存器设置一致）
// ─────────────────────────────────────────────────────────────

/**
 * DM4310 物理量映射范围。
 *
 * 这些值必须与电机内部 PMAX/VMAX/TMAX 寄存器一致，
 * 否则发送的命令和收到的反馈会产生等比例缩放错误。
 *
 * 默认值来自 Phase 1 实验配置（DM4310 出厂默认）：
 *   PMAX = 12.5 rad，VMAX = 30 rad/s，TMAX = 10 Nm
 */
struct MotorLimits {
    float q_max   = 12.5f;   ///< 位置映射范围 ±q_max (rad)
    float dq_max  = 30.0f;   ///< 速度映射范围 ±dq_max (rad/s)
    float tau_max = 10.0f;   ///< 力矩映射范围 ±tau_max (Nm)

    // Kp/Kd 范围固定，不可由寄存器修改
    static constexpr float kp_min = 0.0f;
    static constexpr float kp_max = 500.0f;
    static constexpr float kd_min = 0.0f;
    static constexpr float kd_max = 5.0f;
};

// ─────────────────────────────────────────────────────────────
// 电机状态（反馈）
// ─────────────────────────────────────────────────────────────

/**
 * 从电机反馈帧解析出的状态。
 *
 * 反馈帧格式（CAN ID = MST_ID，8 字节）：
 *   D[0]    : ID(低4bit) | ERR(高4bit)
 *   D[1:2]  : 位置，16-bit 定点
 *   D[3:4]  : 速度高12bit（跨字节）
 *   D[4:5]  : 力矩低12bit（跨字节）
 *   D[6]    : MOS 温度 (°C)
 *   D[7]    : 线圈温度 (°C)
 */
struct MotorState {
    uint8_t motor_id = 0;    ///< 电机 CAN ID（反馈帧 D[0] 低4位）
    uint8_t err_code = 0;    ///< 错误码（反馈帧 D[0] 高4位）

    float   pos  = 0.0f;    ///< 当前位置 (rad)
    float   vel  = 0.0f;    ///< 当前速度 (rad/s)
    float   tau  = 0.0f;    ///< 当前力矩估算 (Nm)，来自电流环

    float   Tmos   = 0.0f;  ///< MOS 管温度 (°C)
    float   Tcoil  = 0.0f;  ///< 线圈温度 (°C)

    bool    valid  = false;  ///< true 表示本结构体已被填充（非默认值）

    /**
     * 错误码含义（err_code 字段）：
     *   0 = 失能（正常）
     *   1 = 使能（正常运行）
     *   2 = 输出轴编码器未校准
     *   8 = 超压
     *   9 = 欠压
     *   A = 过电流
     *   B = MOS 过温
     *   C = 电机线圈过温
     *   D = 通讯丢失
     *   E = 过载
     */
    bool is_enabled()    const { return err_code == 1; }
    bool has_error()     const { return err_code > 1; }
    std::string err_str() const;
};

// ─────────────────────────────────────────────────────────────
// MIT 控制命令
// ─────────────────────────────────────────────────────────────

/**
 * MIT 模式控制命令（阻抗控制）。
 *
 * 电机内部执行：
 *   τ_output = kp × (q_des - q) + kd × (dq_des - dq) + tau_ff
 *
 * 参数范围（由 MotorLimits 定义）：
 *   q_des   ∈ [-q_max,  +q_max]   (rad)
 *   dq_des  ∈ [-dq_max, +dq_max]  (rad/s)
 *   kp      ∈ [0, 500]
 *   kd      ∈ [0, 5]
 *   tau_ff  ∈ [-tau_max, +tau_max] (Nm)
 */
struct MotorCommand {
    float q_des  = 0.0f;   ///< 目标位置 (rad)
    float dq_des = 0.0f;   ///< 目标速度 (rad/s)
    float kp     = 0.0f;   ///< 位置比例增益
    float kd     = 0.5f;   ///< 速度阻尼增益（默认 0.5 为安全起步值）
    float tau_ff = 0.0f;   ///< 力矩前馈 (Nm)
};

// ─────────────────────────────────────────────────────────────
// 协议函数
// ─────────────────────────────────────────────────────────────

/**
 * 将 MIT 命令打包为 8 字节 CAN 数据。
 *
 * 帧布局（共 60 bit 有效信息，打包进 8 字节）：
 *   Byte 0    : q_des[15:8]
 *   Byte 1    : q_des[7:0]
 *   Byte 2    : dq_des[11:4]
 *   Byte 3    : dq_des[3:0] | kp[11:8]
 *   Byte 4    : kp[7:0]
 *   Byte 5    : kd[11:4]
 *   Byte 6    : kd[3:0]    | tau_ff[11:8]
 *   Byte 7    : tau_ff[7:0]
 *
 * @param cmd     控制命令
 * @param limits  电机量化参数（必须与电机内部设置一致）
 * @param out     输出缓冲区（8 字节）
 */
void pack_mit_frame(const MotorCommand& cmd,
                    const MotorLimits& limits,
                    uint8_t out[8]);

/**
 * 将反馈帧原始字节解析为 MotorState。
 *
 * @param data    CAN 帧 data 字段（至少 8 字节）
 * @param limits  电机量化参数
 * @return        解析出的电机状态（valid = true）
 */
MotorState unpack_feedback(const uint8_t data[8],
                           const MotorLimits& limits);

/**
 * 生成使能命令的 8 字节载荷（全 FF...FC）。
 * 发送到电机 CAN ID（MIT 模式 offset = 0x00）。
 */
void make_enable_payload(uint8_t out[8]);

/**
 * 生成失能命令的 8 字节载荷（全 FF...FD）。
 */
void make_disable_payload(uint8_t out[8]);

/**
 * 生成设置零点命令（全 FF...FE）。
 */
void make_set_zero_payload(uint8_t out[8]);

/**
 * 生成清除错误命令（全 FF...FB）。
 */
void make_clear_err_payload(uint8_t out[8]);

/**
 * 生成写寄存器命令的 8 字节载荷（发送到 0x7FF）。
 *
 * 帧格式：[CANID_L, CANID_H, 0x55, RID, d0, d1, d2, d3]
 *
 * @param motor_can_id  目标电机 CAN ID
 * @param rid           寄存器地址（见 §8.1 寄存器列表）
 * @param value         写入值的原始字节（低位在前）
 * @param out           输出缓冲区（8 字节）
 */
void make_write_reg_payload(uint16_t motor_can_id, uint8_t rid,
                            uint32_t value,
                            uint8_t out[8]);

/**
 * 生成读寄存器命令的 4 字节载荷（发送到 0x7FF）。
 *
 * 帧格式：[CANID_L, CANID_H, 0x33, RID]
 *
 * @param motor_can_id  目标电机 CAN ID
 * @param rid           寄存器地址
 * @param out           输出缓冲区（4 字节）
 */
void make_read_reg_payload(uint16_t motor_can_id, uint8_t rid,
                           uint8_t out[4]);

// ─────────────────────────────────────────────────────────────
// 常用寄存器地址（RID）
// ─────────────────────────────────────────────────────────────
namespace reg {
    static constexpr uint8_t CTRL_MODE = 0x0A;  ///< 控制模式（1=MIT, 2=位置速度, 3=速度, 4=力位混控）
    static constexpr uint8_t MST_ID    = 0x07;  ///< 反馈帧 ID
    static constexpr uint8_t ESC_ID    = 0x08;  ///< 接收 ID
    static constexpr uint8_t PMAX      = 0x15;  ///< 位置映射范围
    static constexpr uint8_t VMAX      = 0x16;  ///< 速度映射范围
    static constexpr uint8_t TMAX      = 0x17;  ///< 力矩映射范围
    static constexpr uint8_t CAN_BR    = 0x23;  ///< CAN 波特率
}  // namespace reg

// ─────────────────────────────────────────────────────────────
// 内部量化辅助函数（暴露供单元测试使用）
// ─────────────────────────────────────────────────────────────

/**
 * 浮点数线性映射到无符号定点整数。
 *
 * 映射关系：out = round((x - xmin) / (xmax - xmin) × (2^bits - 1))
 * 超出范围时自动钳位到 [0, 2^bits - 1]。
 *
 * @param x     输入浮点值
 * @param xmin  物理量最小值
 * @param xmax  物理量最大值
 * @param bits  定点位数（12 或 16）
 */
uint16_t float_to_uint(float x, float xmin, float xmax, int bits);

/**
 * 无符号定点整数反映射到浮点数。
 *
 * 映射关系：out = x / (2^bits - 1) × (xmax - xmin) + xmin
 */
float uint_to_float(uint16_t x, float xmin, float xmax, int bits);

}  // namespace dm_motor_driver