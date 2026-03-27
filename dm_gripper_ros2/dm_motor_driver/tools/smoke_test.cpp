/**
 * smoke_test.cpp
 *
 * DmMotor 硬件冒烟测试
 * Phase 2 Task 6 验证
 *
 * 目的：
 *   在真实 DM4310 电机上验证整个库调用链：
 *     SocketCan → DmMotor → enable → send_command → get_state → disable
 *
 * 验证点：
 *   1. SocketCan 能打开 CAN 接口
 *   2. DmMotor 使能后能收到有效反馈（valid=true, is_stale=false）
 *   3. 反馈数据合理（pos 在 ±12.5 rad 范围内，温度 > 0）
 *   4. 发送低刚度阻尼命令后电机状态稳定
 *   5. 接收线程能被安全停止（析构不卡死）
 *   6. 看门狗：断开后 is_stale() 最终变为 true（可选）
 *
 * 编译（在 pixi shell 或 source ROS 后）：
 *   colcon build --packages-select dm_motor_driver
 *   # 可执行文件在 install/dm_motor_driver/lib/dm_motor_driver/smoke_test
 *
 * 运行：
 *   sudo ./smoke_test [iface] [can_id] [mst_id]
 *   sudo ./smoke_test can0 0x01 0x00   （默认值）
 *
 * ⚠️  安全注意：
 *   - 电机轴上不要有大负载
 *   - 命令为纯阻尼（kp=0, kd=0.5），电机不会主动运动
 *   - Ctrl+C 或测试完成后自动失能
 */

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "dm_motor_driver/dm_motor.hpp"
#include "dm_motor_driver/dm_protocol.hpp"
#include "dm_motor_driver/socketcan.hpp"

using namespace dm_motor_driver;
using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
// 颜色输出辅助
// ─────────────────────────────────────────────────────────────
#define GREEN  "\033[32m"
#define RED    "\033[31m"
#define YELLOW "\033[33m"
#define RESET  "\033[0m"

static int g_passed = 0;
static int g_failed = 0;

static void pass(const std::string& name)
{
    std::cout << GREEN << "  [PASS] " << RESET << name << "\n";
    ++g_passed;
}

static void fail(const std::string& name, const std::string& reason)
{
    std::cout << RED << "  [FAIL] " << RESET << name << "\n"
              << "         原因：" << reason << "\n";
    ++g_failed;
}

// ─────────────────────────────────────────────────────────────
// 测试函数
// ─────────────────────────────────────────────────────────────

/**
 * 测试1：SocketCan 能成功打开接口
 */
static bool test_socketcan_open(const std::string& iface)
{
    std::cout << "\n── Test 1: SocketCan 打开接口 ──\n";
    try {
        SocketCan can(iface);
        pass("SocketCan(" + iface + ") 构造成功");
        return true;
    } catch (const std::exception& e) {
        fail("SocketCan 打开", e.what());
        return false;
    }
}

/**
 * 测试2：使能后能收到有效反馈
 */
static bool test_enable_and_feedback(const std::string& iface,
                                     uint32_t can_id,
                                     uint32_t mst_id)
{
    std::cout << "\n── Test 2: 使能 + 反馈接收 ──\n";

    SocketCan   can(iface);
    MotorLimits limits;
    DmMotor     motor(can, can_id, mst_id, limits);

    // 使能
    motor.enable();
    std::cout << "  [INFO] 使能命令已发送，等待反馈 1s...\n";
    MotorCommand keepalive;
    keepalive.kp = 0.0f; keepalive.kd = 0.5f;
    auto t_end = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < t_end) {
        motor.send_command(keepalive);
        std::this_thread::sleep_for(5ms);
    }

    // 检查是否收到反馈
    MotorState state = motor.get_state();

    if (!state.valid) {
        fail("收到反馈", "1s 内未收到任何反馈帧，检查 CAN 接线和电机 ID");
        motor.disable();
        return false;
    }
    pass("收到有效反馈帧（valid=true）");

    // 检查看门狗（刚收到，不应 stale）
    if (motor.is_stale()) {
        fail("看门狗状态", "刚收到反馈但 is_stale() 返回 true，stale_timeout 设置有误");
    } else {
        pass("看门狗正常（is_stale=false）");
    }

    // 检查使能状态
    if (!state.is_enabled()) {
        fail("电机使能状态",
             "err_code=" + std::to_string(state.err_code) +
             " (" + state.err_str() + ")，电机未进入使能状态");
    } else {
        pass("电机已使能（err_code=1）");
    }

    // 打印反馈数据
    std::cout << "\n  反馈数据快照：\n"
              << "    motor_id : " << static_cast<int>(state.motor_id) << "\n"
              << "    err_code : " << static_cast<int>(state.err_code)
              << " (" << state.err_str() << ")\n"
              << std::fixed << std::setprecision(4)
              << "    pos      : " << state.pos  << " rad\n"
              << "    vel      : " << state.vel  << " rad/s\n"
              << "    tau      : " << state.tau  << " Nm\n"
              << "    Tmos     : " << state.Tmos  << " °C\n"
              << "    Tcoil    : " << state.Tcoil << " °C\n";

    // 数值合理性检查
    if (std::abs(state.pos) > 12.5f) {
        fail("位置范围", "pos=" + std::to_string(state.pos) +
             " 超出 ±12.5 rad，检查 PMAX 参数是否与电机内部一致");
    } else {
        pass("位置值在合理范围内（|pos| ≤ 12.5 rad）");
    }

    if (state.Tmos < 10.0f || state.Tmos > 100.0f) {
        // 温度异常（<10°C 可能是解析错误，>100°C 过热）
        std::cout << YELLOW << "  [WARN] " << RESET
                  << "Tmos=" << state.Tmos << "°C，温度值异常，检查反馈解析\n";
    } else {
        pass("温度值合理（10~100°C）");
    }

    motor.disable();
    return true;
}

/**
 * 测试3：发送低刚度命令后状态稳定
 */
static bool test_send_command(const std::string& iface,
                              uint32_t can_id,
                              uint32_t mst_id)
{
    std::cout << "\n── Test 3: 发送阻尼命令 ──\n";
    std::cout << "  [INFO] 命令参数：kp=0, kd=0.5, q=当前位置, tau=0\n"
              << "         电机不会主动运动，只有阻尼感\n";

    SocketCan   can(iface);
    MotorLimits limits;
    DmMotor     motor(can, can_id, mst_id, limits);

    motor.enable();
    std::this_thread::sleep_for(500ms);

    // 获取当前位置作为目标（原地保持）
    MotorState init = motor.get_state();
    if (!init.valid) {
        fail("发送命令前获取初始状态", "未收到反馈");
        motor.disable();
        return false;
    }

    // 注册回调，统计反馈频率
    int feedback_count = 0;
    motor.set_feedback_callback([&](const MotorState&) {
        ++feedback_count;
    });

    // 发送 200Hz 阻尼命令，持续 2s
    MotorCommand cmd;
    cmd.q_des  = init.pos;   // 原地
    cmd.dq_des = 0.0f;
    cmd.kp     = 0.0f;       // 不追位置
    cmd.kd     = 0.5f;       // 纯阻尼
    cmd.tau_ff = 0.0f;

    auto t_end = std::chrono::steady_clock::now() + 2s;
    int  sent  = 0;
    while (std::chrono::steady_clock::now() < t_end) {
        motor.send_command(cmd);
        ++sent;
        std::this_thread::sleep_for(5ms);  // 200Hz
    }

    std::cout << "  [INFO] 发送了 " << sent << " 帧命令，"
              << "收到 " << feedback_count << " 帧反馈\n";

    // 反馈频率检查（期望约 200Hz × 2s = 400 帧，允许 50% 误差）
    if (feedback_count < 200) {
        fail("反馈频率",
             "2s 内只收到 " + std::to_string(feedback_count) +
             " 帧，期望 ≥ 200 帧，CAN 通信可能不稳定");
    } else {
        pass("反馈频率正常（≥ 200 帧/2s）");
    }

    // 位置漂移检查（纯阻尼命令下，位置不应大幅移动）
    MotorState final_state = motor.get_state();
    float pos_drift = std::abs(final_state.pos - init.pos);
    if (pos_drift > 0.5f) {
        std::cout << YELLOW << "  [WARN] " << RESET
                  << "位置漂移 " << pos_drift << " rad，"
                  << "可能是轴没有固定或有外力干扰\n";
    } else {
        pass("位置稳定（漂移 < 0.5 rad）");
    }

    motor.disable();
    return true;
}

/**
 * 测试4：析构安全性（接收线程能正常停止）
 */
static bool test_destructor_safety(const std::string& iface,
                                   uint32_t can_id,
                                   uint32_t mst_id)
{
    std::cout << "\n── Test 4: 析构安全性 ──\n";

    auto t_start = std::chrono::steady_clock::now();

    {
        // 在作用域内创建，离开时自动析构
        SocketCan   can(iface);
        MotorLimits limits;
        DmMotor     motor(can, can_id, mst_id, limits);
        motor.enable();
        std::this_thread::sleep_for(200ms);
        // motor 和 can 在这里析构
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - t_start);

    // 析构应该在合理时间内完成（< 2s，即两个接收超时周期）
    if (elapsed.count() > 2000) {
        fail("析构时间",
             "析构耗时 " + std::to_string(elapsed.count()) +
             "ms，超过 2s，接收线程可能卡死");
        return false;
    }

    pass("析构正常完成（耗时 " + std::to_string(elapsed.count()) + "ms）");
    return true;
}

// ─────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    // 解析参数
    std::string iface   = (argc > 1) ? argv[1] : "can0";
    uint32_t    can_id  = (argc > 2) ? static_cast<uint32_t>(
                              std::stoi(argv[2], nullptr, 0)) : 0x01;
    uint32_t    mst_id  = (argc > 3) ? static_cast<uint32_t>(
                              std::stoi(argv[3], nullptr, 0)) : 0x00;

    std::cout << "═══════════════════════════════════════\n"
              << " DmMotor 硬件冒烟测试\n"
              << "═══════════════════════════════════════\n"
              << "  接口  : " << iface << "\n"
              << "  CAN ID: 0x" << std::hex << can_id << std::dec << "\n"
              << "  MST ID: 0x" << std::hex << mst_id << std::dec << "\n\n"
              << "  ⚠️  确认电机轴安全后按 Enter 开始...\n";
    std::cin.get();

    // 逐项测试，前置测试失败则跳过后续
    bool ok = test_socketcan_open(iface);
    if (!ok) {
        std::cout << RED
                  << "\n[ABORT] SocketCan 无法打开，后续测试跳过\n"
                  << "  请先执行: sudo ip link set " << iface
                  << " up type can bitrate 1000000\n"
                  << RESET;
        return 1;
    }

    test_enable_and_feedback(iface, can_id, mst_id);
    test_send_command(iface, can_id, mst_id);
    test_destructor_safety(iface, can_id, mst_id);

    // 最终报告
    std::cout << "\n═══════════════════════════════════════\n"
              << " 测试结果：" << g_passed << " 通过  "
              << g_failed << " 失败\n"
              << "═══════════════════════════════════════\n";

    if (g_failed == 0) {
        std::cout << GREEN << " Phase 2 硬件验证通过，可进入 Phase 3\n"
                  << RESET;
    } else {
        std::cout << RED << " 有测试失败，请根据上方提示排查后重跑\n"
                  << RESET;
    }

    return g_failed == 0 ? 0 : 1;
}