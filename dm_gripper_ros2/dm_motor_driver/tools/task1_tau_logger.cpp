/**
 * tau_logger.cpp
 *
 * DM4310 τ_feedback 质量评估工具
 * Task 1 - Phase 1 实验模块
 *
 * 用途：
 *   以 200Hz 向电机发送固定 tau_ff，同步记录 τ_feedback 到 CSV。
 *   支持三种实验模式：
 *     freerun  - 空载扫描，tau_ff 从 {0.1, 0.5, 1.0, 2.0} Nm 各跑 10s
 *     manual   - 固定 tau_ff，手动施力，持续记录（Ctrl+C 结束）
 *     friction - 位置往返，记录正反向到达时的 τ_feedback 差值
 *
 * 编译：
 *   mkdir -p build && cd build && cmake .. && make tau_logger -j4
 *
 * 运行（需要 CAP_NET_RAW）：
 *   sudo ./tau_logger freerun   --iface can0 --id 0x01 --out csv/exp1_freerun.csv
 *   sudo ./tau_logger manual    --iface can0 --id 0x01 --tau 1.0 --out csv/exp1_manual_load.csv
 *   sudo ./tau_logger friction  --iface can0 --id 0x01 --pos 3.14 --out csv/exp1_friction.csv
 *
 * 输出 CSV 格式：
 *   timestamp_ms, tau_cmd, pos_rad, vel_rad_s, tau_feedback, Tmos, Tcoil
 */

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

// ─────────────────────────────────────────────────────────────
// 配置（命令行可覆盖）
// ─────────────────────────────────────────────────────────────
struct Config {
    std::string iface   = "can0";
    uint32_t    can_id  = 0x01;
    uint32_t    mst_id  = 0x00;
    std::string mode    = "freerun";   // freerun | manual | friction
    std::string out     = "tau_log.csv";
    float       tau_cmd = 1.0f;        // manual 模式固定指令
    float       pos_target = 3.14f;    // friction 模式目标位置 (rad)

    // DM4310 限位（与 demo 一致，需与电机内部设置匹配）
    float Q_MAX   = 12.5f;
    float DQ_MAX  = 30.0f;
    float TAU_MAX = 10.0f;
};

// ─────────────────────────────────────────────────────────────
// 量化辅助
// ─────────────────────────────────────────────────────────────
static uint16_t float_to_uint(float x, float xmin, float xmax, int bits)
{
    float span = xmax - xmin;
    float norm = (x - xmin) / span;
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    return static_cast<uint16_t>(norm * static_cast<float>((1 << bits) - 1));
}

static float uint_to_float(uint16_t x, float xmin, float xmax, int bits)
{
    float span = xmax - xmin;
    float norm = static_cast<float>(x) / static_cast<float>((1 << bits) - 1);
    return norm * span + xmin;
}

// ─────────────────────────────────────────────────────────────
// MIT 帧打包
// ─────────────────────────────────────────────────────────────
static void pack_mit(const Config& cfg,
                     float kp, float kd,
                     float q, float dq, float tau,
                     uint8_t out[8])
{
    uint16_t q_u   = float_to_uint(q,   -cfg.Q_MAX,   cfg.Q_MAX,   16);
    uint16_t dq_u  = float_to_uint(dq,  -cfg.DQ_MAX,  cfg.DQ_MAX,  12);
    uint16_t kp_u  = float_to_uint(kp,  0.0f, 500.0f, 12);
    uint16_t kd_u  = float_to_uint(kd,  0.0f,   5.0f, 12);
    uint16_t tau_u = float_to_uint(tau, -cfg.TAU_MAX, cfg.TAU_MAX, 12);

    out[0] = (q_u >> 8) & 0xFF;
    out[1] =  q_u       & 0xFF;
    out[2] =  dq_u >> 4;
    out[3] = ((dq_u  & 0xF) << 4) | ((kp_u >> 8) & 0xF);
    out[4] =  kp_u  & 0xFF;
    out[5] =  kd_u  >> 4;
    out[6] = ((kd_u  & 0xF) << 4) | ((tau_u >> 8) & 0xF);
    out[7] =  tau_u & 0xFF;
}

// ─────────────────────────────────────────────────────────────
// 反馈帧解析
// ─────────────────────────────────────────────────────────────
struct MotorFeedback {
    float pos  = 0.0f;  // rad
    float vel  = 0.0f;  // rad/s
    float tau  = 0.0f;  // Nm
    float Tmos = 0.0f;  // °C
    float Tcoil= 0.0f;  // °C
    int   err  = 0;
    bool  valid = false;
};

static MotorFeedback decode_feedback(const uint8_t* d, const Config& cfg)
{
    MotorFeedback fb;
    fb.err  = (d[0] >> 4) & 0x0F;
    uint16_t q_u   = (static_cast<uint16_t>(d[1]) << 8) | d[2];
    uint16_t dq_u  = (static_cast<uint16_t>(d[3]) << 4) | (d[4] >> 4);
    uint16_t tau_u = (static_cast<uint16_t>(d[4] & 0xF) << 8) | d[5];
    fb.pos   = uint_to_float(q_u,   -cfg.Q_MAX,  cfg.Q_MAX,  16);
    fb.vel   = uint_to_float(dq_u,  -cfg.DQ_MAX, cfg.DQ_MAX, 12);
    fb.tau   = uint_to_float(tau_u, -cfg.TAU_MAX, cfg.TAU_MAX, 12);
    fb.Tmos  = static_cast<float>(d[6]);
    fb.Tcoil = static_cast<float>(d[7]);
    fb.valid = true;
    return fb;
}

// ─────────────────────────────────────────────────────────────
// SocketCAN
// ─────────────────────────────────────────────────────────────
static int open_can(const std::string& iface)
{
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) { perror("socket"); return -1; }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl(SIOCGIFINDEX)");
        close(sock); return -1;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind"); close(sock); return -1;
    }

    struct timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 100'000;  // 100ms recv timeout
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return sock;
}

static bool send_frame(int sock, uint32_t id, const uint8_t* data, uint8_t len)
{
    struct can_frame f{};
    f.can_id  = id & CAN_SFF_MASK;
    f.can_dlc = len;
    memcpy(f.data, data, len);
    return write(sock, &f, sizeof(f)) == sizeof(f);
}

static bool recv_feedback(int sock, uint32_t mst_id, MotorFeedback& out_fb,
                          const Config& cfg)
{
    struct can_frame f{};
    ssize_t n = read(sock, &f, sizeof(f));
    if (n < 0) return false;
    uint32_t rx_id = f.can_id & CAN_SFF_MASK;
    if (rx_id == mst_id && f.can_dlc >= 8) {
        out_fb = decode_feedback(f.data, cfg);
        return true;
    }
    return false;
}

// 使能命令
static void send_enable(int sock, uint32_t id)
{
    uint8_t p[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    send_frame(sock, id, p, 8);
}

// 失能命令
static void send_disable(int sock, uint32_t id)
{
    uint8_t p[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    send_frame(sock, id, p, 8);
}

// ─────────────────────────────────────────────────────────────
// CSV 记录器
// ─────────────────────────────────────────────────────────────
struct Record {
    int64_t timestamp_ms;
    float   tau_cmd;
    float   pos;
    float   vel;
    float   tau_feedback;
    float   Tmos;
    float   Tcoil;
    std::string label;  // 用于区分实验阶段
};

static void write_csv(const std::string& path, const std::vector<Record>& records)
{
    std::ofstream f(path);
    if (!f) {
        std::cerr << "[ERROR] Cannot open output file: " << path << "\n";
        return;
    }
    f << "timestamp_ms,tau_cmd_Nm,pos_rad,vel_rad_s,tau_feedback_Nm,Tmos_C,Tcoil_C,label\n";
    for (const auto& r : records) {
        f << r.timestamp_ms << ","
          << std::fixed << std::setprecision(5)
          << r.tau_cmd      << ","
          << r.pos          << ","
          << r.vel          << ","
          << r.tau_feedback << ","
          << r.Tmos         << ","
          << r.Tcoil        << ","
          << r.label        << "\n";
    }
    std::cout << "[CSV] Written " << records.size() << " records to " << path << "\n";
}

// ─────────────────────────────────────────────────────────────
// 实时统计（在线均值/方差，Welford 算法）
// ─────────────────────────────────────────────────────────────
struct OnlineStats {
    int     n    = 0;
    double  mean = 0.0;
    double  M2   = 0.0;

    void update(double x) {
        ++n;
        double delta  = x - mean;
        mean += delta / n;
        double delta2 = x - mean;
        M2   += delta * delta2;
    }
    double variance() const { return n < 2 ? 0.0 : M2 / (n - 1); }
    double stddev()   const { return std::sqrt(variance()); }
};

// ─────────────────────────────────────────────────────────────
// 全局退出信号
// ─────────────────────────────────────────────────────────────
static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop.store(true); }

// ─────────────────────────────────────────────────────────────
// 控制循环：以 200Hz 发送命令，接收反馈，填充 records
// ─────────────────────────────────────────────────────────────
using Clock    = std::chrono::steady_clock;
using Ms       = std::chrono::milliseconds;
using Duration = std::chrono::duration<double>;

static void control_loop(int sock, const Config& cfg,
                         float kp, float kd, float q_des, float dq_des, float tau_ff,
                         double duration_sec,
                         const std::string& label,
                         std::vector<Record>& records)
{
    const Duration period(0.005);  // 5ms = 200Hz
    auto t_start = Clock::now();
    auto t_end   = t_start + std::chrono::duration_cast<Clock::duration>(
                       std::chrono::duration<double>(duration_sec));

    uint8_t payload[8];
    MotorFeedback fb;

    while (!g_stop.load() && Clock::now() < t_end) {
        auto t0 = Clock::now();

        // 发送 MIT 帧
        pack_mit(cfg, kp, kd, q_des, dq_des, tau_ff, payload);
        send_frame(sock, cfg.can_id, payload, 8);

        // 接收反馈
        if (recv_feedback(sock, cfg.mst_id, fb, cfg) && fb.valid) {
            auto elapsed = std::chrono::duration_cast<Ms>(Clock::now() - t_start);
            records.push_back({
                elapsed.count(),
                tau_ff,
                fb.pos, fb.vel, fb.tau, fb.Tmos, fb.Tcoil,
                label
            });
        }

        std::this_thread::sleep_until(
            t0 + std::chrono::duration_cast<Clock::duration>(period));
    }
}

// ─────────────────────────────────────────────────────────────
// 模式 1：freerun — 空载扫描
// ─────────────────────────────────────────────────────────────
static void run_freerun(int sock, const Config& cfg)
{
    const std::vector<float> tau_levels = {0.0f, 0.1f, 0.5f, 1.0f, 2.0f};
    const double hold_sec = 10.0;

    std::cout << "\n[FREERUN] 空载力矩扫描\n"
              << "  电机轴保持自由（无外力）\n"
              << "  tau_ff 序列: ";
    for (float t : tau_levels) std::cout << t << " ";
    std::cout << "Nm，各 " << hold_sec << " 秒\n\n";

    // 先发零力矩使能，等待 2s 稳定
    std::cout << "[INFO] 使能电机，等待 2s...\n";
    send_enable(sock, cfg.can_id);
    usleep(200'000);
    for (int i = 0; i < 5; ++i) {
        send_enable(sock, cfg.can_id);
        usleep(2000);
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::vector<Record> records;
    records.reserve(static_cast<size_t>(tau_levels.size() * hold_sec * 200));

    for (float tau : tau_levels) {
        if (g_stop.load()) break;

        std::cout << "[FREERUN] tau_ff = " << tau << " Nm ... ";
        std::flush(std::cout);

        OnlineStats stats;
        std::string label = "tau_" + std::to_string(static_cast<int>(tau * 10));

        // 先运行 hold_sec，实时统计
        auto before_size = records.size();
        control_loop(sock, cfg,
                     /*kp=*/0.0f, /*kd=*/0.5f,
                     /*q_des=*/0.0f, /*dq_des=*/0.0f, /*tau_ff=*/tau,
                     hold_sec, label, records);

        // 计算这段的统计
        for (size_t i = before_size; i < records.size(); ++i)
            stats.update(static_cast<double>(records[i].tau_feedback));

        std::cout << "完成  mean=" << std::fixed << std::setprecision(4)
                  << stats.mean << " Nm  std=" << stats.stddev() << " Nm\n";
    }

    write_csv(cfg.out, records);

    // 统计报告
    std::cout << "\n────────────────────────────────────────\n"
              << "[FREERUN] 实验完成，结果摘要：\n\n"
              << "  tau_ff(Nm) | tau_fb 均值(Nm) | tau_fb 标准差(Nm)\n"
              << "  -----------+----------------+------------------\n";

    for (float tau : tau_levels) {
        OnlineStats s;
        std::string lbl = "tau_" + std::to_string(static_cast<int>(tau * 10));
        for (const auto& r : records)
            if (r.label == lbl) s.update(static_cast<double>(r.tau_feedback));
        std::cout << "  " << std::setw(10) << tau << " | "
                  << std::setw(14) << std::setprecision(4) << s.mean << " | "
                  << std::setw(18) << s.stddev() << "\n";
    }
    std::cout << "────────────────────────────────────────\n\n"
              << "[提示] 将上表填入 docs/experiments/exp1_tau_feedback_quality.md\n";
}

// ─────────────────────────────────────────────────────────────
// 模式 2：manual — 固定 tau_ff，手动施力
// ─────────────────────────────────────────────────────────────
static void run_manual(int sock, const Config& cfg)
{
    std::cout << "\n[MANUAL] 手动阻力实验\n"
              << "  固定 tau_ff = " << cfg.tau_cmd << " Nm\n"
              << "  操作：用手缓慢阻拦电机轴，观察 tau_feedback 变化\n"
              << "  按 Ctrl+C 结束\n\n";

    send_enable(sock, cfg.can_id);
    usleep(200'000);
    for (int i = 0; i < 5; ++i) { send_enable(sock, cfg.can_id); usleep(2000); }

    std::vector<Record> records;
    records.reserve(20000);

    // 实时打印
    std::thread print_thread([&](){
        while (!g_stop.load()) {
            if (!records.empty()) {
                const auto& r = records.back();
                std::cout << "\r  t=" << std::setw(6) << r.timestamp_ms << "ms"
                          << "  tau_fb=" << std::setw(7) << std::setprecision(4) << r.tau_feedback << " Nm"
                          << "  pos=" << std::setw(7) << std::setprecision(3) << r.pos << " rad"
                          << "  vel=" << std::setw(6) << std::setprecision(2) << r.vel << " rad/s"
                          << std::flush;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "\n";
    });

    control_loop(sock, cfg,
                 /*kp=*/0.0f, /*kd=*/0.5f,
                 /*q_des=*/0.0f, /*dq_des=*/0.0f, /*tau_ff=*/cfg.tau_cmd,
                 1e9,  // 无限循环直到 Ctrl+C
                 "manual", records);

    g_stop.store(true);
    if (print_thread.joinable()) print_thread.join();

    write_csv(cfg.out, records);

    // 统计
    OnlineStats s_free, s_load;
    // 前 20% 认为是自由运动，后 80% 认为有手动阻力（粗略）
    size_t split = records.size() / 5;
    for (size_t i = 0;     i < split;           ++i) s_free.update(records[i].tau_feedback);
    for (size_t i = split; i < records.size();  ++i) s_load.update(records[i].tau_feedback);

    std::cout << "\n[MANUAL] 统计摘要：\n"
              << "  前段（自由）均值: " << s_free.mean << " Nm  std: " << s_free.stddev() << " Nm\n"
              << "  后段（施力）均值: " << s_load.mean << " Nm  std: " << s_load.stddev() << " Nm\n"
              << "  差值（可检测信号）: " << (s_load.mean - s_free.mean) << " Nm\n";
}

// ─────────────────────────────────────────────────────────────
// 模式 3：friction — 位置往返，估算摩擦力矩
// ─────────────────────────────────────────────────────────────
static void run_friction(int sock, const Config& cfg)
{
    const float pos_a =  0.0f;
    const float pos_b =  cfg.pos_target;
    const float kp    = 30.0f;
    const float kd    =  2.0f;
    const double hold_sec = 5.0;

    std::cout << "\n[FRICTION] 摩擦力矩估算实验\n"
              << "  往返位置: " << pos_a << " rad <-> " << pos_b << " rad\n"
              << "  kp=" << kp << "  kd=" << kd << "\n"
              << "  每个位置保持 " << hold_sec << " 秒\n\n";

    send_enable(sock, cfg.can_id);
    usleep(200'000);
    for (int i = 0; i < 5; ++i) { send_enable(sock, cfg.can_id); usleep(2000); }

    std::vector<Record> records;
    records.reserve(8000);

    // 回到 pos_a
    std::cout << "[FRICTION] → 移动到 pos_a = " << pos_a << " rad\n";
    control_loop(sock, cfg, kp, kd, pos_a, 0.0f, 0.0f, hold_sec, "pos_a_fwd", records);

    // 移到 pos_b
    std::cout << "[FRICTION] → 移动到 pos_b = " << pos_b << " rad\n";
    control_loop(sock, cfg, kp, kd, pos_b, 0.0f, 0.0f, hold_sec, "pos_b_fwd", records);

    // 返回 pos_a
    std::cout << "[FRICTION] → 返回 pos_a = " << pos_a << " rad\n";
    control_loop(sock, cfg, kp, kd, pos_a, 0.0f, 0.0f, hold_sec, "pos_a_bwd", records);

    // 再到 pos_b
    std::cout << "[FRICTION] → 返回 pos_b = " << pos_b << " rad\n";
    control_loop(sock, cfg, kp, kd, pos_b, 0.0f, 0.0f, hold_sec, "pos_b_bwd", records);

    write_csv(cfg.out, records);

    // 对比稳态 τ_feedback
    auto avg_tau = [&](const std::string& lbl) {
        OnlineStats s;
        // 只取后半段（稳态）
        std::vector<size_t> idx;
        for (size_t i = 0; i < records.size(); ++i)
            if (records[i].label == lbl) idx.push_back(i);
        if (idx.empty()) return 0.0;
        size_t half = idx.size() / 2;
        for (size_t i = half; i < idx.size(); ++i)
            s.update(records[idx[i]].tau_feedback);
        return s.mean;
    };

    double tau_fwd = avg_tau("pos_b_fwd");
    double tau_bwd = avg_tau("pos_b_bwd");

    std::cout << "\n[FRICTION] 摩擦力矩估算：\n"
              << "  pos_b 正向到达时 τ_feedback: " << std::setprecision(4) << tau_fwd << " Nm\n"
              << "  pos_b 逆向到达时 τ_feedback: " << tau_bwd << " Nm\n"
              << "  差值（2 × 动摩擦估算）: " << std::abs(tau_fwd - tau_bwd) << " Nm\n"
              << "  单向动摩擦力矩 ≈ " << std::abs(tau_fwd - tau_bwd) / 2.0 << " Nm\n";
}

// ─────────────────────────────────────────────────────────────
// 参数解析
// ─────────────────────────────────────────────────────────────
static Config parse_args(int argc, char** argv)
{
    Config cfg;
    if (argc < 2) {
        std::cout << "Usage: tau_logger <mode> [options]\n"
                  << "  mode:  freerun | manual | friction\n"
                  << "  --iface <can0>      CAN 接口 (default: can0)\n"
                  << "  --id    <0x01>      电机 CAN ID (default: 0x01)\n"
                  << "  --mst   <0x00>      电机反馈 ID (default: 0x00)\n"
                  << "  --out   <file.csv>  输出 CSV 文件\n"
                  << "  --tau   <1.0>       manual 模式固定力矩 (Nm)\n"
                  << "  --pos   <3.14>      friction 模式目标位置 (rad)\n"
                  << "  --pmax  <12.5>      位置映射范围 (rad)\n"
                  << "  --vmax  <30.0>      速度映射范围 (rad/s)\n"
                  << "  --tmax  <10.0>      力矩映射范围 (Nm)\n";
        std::exit(0);
    }
    cfg.mode = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string key = argv[i];
        if (i + 1 >= argc) break;
        std::string val = argv[++i];
        if      (key == "--iface") cfg.iface      = val;
        else if (key == "--id")    cfg.can_id      = static_cast<uint32_t>(std::stoi(val, nullptr, 0));
        else if (key == "--mst")   cfg.mst_id      = static_cast<uint32_t>(std::stoi(val, nullptr, 0));
        else if (key == "--out")   cfg.out         = val;
        else if (key == "--tau")   cfg.tau_cmd     = std::stof(val);
        else if (key == "--pos")   cfg.pos_target  = std::stof(val);
        else if (key == "--pmax")  cfg.Q_MAX       = std::stof(val);
        else if (key == "--vmax")  cfg.DQ_MAX      = std::stof(val);
        else if (key == "--tmax")  cfg.TAU_MAX     = std::stof(val);
    }
    // 默认输出文件名
    if (cfg.out == "tau_log.csv") {
        cfg.out = "csv/exp1_" + cfg.mode + ".csv";
    }
    return cfg;
}

// ─────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    Config cfg = parse_args(argc, argv);

    std::cout << "===========================================\n"
              << " DM4310 τ_feedback 质量评估工具 - Task 1\n"
              << "===========================================\n"
              << "  接口: " << cfg.iface   << "\n"
              << "  电机 ID: 0x" << std::hex << cfg.can_id << std::dec << "\n"
              << "  反馈 ID: 0x" << std::hex << cfg.mst_id << std::dec << "\n"
              << "  模式: " << cfg.mode << "\n"
              << "  输出: " << cfg.out  << "\n\n";

    // 创建输出目录
    if (std::system("mkdir -p csv") != 0) {
        std::cerr << "[WARN] Could not create csv directory\n";
    }

    int sock = open_can(cfg.iface);
    if (sock < 0) {
        std::cerr << "[ERROR] 无法打开 " << cfg.iface << "\n"
                  << "  请先执行: sudo ip link set " << cfg.iface
                  << " up type can bitrate 1000000\n";
        return 1;
    }
    std::cout << "[OK] SocketCAN 已打开\n";

    if      (cfg.mode == "freerun")  run_freerun(sock, cfg);
    else if (cfg.mode == "manual")   run_manual(sock, cfg);
    else if (cfg.mode == "friction") run_friction(sock, cfg);
    else {
        std::cerr << "[ERROR] 未知模式: " << cfg.mode << "\n";
        close(sock);
        return 1;
    }

    // 失能
    for (int i = 0; i < 5; ++i) { send_disable(sock, cfg.can_id); usleep(2000); }
    close(sock);

    std::cout << "\n[OK] 程序安全退出\n";
    return 0;
}