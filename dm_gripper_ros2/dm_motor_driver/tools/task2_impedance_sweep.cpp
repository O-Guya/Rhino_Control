/**
 * impedance_sweep.cpp
 *
 * DM4310 阻抗参数摸底工具
 * Task 2 - Phase 1 实验模块
 *
 * 用途：
 *   以 MIT 模式发送阻抗命令，支持三种实验模式：
 *     kp_sweep   - kd 固定，扫描 kp={5,10,20,50,100}，各保持 5s
 *     kd_sweep   - kp 固定，扫描 kd={0.5,1.0,2.0,5.0}，各保持 5s
 *     tau_limit  - 固定 kp/kd，验证 tau_max 截断效果（用手阻拦）
 *
 * 编译：
 *   g++ -std=c++17 -O2 -I include tools/impedance_sweep.cpp -lpthread -o impedance_sweep
 *
 * 运行：
 *   sudo ./impedance_sweep kp_sweep  --kd 1.0 --out csv/exp2_kp_sweep.csv
 *   sudo ./impedance_sweep kd_sweep  --kp 20.0 --out csv/exp2_kd_sweep.csv
 *   sudo ./impedance_sweep tau_limit --kp 20.0 --kd 2.0 --tau-max 2.0
 *
 * 输出 CSV：
 *   timestamp_ms, kp, kd, q_des, q_actual, vel, tau_feedback, tau_cmd, label
 */

#include <atomic>
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
// 配置
// ─────────────────────────────────────────────────────────────
struct Config {
    std::string iface   = "can0";
    uint32_t    can_id  = 0x01;
    uint32_t    mst_id  = 0x00;
    std::string mode    = "kp_sweep";
    std::string out     = "";

    // 扫描参数
    float kp_fixed  = 20.0f;   // kd_sweep / tau_limit 用
    float kd_fixed  = 1.0f;    // kp_sweep / tau_limit 用
    float tau_max   = 3.0f;    // tau_limit 模式截断上限 (Nm)
    float q_des     = 0.0f;    // 目标位置 (rad)，保持静止

    // DM4310 限位（与电机内部设置一致）
    float Q_MAX   = 12.5f;
    float DQ_MAX  = 30.0f;
    float TAU_MAX = 10.0f;
};

// ─────────────────────────────────────────────────────────────
// 量化
// ─────────────────────────────────────────────────────────────
static uint16_t float_to_uint(float x, float xmin, float xmax, int bits)
{
    float norm = (x - xmin) / (xmax - xmin);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    return static_cast<uint16_t>(norm * static_cast<float>((1 << bits) - 1));
}

static float uint_to_float(uint16_t x, float xmin, float xmax, int bits)
{
    return static_cast<float>(x) / static_cast<float>((1 << bits) - 1)
           * (xmax - xmin) + xmin;
}

// ─────────────────────────────────────────────────────────────
// MIT 帧打包（tau_ff 带截断）
// ─────────────────────────────────────────────────────────────
static void pack_mit(const Config& cfg,
                     float kp, float kd,
                     float q, float dq,
                     float tau_ff, float tau_max,
                     uint8_t out[8])
{
    // tau_max 截断（双向）
    if (tau_ff >  tau_max) tau_ff =  tau_max;
    if (tau_ff < -tau_max) tau_ff = -tau_max;

    uint16_t q_u   = float_to_uint(q,   -cfg.Q_MAX,   cfg.Q_MAX,   16);
    uint16_t dq_u  = float_to_uint(dq,  -cfg.DQ_MAX,  cfg.DQ_MAX,  12);
    uint16_t kp_u  = float_to_uint(kp,  0.0f, 500.0f, 12);
    uint16_t kd_u  = float_to_uint(kd,  0.0f,   5.0f, 12);
    uint16_t tau_u = float_to_uint(tau_ff, -cfg.TAU_MAX, cfg.TAU_MAX, 12);

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
// 反馈解析
// ─────────────────────────────────────────────────────────────
struct Feedback {
    float pos = 0.0f, vel = 0.0f, tau = 0.0f;
    float Tmos = 0.0f, Tcoil = 0.0f;
    int   err  = 0;
    bool  valid = false;
};

static Feedback decode(const uint8_t* d, const Config& cfg)
{
    Feedback fb;
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
        perror("ioctl"); close(sock); return -1;
    }
    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind"); close(sock); return -1;
    }
    struct timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 100'000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return sock;
}

static void send_frame(int sock, uint32_t id, const uint8_t* d, uint8_t len)
{
    struct can_frame f{};
    f.can_id  = id & CAN_SFF_MASK;
    f.can_dlc = len;
    memcpy(f.data, d, len);
    ssize_t _w = write(sock, &f, sizeof(f)); (void)_w;
}

static bool recv_fb(int sock, uint32_t mst_id, Feedback& fb, const Config& cfg)
{
    struct can_frame f{};
    if (read(sock, &f, sizeof(f)) < 0) return false;
    if ((f.can_id & CAN_SFF_MASK) == mst_id && f.can_dlc >= 8) {
        fb = decode(f.data, cfg);
        return true;
    }
    return false;
}

static void send_enable(int sock, uint32_t id)
{
    uint8_t p[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    send_frame(sock, id, p, 8);
}
static void send_disable(int sock, uint32_t id)
{
    uint8_t p[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
    send_frame(sock, id, p, 8);
}

// ─────────────────────────────────────────────────────────────
// CSV / 在线统计
// ─────────────────────────────────────────────────────────────
struct Record {
    int64_t ts_ms;
    float kp, kd, q_des, q_actual, vel, tau_fb, tau_cmd;
    std::string label;
};

static void write_csv(const std::string& path, const std::vector<Record>& recs)
{
    std::ofstream f(path);
    f << "timestamp_ms,kp,kd,q_des_rad,q_actual_rad,vel_rad_s,"
         "tau_feedback_Nm,tau_cmd_Nm,label\n";
    for (const auto& r : recs) {
        f << r.ts_ms << ","
          << std::fixed << std::setprecision(4)
          << r.kp << "," << r.kd << ","
          << r.q_des << "," << r.q_actual << ","
          << r.vel  << "," << r.tau_fb << ","
          << r.tau_cmd << "," << r.label << "\n";
    }
    std::cout << "[CSV] " << recs.size() << " records → " << path << "\n";
}

struct Stats {
    int n = 0; double mean = 0, M2 = 0;
    void update(double x) {
        ++n; double d = x-mean; mean += d/n; M2 += d*(x-mean);
    }
    double std() const { return n<2 ? 0.0 : std::sqrt(M2/(n-1)); }
    double max_abs = 0.0;
    void update_max(double x) { if(std::abs(x)>max_abs) max_abs=std::abs(x); }
};

// ─────────────────────────────────────────────────────────────
// 核心控制循环
// ─────────────────────────────────────────────────────────────
static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop.store(true); }

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

struct LoopResult {
    Stats pos_error;   // q_des - q_actual
    Stats vel;
    Stats tau_fb;
    bool  oscillated = false;  // 位置误差峰值 > 0.5 rad 认为振荡
};

static LoopResult run_loop(int sock, const Config& cfg,
                            float kp, float kd, float q_des,
                            float tau_max, double duration_sec,
                            const std::string& label,
                            std::vector<Record>& records)
{
    const std::chrono::duration<double> period(0.005);
    auto t_start = Clock::now();
    auto t_end   = t_start + std::chrono::duration_cast<Clock::duration>(
                       std::chrono::duration<double>(duration_sec));

    uint8_t payload[8];
    Feedback fb;
    LoopResult result;

    while (!g_stop.load() && Clock::now() < t_end) {
        auto t0 = Clock::now();

        // 阻抗控制律：tau_ff = 0（纯 kp/kd，不加前馈）
        pack_mit(cfg, kp, kd, q_des, 0.0f, 0.0f, tau_max, payload);
        send_frame(sock, cfg.can_id, payload, 8);

        if (recv_fb(sock, cfg.mst_id, fb, cfg) && fb.valid) {
            auto ts = std::chrono::duration_cast<Ms>(Clock::now() - t_start);
            float err = q_des - fb.pos;

            records.push_back({ts.count(), kp, kd, q_des,
                                fb.pos, fb.vel, fb.tau, 0.0f, label});

            result.pos_error.update(err);
            result.pos_error.update_max(err);
            result.vel.update(fb.vel);
            result.tau_fb.update(fb.tau);

            if (std::abs(err) > 0.5f) result.oscillated = true;
        }

        std::this_thread::sleep_until(
            t0 + std::chrono::duration_cast<Clock::duration>(period));
    }
    return result;
}

// ─────────────────────────────────────────────────────────────
// 实时打印线程（供 tau_limit 模式使用）
// ─────────────────────────────────────────────────────────────
static void print_live(const std::vector<Record>& records, std::atomic<bool>& stop)
{
    while (!stop.load()) {
        if (!records.empty()) {
            const auto& r = records.back();
            std::cout << "\r  q_err=" << std::setw(7) << std::setprecision(4)
                      << (r.q_des - r.q_actual)
                      << " rad  tau_fb=" << std::setw(7) << r.tau_fb
                      << " Nm  vel=" << std::setw(6) << std::setprecision(2)
                      << r.vel << " rad/s" << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "\n";
}

// ─────────────────────────────────────────────────────────────
// 模式 1：kp_sweep
// ─────────────────────────────────────────────────────────────
static void run_kp_sweep(int sock, const Config& cfg)
{
    const std::vector<float> kp_levels = {5.0f, 10.0f, 20.0f, 50.0f, 100.0f};
    const float kd        = cfg.kd_fixed;
    const double hold_sec = 5.0;
    const float  tau_max  = cfg.TAU_MAX;  // 全量程，不限制（观察原始行为）

    std::cout << "\n[KP_SWEEP] kp 扫描（kd=" << kd << " 固定）\n"
              << "  电机保持在当前位置附近，用手轻拨轴感受弹性感\n"
              << "  如发现明显振荡请按 Ctrl+C\n\n";

    send_enable(sock, cfg.can_id);
    usleep(200'000);
    for (int i = 0; i < 5; ++i) { send_enable(sock, cfg.can_id); usleep(2000); }

    // 先稳定 2s，获取当前位置作为 q_des
    std::vector<Record> warmup;
    run_loop(sock, cfg, 5.0f, kd, cfg.q_des, tau_max, 2.0, "warmup", warmup);
    float q_des = warmup.empty() ? cfg.q_des : warmup.back().q_actual;
    std::cout << "[INFO] 零点锁定：q_des = " << q_des << " rad\n\n";

    std::vector<Record> records;
    records.reserve(static_cast<size_t>(kp_levels.size() * hold_sec * 200));

    std::cout << "  kp    | 位置误差 max(rad) | 速度 std(rad/s) | 振荡？\n"
              << "  ------+------------------+------------------+-------\n";

    for (float kp : kp_levels) {
        if (g_stop.load()) break;

        std::string label = "kp" + std::to_string(static_cast<int>(kp));
        auto res = run_loop(sock, cfg, kp, kd, q_des, tau_max, hold_sec, label, records);

        std::cout << "  " << std::setw(5) << kp << " | "
                  << std::setw(16) << std::setprecision(4) << res.pos_error.max_abs << " | "
                  << std::setw(16) << res.vel.std() << " | "
                  << (res.oscillated ? "⚠️  振荡" : "OK") << "\n";

        if (res.oscillated) {
            std::cout << "\n  ⚠️  检测到振荡，停止继续增大 kp\n"
                      << "  推荐 kp 安全上限：" << kp / 2.0f << "\n";
            break;
        }
    }

    if (!cfg.out.empty()) write_csv(cfg.out, records);

    std::cout << "\n[KP_SWEEP] 完成\n"
              << "  → 将最大无振荡 kp 填入 exp2_impedance_tuning.md\n"
              << "  → 下一步：kd_sweep --kp <安全值>\n";
}

// ─────────────────────────────────────────────────────────────
// 模式 2：kd_sweep
// ─────────────────────────────────────────────────────────────
static void run_kd_sweep(int sock, const Config& cfg)
{
    const std::vector<float> kd_levels = {0.5f, 1.0f, 2.0f, 5.0f};
    const float kp        = cfg.kp_fixed;
    const double hold_sec = 5.0;
    const float  tau_max  = cfg.TAU_MAX;

    std::cout << "\n[KD_SWEEP] kd 扫描（kp=" << kp << " 固定）\n"
              << "  每个 kd 保持 " << hold_sec << "s，期间用手拨动轴一下，\n"
              << "  观察恢复速度和过冲量\n\n";

    send_enable(sock, cfg.can_id);
    usleep(200'000);
    for (int i = 0; i < 5; ++i) { send_enable(sock, cfg.can_id); usleep(2000); }

    std::vector<Record> warmup;
    run_loop(sock, cfg, kp, 1.0f, cfg.q_des, tau_max, 2.0, "warmup", warmup);
    float q_des = warmup.empty() ? cfg.q_des : warmup.back().q_actual;
    std::cout << "[INFO] 零点：q_des = " << q_des << " rad\n\n";

    std::vector<Record> records;
    records.reserve(static_cast<size_t>(kd_levels.size() * hold_sec * 200));

    std::cout << "  kd    | 速度峰值(rad/s) | tau_fb std(Nm) | 手感描述\n"
              << "  ------+----------------+----------------+----------\n";

    for (float kd : kd_levels) {
        if (g_stop.load()) break;

        std::string label = "kd" + std::to_string(static_cast<int>(kd * 10));
        auto res = run_loop(sock, cfg, kp, kd, q_des, tau_max, hold_sec, label, records);

        std::cout << "  " << std::setw(5) << kd << " | "
                  << std::setw(14) << std::setprecision(3) << res.vel.max_abs << " | "
                  << std::setw(14) << res.tau_fb.std() << " | "
                  << "(手动记录)\n";
    }

    if (!cfg.out.empty()) write_csv(cfg.out, records);

    std::cout << "\n[KD_SWEEP] 完成\n"
              << "  临界阻尼估算：kd_critical ≈ 2 × sqrt(kp × J_eq)\n"
              << "  J_eq（空载约等于电机转动惯量）= 待确认\n"
              << "  → 将感觉最顺滑的 kd 填入 exp2_impedance_tuning.md\n";
}

// ─────────────────────────────────────────────────────────────
// 模式 3：tau_limit（验证力矩截断 + 手动阻力感受）
// ─────────────────────────────────────────────────────────────
static void run_tau_limit(int sock, const Config& cfg)
{
    const float kp      = cfg.kp_fixed;
    const float kd      = cfg.kd_fixed;
    const float tau_max = cfg.tau_max;

    std::cout << "\n[TAU_LIMIT] 力矩截断验证\n"
              << "  kp=" << kp << "  kd=" << kd << "  tau_max=" << tau_max << " Nm\n"
              << "  操作：用手阻拦电机轴，感受最大阻力（应等于 tau_max 对应的力）\n"
              << "  按 Ctrl+C 结束\n\n";

    send_enable(sock, cfg.can_id);
    usleep(200'000);
    for (int i = 0; i < 5; ++i) { send_enable(sock, cfg.can_id); usleep(2000); }

    std::vector<Record> warmup;
    run_loop(sock, cfg, kp, kd, cfg.q_des, tau_max, 2.0, "warmup", warmup);
    float q_des = warmup.empty() ? cfg.q_des : warmup.back().q_actual;
    std::cout << "[INFO] 零点：q_des = " << q_des << " rad\n";
    std::cout << "[INFO] 开始，实时显示中...\n";

    std::vector<Record> records;
    records.reserve(20000);

    // 实时打印线程
    std::atomic<bool> print_stop{false};
    std::thread pt([&](){ print_live(records, print_stop); });

    // 修改 record 写入，带 q_des
    auto t_start = Clock::now();
    const std::chrono::duration<double> period(0.005);

    while (!g_stop.load()) {
        auto t0 = Clock::now();
        uint8_t payload[8];
        pack_mit(cfg, kp, kd, q_des, 0.0f, 0.0f, tau_max, payload);
        send_frame(sock, cfg.can_id, payload, 8);

        Feedback fb;
        if (recv_fb(sock, cfg.mst_id, fb, cfg) && fb.valid) {
            auto ts = std::chrono::duration_cast<Ms>(Clock::now() - t_start);
            records.push_back({ts.count(), kp, kd, q_des,
                               fb.pos, fb.vel, fb.tau, tau_max, "tau_limit"});
        }
        std::this_thread::sleep_until(
            t0 + std::chrono::duration_cast<Clock::duration>(period));
    }

    print_stop.store(true);
    if (pt.joinable()) pt.join();

    // 统计 tau_fb 峰值
    float tau_fb_max = 0.0f;
    for (const auto& r : records)
        if (std::abs(r.tau_fb) > tau_fb_max) tau_fb_max = std::abs(r.tau_fb);

    std::cout << "\n[TAU_LIMIT] 统计：\n"
              << "  tau_max 设定: " << tau_max << " Nm\n"
              << "  tau_feedback 峰值: " << tau_fb_max << " Nm\n";

    if (tau_fb_max < tau_max * 1.2f)
        std::cout << "  ✅ 力矩截断有效（tau_fb ≈ tau_max 量级）\n";
    else
        std::cout << "  ⚠️  tau_fb 超出 tau_max 较多，检查 TAU_MAX 参数是否与电机匹配\n";

    if (!cfg.out.empty()) write_csv(cfg.out, records);
}

// ─────────────────────────────────────────────────────────────
// 参数解析
// ─────────────────────────────────────────────────────────────
static Config parse_args(int argc, char** argv)
{
    Config cfg;
    if (argc < 2) {
        std::cout
            << "Usage: impedance_sweep <mode> [options]\n"
            << "  mode:  kp_sweep | kd_sweep | tau_limit\n"
            << "  --iface  <can0>    CAN 接口\n"
            << "  --id     <0x01>    电机 CAN ID\n"
            << "  --mst    <0x00>    反馈 ID\n"
            << "  --kp     <20.0>    固定 kp（kd_sweep/tau_limit 用）\n"
            << "  --kd     <1.0>     固定 kd（kp_sweep/tau_limit 用）\n"
            << "  --tau-max <3.0>    tau_limit 截断上限 (Nm)\n"
            << "  --q-des  <0.0>     目标位置 (rad)，默认当前位置\n"
            << "  --out    <file>    CSV 输出路径\n"
            << "  --pmax   <12.5>    位置映射范围\n"
            << "  --vmax   <30.0>    速度映射范围\n"
            << "  --tmax   <10.0>    力矩映射范围\n";
        std::exit(0);
    }
    cfg.mode = argv[1];
    for (int i = 2; i < argc - 1; ++i) {
        std::string k = argv[i], v = argv[i+1]; ++i;
        if      (k == "--iface")   cfg.iface    = v;
        else if (k == "--id")      cfg.can_id   = static_cast<uint32_t>(std::stoi(v,nullptr,0));
        else if (k == "--mst")     cfg.mst_id   = static_cast<uint32_t>(std::stoi(v,nullptr,0));
        else if (k == "--kp")      cfg.kp_fixed = std::stof(v);
        else if (k == "--kd")      cfg.kd_fixed = std::stof(v);
        else if (k == "--tau-max") cfg.tau_max  = std::stof(v);
        else if (k == "--q-des")   cfg.q_des    = std::stof(v);
        else if (k == "--out")     cfg.out      = v;
        else if (k == "--pmax")    cfg.Q_MAX    = std::stof(v);
        else if (k == "--vmax")    cfg.DQ_MAX   = std::stof(v);
        else if (k == "--tmax")    cfg.TAU_MAX  = std::stof(v);
    }
    if (cfg.out.empty())
        cfg.out = "csv/exp2_" + cfg.mode + ".csv";
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
              << " DM4310 阻抗参数摸底工具 - Task 2\n"
              << "===========================================\n"
              << "  接口: " << cfg.iface << "  电机 ID: 0x"
              << std::hex << cfg.can_id << std::dec << "\n"
              << "  模式: " << cfg.mode << "\n\n";

    if (std::system("mkdir -p csv") != 0)
        std::cerr << "[WARN] cannot create csv dir\n";

    int sock = open_can(cfg.iface);
    if (sock < 0) {
        std::cerr << "[ERROR] 无法打开 " << cfg.iface
                  << "\n  sudo ip link set " << cfg.iface
                  << " up type can bitrate 1000000\n";
        return 1;
    }

    if      (cfg.mode == "kp_sweep")  run_kp_sweep(sock, cfg);
    else if (cfg.mode == "kd_sweep")  run_kd_sweep(sock, cfg);
    else if (cfg.mode == "tau_limit") run_tau_limit(sock, cfg);
    else {
        std::cerr << "[ERROR] 未知模式: " << cfg.mode << "\n";
        close(sock); return 1;
    }

    for (int i = 0; i < 5; ++i) { send_disable(sock, cfg.can_id); usleep(2000); }
    close(sock);
    std::cout << "[OK] 安全退出\n";
    return 0;
}