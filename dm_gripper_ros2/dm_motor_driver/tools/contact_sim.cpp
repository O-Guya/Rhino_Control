/**
 * contact_sim.cpp
 *
 * DM4310 接触检测阈值标定工具
 * Task 3 - Phase 1 实验模块
 *
 * 用途：
 *   驱动电机旋转，实时记录 q_error 和 tau_feedback，
 *   模拟夹爪闭合时遇到物体的过程，标定接触检测阈值。
 *
 *   两种模式：
 *     soft  - 用软性物体（橡皮泥/海绵）阻挡，模拟软物体夹取
 *     hard  - 用手或硬物阻挡，模拟硬物体夹取
 *
 * 编译：
 *   g++ -std=c++17 -O2 -I include tools/contact_sim.cpp -lpthread -o contact_sim
 *
 * 运行：
 *   sudo ./contact_sim soft --kp 30.0 --kd 1.0 --out csv/exp3_soft.csv
 *   sudo ./contact_sim hard --kp 30.0 --kd 1.0 --out csv/exp3_hard.csv
 *
 * 操作说明：
 *   1. 启动后电机缓慢向目标位置运动
 *   2. 在运动路径中放置阻挡物（soft/hard）
 *   3. 观察终端实时打印的 q_error 和 tau_fb 变化
 *   4. Ctrl+C 结束，查看统计摘要
 *
 * 输出 CSV：
 *   timestamp_ms, q_cmd, q_actual, q_error, vel, tau_feedback,
 *   contact_detected_tau, contact_detected_pos, label
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
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
    std::string iface  = "can0";
    uint32_t    can_id = 0x01;
    uint32_t    mst_id = 0x00;
    std::string mode   = "soft";
    std::string out    = "";

    float kp      = 30.0f;
    float kd      =  1.0f;
    float tau_max =  3.0f;

    // 运动参数：从当前位置缓慢推进 travel_rad
    float travel_rad  = 1.0f;   // 推进距离（rad），足够碰到阻挡物
    float approach_kp = 5.0f;   // 低刚度缓慢接近（模拟夹爪闭合速度）

    // 检测阈值（用于实时标注，不影响记录）
    float tau_threshold    = 0.24f;  // Nm，来自 Task 1 结论
    float q_err_threshold  = 0.05f;  // rad，初始猜测，实验后调整
    int   window_size      = 10;     // 帧数

    // DM4310 限位
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

static void pack_mit(const Config& cfg,
                     float kp, float kd,
                     float q, float dq, float tau_ff,
                     uint8_t out[8])
{
    if (tau_ff >  cfg.tau_max) tau_ff =  cfg.tau_max;
    if (tau_ff < -cfg.tau_max) tau_ff = -cfg.tau_max;

    uint16_t q_u   = float_to_uint(q,     -cfg.Q_MAX,   cfg.Q_MAX,   16);
    uint16_t dq_u  = float_to_uint(dq,    -cfg.DQ_MAX,  cfg.DQ_MAX,  12);
    uint16_t kp_u  = float_to_uint(kp,    0.0f, 500.0f, 12);
    uint16_t kd_u  = float_to_uint(kd,    0.0f,   5.0f, 12);
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
// 反馈
// ─────────────────────────────────────────────────────────────
struct Feedback {
    float pos = 0, vel = 0, tau = 0;
    float Tmos = 0, Tcoil = 0;
    bool  valid = false;
};

static Feedback decode(const uint8_t* d, const Config& cfg)
{
    Feedback fb;
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
    tv.tv_sec = 0; tv.tv_usec = 100'000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return sock;
}

static void send_frame(int sock, uint32_t id, const uint8_t* d, uint8_t len)
{
    struct can_frame f{};
    f.can_id = id & CAN_SFF_MASK;
    f.can_dlc = len;
    memcpy(f.data, d, len);
    ssize_t w = write(sock, &f, sizeof(f));
    (void)w;
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
// 滑动窗口接触检测器（与 ContactDetector 类设计一致）
// ─────────────────────────────────────────────────────────────
class ContactDetector {
public:
    explicit ContactDetector(const Config& cfg)
        : tau_thresh_(cfg.tau_threshold)
        , q_err_thresh_(cfg.q_err_threshold)
        , window_(cfg.window_size)
    {}

    // 返回：0=无接触, 1=tau触发, 2=position触发, 3=双重触发
    int update(float q_error, float tau_fb)
    {
        // 滑窗：记录每帧是否满足条件
        tau_window_.push_back(std::abs(tau_fb) > tau_thresh_);
        pos_window_.push_back(std::abs(q_error) > q_err_thresh_);
        if (static_cast<int>(tau_window_.size()) > window_) {
            tau_window_.pop_front();
            pos_window_.pop_front();
        }

        // 全窗口满足才触发
        bool tau_triggered = all_true(tau_window_);
        bool pos_triggered = all_true(pos_window_);

        if (tau_triggered && pos_triggered) return 3;
        if (pos_triggered) return 2;
        if (tau_triggered) return 1;
        return 0;
    }

    void reset()
    {
        tau_window_.clear();
        pos_window_.clear();
    }

private:
    float tau_thresh_, q_err_thresh_;
    int   window_;
    std::deque<bool> tau_window_, pos_window_;

    static bool all_true(const std::deque<bool>& w)
    {
        if (w.empty()) return false;
        for (bool v : w) if (!v) return false;
        return true;
    }
};

// ─────────────────────────────────────────────────────────────
// CSV
// ─────────────────────────────────────────────────────────────
struct Record {
    int64_t ts_ms;
    float   q_cmd, q_actual, q_error;
    float   vel, tau_fb;
    int     contact_tau, contact_pos;  // 0=no, 1=yes
    std::string label;
};

static void write_csv(const std::string& path, const std::vector<Record>& recs)
{
    std::ofstream f(path);
    f << "timestamp_ms,q_cmd_rad,q_actual_rad,q_error_rad,"
         "vel_rad_s,tau_feedback_Nm,"
         "contact_tau,contact_pos,label\n";
    for (const auto& r : recs) {
        f << r.ts_ms << ","
          << std::fixed << std::setprecision(5)
          << r.q_cmd    << "," << r.q_actual << ","
          << r.q_error  << "," << r.vel      << ","
          << r.tau_fb   << ","
          << r.contact_tau << "," << r.contact_pos << ","
          << r.label    << "\n";
    }
    std::cout << "[CSV] " << recs.size() << " records → " << path << "\n";
}

// ─────────────────────────────────────────────────────────────
// 全局退出
// ─────────────────────────────────────────────────────────────
static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop.store(true); }

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

// ─────────────────────────────────────────────────────────────
// 主实验循环
// ─────────────────────────────────────────────────────────────
static void run_experiment(int sock, const Config& cfg, const std::string& label)
{
    // ── 1. 使能，获取当前位置 ──────────────────────────────
    send_enable(sock, cfg.can_id);
    usleep(200'000);
    for (int i = 0; i < 5; ++i) { send_enable(sock, cfg.can_id); usleep(2000); }

    // 读取当前位置
    float q_start = 0.0f;
    {
        Feedback fb;
        for (int i = 0; i < 20; ++i) {
            if (recv_fb(sock, cfg.mst_id, fb, cfg) && fb.valid) {
                q_start = fb.pos;
                break;
            }
        }
    }
    float q_target = q_start + cfg.travel_rad;

    std::cout << "[INFO] 起始位置: " << std::fixed << std::setprecision(4)
              << q_start << " rad\n"
              << "[INFO] 目标位置: " << q_target << " rad\n"
              << "[INFO] 推进距离: " << cfg.travel_rad << " rad\n\n";

    // ── 2. 操作提示 ────────────────────────────────────────
    std::cout << "  准备好后按 Enter 开始...\n";
    std::cin.get();

    if      (label == "soft")
        std::cout << "[操作] 在电机轴运动路径上放置软性阻挡物（橡皮泥/海绵）\n"
                  << "       阻挡物放好后等待电机接触\n\n";
    else if (label == "hard")
        std::cout << "[操作] 准备好用手或硬物阻挡轴，电机开始运动后随时可以阻拦\n\n";

    // ── 3. 控制循环 ────────────────────────────────────────
    ContactDetector detector(cfg);
    std::vector<Record> records;
    records.reserve(10000);

    auto t_start = Clock::now();
    const std::chrono::duration<double> period(0.005);

    // 接触事件统计
    int64_t  contact_ts_tau = -1;   // 第一次 tau 触发时刻
    int64_t  contact_ts_pos = -1;   // 第一次 pos 触发时刻
    float    contact_q_error_at_tau = 0.0f;
    float    contact_tau_at_pos     = 0.0f;

    uint8_t payload[8];
    Feedback fb;

    while (!g_stop.load()) {
        auto t0 = Clock::now();
        auto ts = std::chrono::duration_cast<Ms>(Clock::now() - t_start);

        // 往返逻辑：到位后切换方向
        if (std::abs(fb.pos - q_target) < 0.05f) {
            std::swap(q_start, q_target);
            detector.reset();
            std::cout << "\n[INFO] 换向 → " << q_target << " rad\n";
        }

        // 低刚度位置命令（模拟夹爪缓慢闭合）
        pack_mit(cfg, cfg.approach_kp, cfg.kd,
                 q_target, 0.0f, 0.0f, payload);
        send_frame(sock, cfg.can_id, payload, 8);

        if (recv_fb(sock, cfg.mst_id, fb, cfg) && fb.valid) {
            float q_err = q_target - fb.pos;
            int   det   = detector.update(q_err, fb.tau);

            bool c_tau = (det == 1 || det == 3);
            bool c_pos = (det == 2 || det == 3);

            records.push_back({ts.count(),
                               q_target, fb.pos, q_err,
                               fb.vel, fb.tau,
                               c_tau ? 1 : 0,
                               c_pos ? 1 : 0,
                               label});

            // 记录首次触发时刻
            if (c_tau && contact_ts_tau < 0) {
                contact_ts_tau = ts.count();
                contact_q_error_at_tau = q_err;
            }
            if (c_pos && contact_ts_pos < 0) {
                contact_ts_pos = ts.count();
                contact_tau_at_pos = fb.tau;
            }

            // 实时打印
            std::cout << "\r  t=" << std::setw(5) << ts.count() << "ms"
                      << "  q_err=" << std::setw(8) << std::setprecision(4) << q_err
                      << " rad  tau=" << std::setw(7) << fb.tau << " Nm"
                      << "  [tau:" << (c_tau ? "✓" : "·")
                      << " pos:" << (c_pos ? "✓" : "·") << "]"
                      << std::flush;
        }

        std::this_thread::sleep_until(
            t0 + std::chrono::duration_cast<Clock::duration>(period));
    }

    std::cout << "\n\n";

    // ── 4. 统计摘要 ────────────────────────────────────────
    // 自由段（前 1s）vs 接触段（最后 1s）
    auto baseline_end = records.empty() ? int64_t(0) : std::min(int64_t(1000), records.back().ts_ms / 4);
    float tau_baseline_mean = 0.0f, q_err_free_mean = 0.0f;
    int   n_free = 0;
    for (const auto& r : records) {
        if (r.ts_ms < baseline_end) {
            tau_baseline_mean += r.tau_fb;
            q_err_free_mean   += r.q_error;
            ++n_free;
        }
    }
    if (n_free > 0) {
        tau_baseline_mean /= n_free;
        q_err_free_mean   /= n_free;
    }

    float tau_contact_max = 0.0f, q_err_contact_max = 0.0f;
    for (const auto& r : records) {
        if (std::abs(r.tau_fb)  > std::abs(tau_contact_max))  tau_contact_max  = r.tau_fb;
        if (std::abs(r.q_error) > std::abs(q_err_contact_max)) q_err_contact_max = r.q_error;
    }

    std::cout << "════════════════════════════════════════\n"
              << " 实验结果摘要：" << label << "\n"
              << "════════════════════════════════════════\n\n"
              << "  【自由运动段】\n"
              << "    tau_fb 基线均值:    " << tau_baseline_mean << " Nm\n"
              << "    q_error 基线均值:   " << q_err_free_mean   << " rad\n\n"
              << "  【接触后峰值】\n"
              << "    tau_fb 最大值:      " << tau_contact_max   << " Nm\n"
              << "    q_error 最大值:     " << q_err_contact_max << " rad\n\n"
              << "  【检测触发时刻】\n"
              << "    tau 判据触发: ";
    if (contact_ts_tau >= 0)
        std::cout << contact_ts_tau << " ms  (q_error=" << contact_q_error_at_tau << " rad)\n";
    else
        std::cout << "未触发（增大 tau_threshold 或检查接触）\n";

    std::cout << "    pos 判据触发: ";
    if (contact_ts_pos >= 0)
        std::cout << contact_ts_pos << " ms  (tau_fb=" << contact_tau_at_pos << " Nm)\n";
    else
        std::cout << "未触发（减小 q_err_threshold 或延长实验）\n";

    // 给出阈值调整建议
    std::cout << "\n  【阈值调整建议】\n";
    if (contact_ts_pos < 0 && std::abs(q_err_contact_max) > 0.01f)
        std::cout << "    q_err_threshold 建议调整为: "
                  << std::abs(q_err_contact_max) * 0.5f << " ~ "
                  << std::abs(q_err_contact_max) * 0.8f << " rad\n";
    if (contact_ts_tau < 0 && std::abs(tau_contact_max) > 0.1f)
        std::cout << "    tau_threshold 建议调整为: "
                  << (tau_baseline_mean + std::abs(tau_contact_max)) * 0.5f << " Nm\n";

    std::cout << "\n  → 将上述数值填入 docs/experiments/exp3_contact_detection.md\n"
              << "════════════════════════════════════════\n";

    write_csv(cfg.out, records);
}

// ─────────────────────────────────────────────────────────────
// 参数解析
// ─────────────────────────────────────────────────────────────
static Config parse_args(int argc, char** argv)
{
    Config cfg;
    if (argc < 2) {
        std::cout
            << "Usage: contact_sim <mode> [options]\n"
            << "  mode:  soft | hard\n"
            << "  --iface      <can0>   CAN 接口\n"
            << "  --id         <0x01>   电机 CAN ID\n"
            << "  --kp         <30.0>   位置控制 kp\n"
            << "  --kd         <1.0>    阻尼 kd\n"
            << "  --approach-kp <5.0>  接近阶段 kp（低刚度）\n"
            << "  --travel     <1.0>    推进距离 (rad)\n"
            << "  --tau-max    <3.0>    力矩截断 (Nm)\n"
            << "  --tau-thresh <0.24>   tau 检测阈值 (Nm)\n"
            << "  --q-thresh   <0.05>   位置误差阈值 (rad)\n"
            << "  --window     <10>     滑窗帧数\n"
            << "  --out        <file>   CSV 输出\n"
            << "  --pmax <12.5> --vmax <30.0> --tmax <10.0>\n";
        std::exit(0);
    }
    cfg.mode = argv[1];
    for (int i = 2; i < argc - 1; ++i) {
        std::string k = argv[i], v = argv[i+1]; ++i;
        if      (k == "--iface")       cfg.iface          = v;
        else if (k == "--id")          cfg.can_id         = static_cast<uint32_t>(std::stoi(v,nullptr,0));
        else if (k == "--mst")         cfg.mst_id         = static_cast<uint32_t>(std::stoi(v,nullptr,0));
        else if (k == "--kp")          cfg.kp             = std::stof(v);
        else if (k == "--kd")          cfg.kd             = std::stof(v);
        else if (k == "--approach-kp") cfg.approach_kp    = std::stof(v);
        else if (k == "--travel")      cfg.travel_rad     = std::stof(v);
        else if (k == "--tau-max")     cfg.tau_max        = std::stof(v);
        else if (k == "--tau-thresh")  cfg.tau_threshold  = std::stof(v);
        else if (k == "--q-thresh")    cfg.q_err_threshold= std::stof(v);
        else if (k == "--window")      cfg.window_size    = std::stoi(v);
        else if (k == "--out")         cfg.out            = v;
        else if (k == "--pmax")        cfg.Q_MAX          = std::stof(v);
        else if (k == "--vmax")        cfg.DQ_MAX         = std::stof(v);
        else if (k == "--tmax")        cfg.TAU_MAX        = std::stof(v);
    }
    if (cfg.out.empty())
        cfg.out = "csv/exp3_" + cfg.mode + ".csv";
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
              << " DM4310 接触检测阈值标定工具 - Task 3\n"
              << "===========================================\n"
              << "  接口: " << cfg.iface
              << "  电机 ID: 0x" << std::hex << cfg.can_id << std::dec << "\n"
              << "  模式: " << cfg.mode << "\n"
              << "  tau_threshold: " << cfg.tau_threshold << " Nm"
              << "  q_err_threshold: " << cfg.q_err_threshold << " rad\n"
              << "  window: " << cfg.window_size << " 帧\n\n"
              << "  ⚠️  电机将向前旋转 " << cfg.travel_rad << " rad\n"
              << "      确保旋转方向安全后再继续\n\n";

    if (std::system("mkdir -p csv") != 0)
        std::cerr << "[WARN] cannot create csv dir\n";

    int sock = open_can(cfg.iface);
    if (sock < 0) {
        std::cerr << "[ERROR] 无法打开 " << cfg.iface
                  << "\n  sudo ip link set " << cfg.iface
                  << " up type can bitrate 1000000\n";
        return 1;
    }

    if (cfg.mode == "soft" || cfg.mode == "hard")
        run_experiment(sock, cfg, cfg.mode);
    else {
        std::cerr << "[ERROR] 未知模式: " << cfg.mode << "\n";
        close(sock); return 1;
    }

    for (int i = 0; i < 5; ++i) { send_disable(sock, cfg.can_id); usleep(2000); }
    close(sock);
    std::cout << "[OK] 安全退出\n";
    return 0;
}