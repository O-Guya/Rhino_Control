/**
 * dm_socketcan_demo.cpp
 *
 * 达妙 DM4310 SocketCAN 最小验证 Demo
 * 通过 Linux SocketCAN (can0) 以 MIT 模式控制 DM4310 电机。
 *
 * 默认配置：
 *   电机 CAN ID  = 0x01
 *   电机 MST ID  = 0x11（反馈帧使用此 ID）
 *   CAN 接口      = can0
 *
 * 协议来源：docs/motor-controller/src/protocol/damiao.cpp
 *          docs/motor-controller/src/main.cpp
 *
 * 编译：
 *   mkdir -p build && cd build && cmake .. && make -j4
 *
 * 运行（需要 CAP_NET_RAW 权限）：
 *   sudo ./dm_socketcan_demo
 */

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cerrno>
#include <atomic>
#include <thread>
#include <chrono>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// ─────────────────────────────────────────────────────────────────────────────
// 配置区：根据实际情况修改
// ─────────────────────────────────────────────────────────────────────────────
static const char* CAN_IFACE  = "can0";
static const uint32_t MOTOR_CAN_ID = 0x01;   // 电机指令 CAN ID
static const uint32_t MOTOR_MST_ID = 0x11;   // 电机反馈 CAN ID (mst_id)

// DM4310 限位参数（来源：damiao.cpp:9）
static const float Q_MAX   = 12.5f;   // rad
static const float DQ_MAX  = 30.0f;   // rad/s
static const float TAU_MAX = 10.0f;   // Nm

// ─────────────────────────────────────────────────────────────────────────────
// 量化辅助函数（来源：damiao.cpp:305-311, main.cpp:21-27）
// ─────────────────────────────────────────────────────────────────────────────

// 浮点 → 无符号定点（线性映射）
static uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bits)
{
    float span = xmax - xmin;
    float norm = (x - xmin) / span;
    return static_cast<uint16_t>(norm * ((1 << bits) - 1));
}

// 无符号定点 → 浮点
static float uint_to_float(uint16_t x, float xmin, float xmax, uint8_t bits)
{
    float span = xmax - xmin;
    float norm = static_cast<float>(x) / static_cast<float>((1 << bits) - 1);
    return norm * span + xmin;
}

// ─────────────────────────────────────────────────────────────────────────────
// MIT 帧打包（来源：damiao.cpp:302-348）
//
// 8 字节布局：
//   [0]    q_uint[15:8]
//   [1]    q_uint[7:0]
//   [2]    dq_uint[11:4]
//   [3]    dq_uint[3:0] | kp_uint[11:8]
//   [4]    kp_uint[7:0]
//   [5]    kd_uint[11:4]
//   [6]    kd_uint[3:0] | tau_uint[11:8]
//   [7]    tau_uint[7:0]
// ─────────────────────────────────────────────────────────────────────────────
static void pack_mit_frame(float kp, float kd,
                           float q, float dq, float tau,
                           uint8_t out[8])
{
    uint16_t q_uint   = float_to_uint(q,   -Q_MAX,   Q_MAX,   16);
    uint16_t dq_uint  = float_to_uint(dq,  -DQ_MAX,  DQ_MAX,  12);
    uint16_t kp_uint  = float_to_uint(kp,  0.0f,     500.0f,  12);
    uint16_t kd_uint  = float_to_uint(kd,  0.0f,     5.0f,    12);
    uint16_t tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12);

    out[0] = (q_uint >> 8) & 0xFF;
    out[1] =  q_uint       & 0xFF;
    out[2] =  dq_uint >> 4;
    out[3] = ((dq_uint  & 0xF) << 4) | ((kp_uint >> 8) & 0xF);
    out[4] =  kp_uint  & 0xFF;
    out[5] =  kd_uint  >> 4;
    out[6] = ((kd_uint  & 0xF) << 4) | ((tau_uint >> 8) & 0xF);
    out[7] =  tau_uint & 0xFF;
}

// ─────────────────────────────────────────────────────────────────────────────
// 反馈帧解析（来源：main.cpp:46-60）
//
// 反馈帧（CAN ID = MOTOR_MST_ID）布局：
//   payload[0]    : 状态/错误字节（跳过）
//   payload[1-2]  : position，16-bit
//   payload[3-4]  : velocity 高12bit 跨两字节
//   payload[4-5]  : torque 低12bit 跨两字节
// ─────────────────────────────────────────────────────────────────────────────
static void decode_feedback(const uint8_t* payload,
                            float* pos, float* vel, float* tau)
{
    uint16_t q_uint   = (static_cast<uint16_t>(payload[1]) << 8) | payload[2];
    uint16_t dq_uint  = (static_cast<uint16_t>(payload[3]) << 4) | (payload[4] >> 4);
    uint16_t tau_uint = (static_cast<uint16_t>(payload[4] & 0xF) << 8) | payload[5];

    *pos = uint_to_float(q_uint,   -Q_MAX,   Q_MAX,   16);
    *vel = uint_to_float(dq_uint,  -DQ_MAX,  DQ_MAX,  12);
    *tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12);
}

// ─────────────────────────────────────────────────────────────────────────────
// SocketCAN 层
// ─────────────────────────────────────────────────────────────────────────────
static int g_sock = -1;

static int open_canfd_socket(const char* iface)
{
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("[ERROR] socket()");
        return -1;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("[ERROR] ioctl(SIOCGIFINDEX)");
        close(sock);
        return -1;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("[ERROR] bind()");
        close(sock);
        return -1;
    }

    printf("[SocketCAN] %s opened (fd=%d, classic CAN mode)\n", iface, sock);
    return sock;
}

// 发送一帧经典 CAN（标准 11-bit ID，8 字节数据）
static bool send_canfd_frame(int sock, uint32_t can_id,
                             const uint8_t* data, uint8_t len)
{
    struct can_frame frame{};
    frame.can_id  = can_id & CAN_SFF_MASK;  // 11-bit 标准帧
    frame.can_dlc = len > 8 ? 8 : len;
    memcpy(frame.data, data, frame.can_dlc);

    ssize_t written = write(sock, &frame, sizeof(struct can_frame));
    if (written != sizeof(struct can_frame)) {
        if (errno == ENOBUFS) {
            // TX 队列已满，跳过本帧，不打印错误（下个周期重试）
            return false;
        }
        perror("[ERROR] write() CAN frame");
        return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 高级命令封装
// ─────────────────────────────────────────────────────────────────────────────

// 切换电机模式为 MIT（写寄存器 RID=10，值=1）
// 来源：damiao.cpp:471-492（switchControlMode + write_motor_param）
static void send_switch_to_mit(int sock, uint32_t motor_can_id)
{
    uint8_t id_low  = motor_can_id & 0xFF;
    uint8_t id_high = (motor_can_id >> 8) & 0xFF;
    uint8_t payload[8] = {id_low, id_high, 0x55, 0x0A, 0x01, 0x00, 0x00, 0x00};
    printf("[TX] Switch mode to MIT (CAN ID 0x7FF)\n");
    send_canfd_frame(sock, 0x7FF, payload, 8);
}

// 发送使能命令（0xFC）
// 来源：damiao.cpp:276-280（control_cmd）
static void send_enable(int sock, uint32_t motor_can_id)
{
    uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    printf("[TX] Enable motor (CAN ID 0x%02X)\n", motor_can_id);
    send_canfd_frame(sock, motor_can_id, payload, 8);
}

// 发送失能命令（0xFD）
static void send_disable(int sock, uint32_t motor_can_id)
{
    uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    printf("[TX] Disable motor (CAN ID 0x%02X)\n", motor_can_id);
    send_canfd_frame(sock, motor_can_id, payload, 8);
}

// 发送 MIT 控制帧
static void send_mit_cmd(int sock, uint32_t motor_can_id,
                         float kp, float kd,
                         float q, float dq, float tau)
{
    uint8_t payload[8];
    pack_mit_frame(kp, kd, q, dq, tau, payload);
    send_canfd_frame(sock, motor_can_id, payload, 8);
}

// ─────────────────────────────────────────────────────────────────────────────
// 接收线程：持续读取 CAN 帧，解析反馈
// ─────────────────────────────────────────────────────────────────────────────
static std::atomic<bool> g_running{true};

static void recv_thread_fn(int sock)
{
    struct can_frame frame{};
    while (g_running.load()) {
        ssize_t nbytes = read(sock, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            if (errno == EINTR) break;
            perror("[ERROR] read() CAN frame");
            break;
        }

        uint32_t rx_id = frame.can_id & CAN_SFF_MASK;  // 去掉标志位

        if (rx_id == MOTOR_MST_ID && frame.can_dlc >= 6) {
            float pos, vel, tau;
            decode_feedback(frame.data, &pos, &vel, &tau);
            printf("[RX] ID=0x%02X | pos=%7.3f rad | vel=%7.3f rad/s | tau=%7.3f Nm\n",
                   rx_id, pos, vel, tau);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 信号处理
// ─────────────────────────────────────────────────────────────────────────────
static void signal_handler(int /*signum*/)
{
    g_running.store(false);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 1. 打开 SocketCAN 接口
    g_sock = open_canfd_socket(CAN_IFACE);
    if (g_sock < 0) {
        fprintf(stderr, "[ERROR] 无法打开 %s，请确认接口已 UP：\n"
                        "  sudo ip link set %s up type can bitrate 1000000\n",
                CAN_IFACE, CAN_IFACE);
        return 1;
    }

    // 2. 切换模式为 MIT
    send_switch_to_mit(g_sock, MOTOR_CAN_ID);
    usleep(5000);

    // 3. 使能电机（重发 5 次，间隔 2ms，与官方例程一致）
    for (int i = 0; i < 5; i++) {
        send_enable(g_sock, MOTOR_CAN_ID);
        usleep(2000);
    }

    // 4. 启动接收线程
    std::thread recv_thread(recv_thread_fn, g_sock);

    // 5. 控制循环 200Hz
    //    初始参数：kp=0, kd=0.5（轻阻尼，安全起步），q/dq/tau=0
    printf("[INFO] 控制循环启动 (200 Hz)，按 Ctrl+C 退出\n");

    using clock    = std::chrono::steady_clock;
    using duration = std::chrono::duration<double>;
    const duration cycle(0.005);  // 5ms = 200Hz

    while (g_running.load()) {
        auto t0 = clock::now();
        send_mit_cmd(g_sock, MOTOR_CAN_ID,
                     /*kp=*/0.0f, /*kd=*/0.5f,
                     /*q=*/0.0f,  /*dq=*/0.0f, /*tau=*/0.0f);
        std::this_thread::sleep_until(t0 + std::chrono::duration_cast<clock::duration>(cycle));
    }

    // 6. 退出：失能电机
    for (int i = 0; i < 5; i++) {
        send_disable(g_sock, MOTOR_CAN_ID);
        usleep(2000);
    }

    g_running.store(false);
    // 关闭 socket 让 recv_thread 的 read() 返回
    close(g_sock);
    g_sock = -1;

    if (recv_thread.joinable()) recv_thread.join();

    printf("[INFO] 程序安全退出\n");
    return 0;
}
