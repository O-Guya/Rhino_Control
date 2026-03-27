#pragma once

/**
 * socketcan.hpp
 *
 * Linux SocketCAN 的 RAII 封装。
 *
 * 职责：
 *   - 管理 CAN socket 的生命周期（构造打开，析构自动关闭）
 *   - 提供发送和接收经典 CAN 帧（8 字节，标准 11-bit ID）的接口
 *   - 不包含任何协议逻辑，只负责字节的收发
 *
 * 使用示例：
 *   SocketCan can("can0");
 *   uint8_t data[8] = {0xFF, ...};
 *   can.send(0x01, data, 8);
 *
 *   CanFrame frame;
 *   if (can.recv(frame, 100)) {  // 100ms 超时
 *       // 处理 frame
 *   }
 */

#include <cstdint>
#include <stdexcept>
#include <string>

namespace dm_motor_driver {

// ─────────────────────────────────────────────────────────────
// CAN 帧数据结构
// ─────────────────────────────────────────────────────────────

/**
 * 经典 CAN 帧（标准帧，11-bit ID，最多 8 字节数据）
 */
struct CanFrame {
    uint32_t id  = 0;      ///< 11-bit 标准帧 ID（高位已屏蔽）
    uint8_t  dlc = 0;      ///< 数据长度（0~8）
    uint8_t  data[8] = {}; ///< 数据字节
};

// ─────────────────────────────────────────────────────────────
// SocketCan 类
// ─────────────────────────────────────────────────────────────

/**
 * RAII 风格的 SocketCAN 封装。
 *
 * 线程安全性：
 *   send() 和 recv() 可以在不同线程同时调用（内核保证 socket 读写原子性），
 *   但不能两个线程同时 send() 或同时 recv()。
 *   典型用法：一个发送线程 + 一个接收线程。
 */
class SocketCan {
public:
    /**
     * 打开指定 CAN 接口。
     *
     * @param iface  接口名，例如 "can0"
     * @param recv_timeout_ms  recv() 调用的超时时间（毫秒），
     *                         0 表示阻塞直到收到数据，
     *                         默认 200ms
     * @throws std::runtime_error  若接口不存在或权限不足
     *
     * 前置条件（需在 shell 中先执行）：
     *   sudo ip link set can0 up type can bitrate 1000000
     */
    explicit SocketCan(const std::string& iface,
                       int recv_timeout_ms = 200);

    /**
     * 析构时自动关闭 socket。
     * 安全：若 socket 已关闭则不做任何操作。
     */
    ~SocketCan();

    // 禁止拷贝（socket 是独占资源）
    SocketCan(const SocketCan&)            = delete;
    SocketCan& operator=(const SocketCan&) = delete;

    // 允许移动
    SocketCan(SocketCan&& other) noexcept;
    SocketCan& operator=(SocketCan&& other) noexcept;

    /**
     * 发送一帧经典 CAN（标准帧）。
     *
     * @param id    11-bit 帧 ID（超出部分被截断）
     * @param data  数据指针
     * @param len   数据长度（最多 8 字节，超出部分被截断）
     * @return true  发送成功
     * @return false TX 缓冲区满（ENOBUFS），可在下一周期重试
     * @throws std::runtime_error  其他发送错误
     */
    bool send(uint32_t id, const uint8_t* data, uint8_t len) const;

    /**
     * 接收一帧 CAN。
     *
     * 调用会阻塞至多 recv_timeout_ms 毫秒（构造时指定）。
     *
     * @param[out] frame  填充收到的帧
     * @return true   收到一帧（frame 已填充）
     * @return false  超时或被信号中断，未收到数据
     * @throws std::runtime_error  socket 读取错误（非超时类错误）
     */
    bool recv(CanFrame& frame) const;

    /**
     * 返回底层 socket 文件描述符。
     * 仅供高级用法（如 select/epoll），一般不需要调用。
     */
    int fd() const { return sock_; }

    /**
     * 返回 CAN 接口名。
     */
    const std::string& iface() const { return iface_; }

private:
    std::string iface_;   ///< CAN 接口名（如 "can0"）
    int         sock_;    ///< 底层 socket 文件描述符，-1 表示未打开
};

}  // namespace dm_motor_driver