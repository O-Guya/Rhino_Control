/**
 * socketcan.cpp
 *
 * SocketCan 类的实现。
 * 封装 Linux SocketCAN 原始 API，提供 RAII 管理和简洁的收发接口。
 */

#include "dm_motor_driver/socketcan.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>   // std::swap

// Linux SocketCAN 头文件
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace dm_motor_driver {

// ─────────────────────────────────────────────────────────────
// 构造：打开 CAN 接口
// ─────────────────────────────────────────────────────────────

SocketCan::SocketCan(const std::string& iface, int recv_timeout_ms)
    : iface_(iface), sock_(-1)
{
    // 1. 创建原始 CAN socket
    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        throw std::runtime_error(
            "[SocketCan] socket() 失败: " + std::string(strerror(errno)) +
            "\n  需要 CAP_NET_RAW 权限（使用 sudo 运行）");
    }

    // 2. 通过接口名查询接口索引
    struct ifreq ifr;
    // 清零避免垃圾数据
    std::memset(&ifr, 0, sizeof(ifr));
    // 拷贝接口名（留一个字节给 '\0'）
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);

    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(sock_);
        sock_ = -1;
        throw std::runtime_error(
            "[SocketCan] 找不到接口 \"" + iface + "\": " + strerror(errno) +
            "\n  请先执行: sudo ip link set " + iface +
            " up type can bitrate 1000000");
    }

    // 3. 绑定到指定接口
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(sock_,
               reinterpret_cast<struct sockaddr*>(&addr),
               sizeof(addr)) < 0) {
        ::close(sock_);
        sock_ = -1;
        throw std::runtime_error(
            "[SocketCan] bind() 失败: " + std::string(strerror(errno)));
    }

    // 4. 设置接收超时
    //    作用：recv() 不会永久阻塞，可以定期检查退出标志
    if (recv_timeout_ms > 0) {
        struct timeval tv;
        tv.tv_sec  = recv_timeout_ms / 1000;
        tv.tv_usec = (recv_timeout_ms % 1000) * 1000;
        if (::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO,
                         &tv, sizeof(tv)) < 0) {
            // 非致命错误：超时设置失败不影响功能，只是 recv() 会阻塞更久
            // 不抛异常，打印警告即可
            // （生产代码中可以用 ROS_WARN，这里保持零 ROS 依赖）
        }
    }
}

// ─────────────────────────────────────────────────────────────
// 析构：关闭 socket
// ─────────────────────────────────────────────────────────────

SocketCan::~SocketCan()
{
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

// ─────────────────────────────────────────────────────────────
// 移动构造 / 移动赋值
// ─────────────────────────────────────────────────────────────

SocketCan::SocketCan(SocketCan&& other) noexcept
    : iface_(std::move(other.iface_))
    , sock_(other.sock_)
{
    // 转移所有权后，原对象不再持有 socket
    other.sock_ = -1;
}

SocketCan& SocketCan::operator=(SocketCan&& other) noexcept
{
    if (this != &other) {
        // 释放当前资源
        if (sock_ >= 0) {
            ::close(sock_);
        }
        iface_      = std::move(other.iface_);
        sock_       = other.sock_;
        other.sock_ = -1;
    }
    return *this;
}

// ─────────────────────────────────────────────────────────────
// send：发送一帧经典 CAN
// ─────────────────────────────────────────────────────────────

bool SocketCan::send(uint32_t id, const uint8_t* data, uint8_t len) const
{
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    // 只保留 11-bit 标准 ID（丢弃扩展帧标志位）
    frame.can_id  = id & CAN_SFF_MASK;
    // DLC 最大 8 字节
    frame.can_dlc = (len > 8) ? 8 : len;
    std::memcpy(frame.data, data, frame.can_dlc);

    ssize_t written = ::write(sock_, &frame, sizeof(struct can_frame));

    if (written == static_cast<ssize_t>(sizeof(struct can_frame))) {
        return true;  // 发送成功
    }

    if (errno == ENOBUFS) {
        // TX 队列暂时已满，非错误，调用方可在下一周期重试
        return false;
    }

    // 其他错误（接口 down、socket 已关闭等）
    throw std::runtime_error(
        "[SocketCan] write() 失败: " + std::string(strerror(errno)));
}

// ─────────────────────────────────────────────────────────────
// recv：接收一帧 CAN
// ─────────────────────────────────────────────────────────────

bool SocketCan::recv(CanFrame& out_frame) const
{
    struct can_frame frame;
    ssize_t nbytes = ::read(sock_, &frame, sizeof(struct can_frame));

    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            // 超时或被信号中断，正常情况
            return false;
        }
        throw std::runtime_error(
            "[SocketCan] read() 失败: " + std::string(strerror(errno)));
    }

    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
        // 收到的数据不完整，丢弃
        return false;
    }

    // 填充输出结构体
    // 去掉标志位（ERR/RTR/EFF），只保留 ID
    out_frame.id  = frame.can_id & CAN_SFF_MASK;
    out_frame.dlc = frame.can_dlc;
    std::memcpy(out_frame.data, frame.data, frame.can_dlc);

    return true;
}

}  // namespace dm_motor_driver