#include "dm_motor_driver/socketcan.hpp"

#include <cstring>
#include <cerrno>
#include <cstdio>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace dm_motor_driver {

SocketCAN::SocketCAN(const std::string& iface, bool use_canfd)
    : iface_(iface), use_canfd_(use_canfd), fd_(-1)
{}

SocketCAN::~SocketCAN()
{
    close();
}

bool SocketCAN::open()
{
    if (fd_ >= 0) return true;  // already open

    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        perror("[SocketCAN] socket()");
        return false;
    }

    struct ifreq ifr{};
    strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("[SocketCAN] ioctl(SIOCGIFINDEX)");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("[SocketCAN] bind()");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    if (use_canfd_) {
        int enable_fd = 1;
        if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                         &enable_fd, sizeof(enable_fd)) < 0) {
            perror("[SocketCAN] setsockopt(CAN_RAW_FD_FRAMES)");
            ::close(fd_);
            fd_ = -1;
            return false;
        }
    }

    // Default receive timeout: 200 ms so recv() doesn't block indefinitely
    struct timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = 200000;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return true;
}

void SocketCAN::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SocketCAN::send(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    if (fd_ < 0) return false;

    if (use_canfd_) {
        struct canfd_frame frame{};
        frame.can_id = can_id & CAN_SFF_MASK;
        frame.len    = (len > CANFD_MAX_DLEN) ? CANFD_MAX_DLEN : len;
        frame.flags  = CANFD_BRS;
        memcpy(frame.data, data, frame.len);

        ssize_t written = ::write(fd_, &frame, sizeof(struct canfd_frame));
        if (written != sizeof(struct canfd_frame)) {
            if (errno != ENOBUFS)
                perror("[SocketCAN] write() CAN FD frame");
            return false;
        }
    } else {
        struct can_frame frame{};
        frame.can_id  = can_id & CAN_SFF_MASK;
        frame.can_dlc = (len > CAN_MAX_DLEN) ? CAN_MAX_DLEN : len;
        memcpy(frame.data, data, frame.can_dlc);

        ssize_t written = ::write(fd_, &frame, sizeof(struct can_frame));
        if (written != sizeof(struct can_frame)) {
            if (errno != ENOBUFS)
                perror("[SocketCAN] write() CAN frame");
            return false;
        }
    }
    return true;
}

bool SocketCAN::recv(uint32_t& can_id, uint8_t* data, uint8_t& len, int timeout_ms)
{
    if (fd_ < 0) return false;

    // Override the default receive timeout if a specific value is requested
    if (timeout_ms >= 0) {
        struct timeval tv{};
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    if (use_canfd_) {
        struct canfd_frame frame{};
        ssize_t nbytes = ::read(fd_, &frame, sizeof(struct canfd_frame));
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                return false;  // timeout
            perror("[SocketCAN] read() CAN FD");
            return false;
        }
        can_id = frame.can_id & CAN_SFF_MASK;
        len    = static_cast<uint8_t>(frame.len);
        memcpy(data, frame.data, frame.len);
    } else {
        struct can_frame frame{};
        ssize_t nbytes = ::read(fd_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                return false;
            perror("[SocketCAN] read() CAN");
            return false;
        }
        can_id = frame.can_id & CAN_SFF_MASK;
        len    = frame.can_dlc;
        memcpy(data, frame.data, frame.can_dlc);
    }
    return true;
}

} // namespace dm_motor_driver
