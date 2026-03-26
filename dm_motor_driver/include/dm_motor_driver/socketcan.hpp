#pragma once

#include <cstdint>
#include <string>

namespace dm_motor_driver {

/// RAII wrapper around a Linux SocketCAN socket.
/// Supports both classic CAN and CAN FD (with BRS).
class SocketCAN {
public:
    /// @param iface    e.g. "can0"
    /// @param use_canfd  true → enable CAN FD + BRS; false → classic CAN
    explicit SocketCAN(const std::string& iface, bool use_canfd = true);
    ~SocketCAN();

    // Non-copyable, non-movable
    SocketCAN(const SocketCAN&) = delete;
    SocketCAN& operator=(const SocketCAN&) = delete;

    /// Open and bind the socket. Returns true on success.
    bool open();

    /// Close the socket (also called by destructor).
    void close();

    /// Returns true if the socket is currently open.
    bool is_open() const { return fd_ >= 0; }

    /// Send a CAN frame.
    /// @param can_id  11-bit standard CAN ID
    /// @param data    payload bytes
    /// @param len     number of bytes to send (max 8 for classic CAN, 64 for CAN FD)
    /// @returns true on success
    bool send(uint32_t can_id, const uint8_t* data, uint8_t len);

    /// Receive one CAN frame (blocking with timeout).
    /// @param[out] can_id   received CAN ID
    /// @param[out] data     buffer (must be >= 64 bytes for CAN FD)
    /// @param[out] len      number of bytes received
    /// @param      timeout_ms  receive timeout; 0 = block forever
    /// @returns true if a frame was received; false on timeout or error
    bool recv(uint32_t& can_id, uint8_t* data, uint8_t& len, int timeout_ms = 200);

private:
    std::string iface_;
    bool        use_canfd_;
    int         fd_{-1};
};

} // namespace dm_motor_driver
