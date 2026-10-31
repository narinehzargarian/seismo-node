// POSIX socket transport: UDP for waveform data, TCP for alerts that must arrive.
#pragma once

#include <cstdint>
#include <span>
#include <string>

#include "seismo/node.hpp"

namespace seismo {

class SocketTransport final : public Transport {
public:
    // drop_every > 0 deliberately skips every Nth packet, to test loss detection.
    SocketTransport(std::string host, std::uint16_t udp_port, std::uint16_t tcp_port, std::uint32_t drop_every = 0);
    ~SocketTransport() override;
    SocketTransport(const SocketTransport&) = delete;
    SocketTransport& operator=(const SocketTransport&) = delete;

    void send_waveform(std::span<const std::uint8_t> packet) override;
    bool send_alert(const std::string& json_line) override;

private:
    bool ensure_tcp();
    void close_tcp();

    std::string host_;
    std::uint16_t tcp_port_;
    std::uint32_t drop_every_;
    std::uint32_t packet_count_ = 0;
    int udp_fd_ = -1;
    int tcp_fd_ = -1;
};

}  // namespace seismo
