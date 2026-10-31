#include "seismo/net.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace seismo {

namespace {

int connect_socket(const std::string& host, std::uint16_t port, int type) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = type;
    addrinfo* result = nullptr;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &result) != 0) {
        return -1;
    }
    int fd = -1;
    for (addrinfo* ai = result; ai != nullptr; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) {
            continue;
        }
        if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) {
            break;
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(result);
    return fd;
}

}  // namespace

SocketTransport::SocketTransport(std::string host, std::uint16_t udp_port, std::uint16_t tcp_port,
                                 std::uint32_t drop_every)
    : host_(std::move(host)), tcp_port_(tcp_port), drop_every_(drop_every) {
    udp_fd_ = connect_socket(host_, udp_port, SOCK_DGRAM);
    if (udp_fd_ < 0) {
        throw std::runtime_error("could not open UDP socket: " + std::string(std::strerror(errno)));
    }
}

SocketTransport::~SocketTransport() {
    close_tcp();
    if (udp_fd_ >= 0) {
        close(udp_fd_);
    }
}

void SocketTransport::send_waveform(std::span<const std::uint8_t> packet) {
    ++packet_count_;
    if (drop_every_ != 0 && packet_count_ % drop_every_ == 0) {
        return;
    }
    // Best effort by design: old waveform data isn't worth retrying. Errors such as
    // ECONNREFUSED just mean no receiver is listening yet.
    (void)send(udp_fd_, packet.data(), packet.size(), 0);
}

bool SocketTransport::ensure_tcp() {
    if (tcp_fd_ >= 0) {
        return true;
    }
    tcp_fd_ = connect_socket(host_, tcp_port_, SOCK_STREAM);
    if (tcp_fd_ < 0) {
        return false;
    }
    int one = 1;
    setsockopt(tcp_fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    timeval timeout{2, 0};  // A stalled receiver must not hang the alert thread forever.
    setsockopt(tcp_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    return true;
}

void SocketTransport::close_tcp() {
    if (tcp_fd_ >= 0) {
        close(tcp_fd_);
        tcp_fd_ = -1;
    }
}

bool SocketTransport::send_alert(const std::string& json_line) {
    if (!ensure_tcp()) {
        return false;
    }
    const std::string framed = json_line + "\n";
    std::size_t sent = 0;
    while (sent < framed.size()) {
        const ssize_t n = send(tcp_fd_, framed.data() + sent, framed.size() - sent, 0);
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            close_tcp();
            return false;  // The caller keeps the alert and retries on a new connection.
        }
        sent += static_cast<std::size_t>(n);
    }
    return true;
}

}  // namespace seismo
