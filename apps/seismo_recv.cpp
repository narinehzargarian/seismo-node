// Ground-side receiver. Takes waveform packets over UDP and alerts over TCP,
// tracks packet loss, and prints JSON lines. Can also save the waveform to CSV.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "seismo/protocol.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

int bind_socket(int type, std::uint16_t port) {
    const int fd = socket(AF_INET, type, 0);
    if (fd < 0) {
        return -1;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (type == SOCK_DGRAM) {
        int buffer = 1 << 20;  // A larger buffer means fewer drops during bursts.
        setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buffer, sizeof(buffer));
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

}  // namespace

int main(int argc, char** argv) {
    std::uint16_t udp_port = 9000;
    std::uint16_t tcp_port = 9001;
    std::string csv_path;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string arg = argv[i];
        if (arg == "--udp-port") udp_port = static_cast<std::uint16_t>(std::atoi(argv[i + 1]));
        else if (arg == "--tcp-port") tcp_port = static_cast<std::uint16_t>(std::atoi(argv[i + 1]));
        else if (arg == "--csv") csv_path = argv[i + 1];
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    const int udp_fd = bind_socket(SOCK_DGRAM, udp_port);
    const int listen_fd = bind_socket(SOCK_STREAM, tcp_port);
    if (udp_fd < 0 || listen_fd < 0 || listen(listen_fd, 4) != 0) {
        std::fputs("error: could not bind ports\n", stderr);
        return 1;
    }

    std::ofstream csv;
    if (!csv_path.empty()) {
        csv.open(csv_path);
        csv << "t_s,x_mg,y_mg,z_mg\n";
    }

    std::printf(R"({"type":"ready","udp_port":%u,"tcp_port":%u})" "\n", udp_port, tcp_port);
    std::fflush(stdout);

    seismo::protocol::GapTracker tracker;
    std::uint64_t corrupt = 0;
    std::uint64_t samples = 0;
    int client_fd = -1;
    std::string line_buffer;
    std::array<std::uint8_t, 2048> buffer{};

    const auto drain_lines = [&line_buffer] {
        std::size_t newline = 0;
        while ((newline = line_buffer.find('\n')) != std::string::npos) {
            std::puts(line_buffer.substr(0, newline).c_str());
            std::fflush(stdout);
            line_buffer.erase(0, newline + 1);
        }
    };

    while (g_stop == 0) {
        std::array<pollfd, 3> fds{{{udp_fd, POLLIN, 0}, {listen_fd, POLLIN, 0}, {client_fd, POLLIN, 0}}};
        const int ready = poll(fds.data(), client_fd >= 0 ? 3 : 2, 100);
        if (ready <= 0) {
            continue;
        }

        if ((fds[0].revents & POLLIN) != 0) {
            const ssize_t n = recv(udp_fd, buffer.data(), buffer.size(), 0);
            if (n > 0) {
                const auto packet = seismo::protocol::decode({buffer.data(), static_cast<std::size_t>(n)});
                if (!packet) {
                    ++corrupt;
                } else {
                    tracker.observe(packet->seq);
                    samples += packet->samples.size();
                    if (csv.is_open()) {
                        for (const auto& s : packet->samples) {
                            csv << static_cast<double>(s.t_us) / 1e6 << ',' << s.x_mg << ',' << s.y_mg << ','
                                << s.z_mg << '\n';
                        }
                    }
                }
            }
        }

        if ((fds[1].revents & POLLIN) != 0) {
            const int accepted = accept(listen_fd, nullptr, nullptr);
            if (accepted >= 0) {
                if (client_fd >= 0) {
                    close(client_fd);  // A reconnecting node replaces its old connection.
                }
                client_fd = accepted;
                line_buffer.clear();
            }
        }

        if (client_fd >= 0 && (fds[2].revents & (POLLIN | POLLHUP)) != 0) {
            const ssize_t n = recv(client_fd, buffer.data(), buffer.size(), 0);
            if (n > 0) {
                line_buffer.append(reinterpret_cast<const char*>(buffer.data()), static_cast<std::size_t>(n));
                drain_lines();
            } else {
                close(client_fd);
                client_fd = -1;
            }
        }
    }

    std::printf(R"({"type":"recv_stats","packets":%llu,"samples":%llu,"lost":%llu,"duplicates":%llu,"reordered":%llu,"corrupt":%llu})"
                "\n",
                static_cast<unsigned long long>(tracker.received()), static_cast<unsigned long long>(samples),
                static_cast<unsigned long long>(tracker.lost()), static_cast<unsigned long long>(tracker.duplicates()),
                static_cast<unsigned long long>(tracker.reordered()), static_cast<unsigned long long>(corrupt));
    if (client_fd >= 0) {
        close(client_fd);
    }
    close(listen_fd);
    close(udp_fd);
    return 0;
}
