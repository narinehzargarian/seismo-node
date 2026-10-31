// The node executable. Reads a real LIS3DH over I2C, or a simulated one with --sim.
#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include "seismo/linux_i2c.hpp"
#include "seismo/net.hpp"
#include "seismo/node.hpp"
#include "seismo/sim_sensor.hpp"

namespace {

seismo::Node* g_node = nullptr;

void on_signal(int) {
    if (g_node != nullptr) {
        g_node->request_stop();  // Only sets an atomic flag, which is safe in a signal handler.
    }
}

void usage() {
    std::puts(
        "usage: seismo-node [options]\n"
        "  --host HOST         receiver host (default 127.0.0.1)\n"
        "  --udp-port PORT     waveform port (default 9000)\n"
        "  --tcp-port PORT     alert port (default 9001)\n"
        "  --i2c DEVICE        real sensor, e.g. /dev/i2c-1\n"
        "  --i2c-addr ADDR     I2C address (default 0x18)\n"
        "  --sim               simulated sensor\n"
        "  --speed N           simulation speed multiplier (default 1)\n"
        "  --duration S        stop after S seconds of sensor time\n"
        "  --quake-at S        simulation: earthquake at S seconds\n"
        "  --slam-at S         simulation: door slam at S seconds\n"
        "  --bus-fault-at S    simulation: bus failure at S seconds\n"
        "  --stuck-at S        simulation: stuck sensor at S seconds\n"
        "  --drop-every N      drop every Nth UDP packet, to test loss detection");
}

}  // namespace

int main(int argc, char** argv) {
    std::string host = "127.0.0.1";
    std::uint16_t udp_port = 9000;
    std::uint16_t tcp_port = 9001;
    std::string i2c_device;
    std::uint8_t i2c_addr = 0x18;
    bool use_sim = false;
    std::uint32_t drop_every = 0;
    double duration_s = 0.0;
    seismo::SimScenario scenario;
    seismo::NodeConfig config;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", arg.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--host") host = next();
        else if (arg == "--udp-port") udp_port = static_cast<std::uint16_t>(std::atoi(next()));
        else if (arg == "--tcp-port") tcp_port = static_cast<std::uint16_t>(std::atoi(next()));
        else if (arg == "--i2c") i2c_device = next();
        else if (arg == "--i2c-addr") i2c_addr = static_cast<std::uint8_t>(std::strtol(next(), nullptr, 0));
        else if (arg == "--sim") use_sim = true;
        else if (arg == "--speed") config.speed = std::max(0.1, std::atof(next()));
        else if (arg == "--duration") duration_s = std::atof(next());
        else if (arg == "--quake-at") scenario.quake_at_s = std::atof(next());
        else if (arg == "--slam-at") scenario.slam_at_s = std::atof(next());
        else if (arg == "--bus-fault-at") scenario.bus_fault_at_s = std::atof(next());
        else if (arg == "--stuck-at") scenario.stuck_at_s = std::atof(next());
        else if (arg == "--drop-every") drop_every = static_cast<std::uint32_t>(std::atoi(next()));
        else {
            usage();
            return arg == "--help" ? 0 : 2;
        }
    }
    if (use_sim == !i2c_device.empty()) {
        std::fputs("choose exactly one of --sim or --i2c DEVICE\n", stderr);
        return 2;
    }
    config.duration_ticks = static_cast<std::uint64_t>(duration_s * 100.0);

    std::signal(SIGPIPE, SIG_IGN);  // Failed sends return an error instead of killing the process.

    try {
        std::unique_ptr<seismo::RegisterBus> bus;
        if (use_sim) {
            auto sim = std::make_unique<seismo::SimulatedLis3dh>(scenario);
            config.on_tick = [raw = sim.get()](std::uint64_t tick) { raw->set_tick(tick); };
            bus = std::move(sim);
        } else {
            bus = std::make_unique<seismo::LinuxI2cBus>(i2c_device, i2c_addr);
        }

        seismo::SocketTransport transport(host, udp_port, tcp_port, drop_every);
        seismo::Node node(*bus, transport, config);
        g_node = &node;
        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);

        node.run();

        g_node = nullptr;
        const auto& stats = node.stats();
        std::printf(
            R"({"type":"node_stats","samples":%llu,"read_errors":%llu,"overruns":%llu,"deadline_misses":%llu,"packets":%llu,"alerts_dropped":%llu})"
            "\n",
            static_cast<unsigned long long>(stats.samples.load()),
            static_cast<unsigned long long>(stats.read_errors.load()),
            static_cast<unsigned long long>(stats.overruns.load()),
            static_cast<unsigned long long>(stats.deadline_misses.load()),
            static_cast<unsigned long long>(stats.packets_sent.load()),
            static_cast<unsigned long long>(stats.alerts_dropped.load()));
    } catch (const std::exception& error) {
        std::fprintf(stderr, "error: %s\n", error.what());
        return 1;
    }
    return 0;
}
