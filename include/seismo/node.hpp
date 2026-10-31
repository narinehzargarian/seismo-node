// The sensor node: a timed acquisition thread, a processing thread, and an alert thread.
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#include "seismo/dsp.hpp"
#include "seismo/lis3dh.hpp"
#include "seismo/ring_buffer.hpp"

namespace seismo {

// How the node talks to the outside world. A fake one is used in tests.
class Transport {
public:
    virtual ~Transport() = default;
    virtual void send_waveform(std::span<const std::uint8_t> packet) = 0;  // Best effort (UDP).
    virtual bool send_alert(const std::string& json_line) = 0;             // Reliable (TCP).
};

struct NodeConfig {
    double speed = 1.0;              // Values above 1 run faster than real time, for simulation.
    std::uint64_t duration_ticks = 0;  // 0 runs until stop is requested.
    std::size_t batch_samples = 25;
    double min_event_seconds = 2.0;  // Shorter events are reported as transients.
    std::function<void(std::uint64_t)> on_tick;  // Lets the simulated sensor follow time.
};

struct NodeStats {
    std::atomic<std::uint64_t> samples{0};
    std::atomic<std::uint64_t> read_errors{0};
    std::atomic<std::uint64_t> overruns{0};
    std::atomic<std::uint64_t> deadline_misses{0};
    std::atomic<std::uint64_t> packets_sent{0};
    std::atomic<std::uint64_t> alerts_dropped{0};
};

class Node {
public:
    Node(RegisterBus& bus, Transport& transport, NodeConfig config);

    void run();  // Blocks until the configured duration ends or stop is requested.
    void request_stop() { stop_.store(true); }
    const NodeStats& stats() const { return stats_; }

private:
    enum class Health : std::uint8_t { BusError, Stuck, Recovered };
    struct HealthEvent {
        Health kind = Health::Recovered;
        std::uint64_t t_us = 0;
    };

    void acquisition_loop();
    void processing_loop();
    void alert_loop();

    void process_sample(const Sample& sample);
    void flush_batch();
    void emit_alert(std::string json_line);
    void report_health(const HealthEvent& event);
    void finish_event(std::uint64_t t_us);

    Lis3dh driver_;
    Transport& transport_;
    NodeConfig config_;
    NodeStats stats_;

    std::atomic<bool> stop_{false};
    std::atomic<bool> acquisition_done_{false};
    std::atomic<bool> processing_done_{false};

    // Lock-free hand-off from acquisition to processing. Holds about 20 s at 100 Hz.
    RingBuffer<Sample, 2048> samples_;
    RingBuffer<HealthEvent, 64> health_;

    // Owned by the processing thread.
    std::array<DcBlocker, 3> filters_;
    StaLtaTrigger trigger_;
    std::vector<Sample> batch_;
    std::array<std::vector<float>, 3> event_axes_;
    std::uint64_t last_t_us_ = 0;
    bool have_last_ = false;
    std::uint64_t event_start_us_ = 0;
    float event_peak_mg_ = 0.0F;
    std::uint32_t seq_ = 0;

    // Alerts can block on the network, so they get their own thread and a bounded queue.
    std::mutex alert_mutex_;
    std::condition_variable alert_cv_;
    std::deque<std::string> alerts_;
};

}  // namespace seismo
