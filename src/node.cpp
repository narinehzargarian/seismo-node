#include "seismo/node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <thread>
#include <utility>

#include "seismo/protocol.hpp"

namespace seismo {

namespace {

using Clock = std::chrono::steady_clock;

constexpr std::uint64_t kTickUs = 10'000;  // 100 Hz
constexpr float kSampleRateHz = 100.0F;
constexpr int kMaxConsecutiveErrors = 5;
constexpr int kStuckSamples = 200;  // Real sensors always have noise, so 2 s of identical values is a fault.
constexpr std::uint64_t kMinBackoffTicks = 10;
constexpr std::uint64_t kMaxBackoffTicks = 160;
constexpr std::size_t kMaxQueuedAlerts = 256;
constexpr std::size_t kMaxEventSamples = 3000;  // Fixed cap: 30 s of samples for frequency analysis.

bool same_values(const Sample& a, const Sample& b) {
    return a.x_mg == b.x_mg && a.y_mg == b.y_mg && a.z_mg == b.z_mg;
}

}  // namespace

Node::Node(RegisterBus& bus, Transport& transport, NodeConfig config)
    : driver_(bus), transport_(transport), config_(std::move(config)), trigger_(StaLtaTrigger::Config{}) {
    batch_.reserve(config_.batch_samples);
    for (auto& axis : event_axes_) {
        axis.reserve(kMaxEventSamples);
    }
}

void Node::run() {
    std::thread alerts(&Node::alert_loop, this);
    std::thread processing(&Node::processing_loop, this);
    std::thread acquisition(&Node::acquisition_loop, this);
    // Shut down in pipeline order, so everything already captured still gets delivered.
    acquisition.join();
    processing.join();
    alerts.join();
}

void Node::acquisition_loop() {
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double, std::micro>(static_cast<double>(kTickUs) / config_.speed));

    bool sensor_ok = false;
    bool in_fault = false;
    bool reported_initial_failure = false;
    int consecutive_errors = 0;
    int identical_samples = 0;
    std::optional<Sample> previous;
    std::uint64_t next_retry_tick = 0;
    std::uint64_t backoff = kMinBackoffTicks;

    const auto enter_fault = [&](Health kind, std::uint64_t tick) {
        sensor_ok = false;
        in_fault = true;
        next_retry_tick = tick + backoff;
        previous.reset();
        identical_samples = 0;
        consecutive_errors = 0;
        health_.push(HealthEvent{kind, tick * kTickUs});
    };

    auto next_wake = Clock::now();
    for (std::uint64_t tick = 0; !stop_.load(std::memory_order_relaxed); ++tick) {
        if (config_.duration_ticks != 0 && tick >= config_.duration_ticks) {
            break;
        }
        if (config_.on_tick) {
            config_.on_tick(tick);
        }
        const std::uint64_t t_us = tick * kTickUs;

        if (!sensor_ok) {
            if (tick >= next_retry_tick) {
                if (driver_.init()) {
                    sensor_ok = true;
                    backoff = kMinBackoffTicks;
                    if (in_fault) {
                        in_fault = false;
                        health_.push(HealthEvent{Health::Recovered, t_us});
                    }
                } else {
                    if (!in_fault && !reported_initial_failure) {
                        reported_initial_failure = true;
                        in_fault = true;
                        health_.push(HealthEvent{Health::BusError, t_us});
                    }
                    // Exponential backoff, so a dead sensor isn't hammered with retries.
                    backoff = std::min(backoff * 2, kMaxBackoffTicks);
                    next_retry_tick = tick + backoff;
                }
            }
        } else if (const auto sample = driver_.read_sample(t_us)) {
            consecutive_errors = 0;
            identical_samples = previous && same_values(*previous, *sample) ? identical_samples + 1 : 0;
            previous = sample;
            if (identical_samples >= kStuckSamples) {
                enter_fault(Health::Stuck, tick);
            } else {
                stats_.samples.fetch_add(1, std::memory_order_relaxed);
                if (!samples_.push(*sample)) {
                    stats_.overruns.fetch_add(1, std::memory_order_relaxed);
                }
            }
        } else {
            stats_.read_errors.fetch_add(1, std::memory_order_relaxed);
            if (++consecutive_errors >= kMaxConsecutiveErrors) {
                enter_fault(Health::BusError, tick);
            }
        }

        // Absolute deadlines, so small delays don't add up into drift.
        next_wake += period;
        const auto now = Clock::now();
        if (now > next_wake) {
            stats_.deadline_misses.fetch_add(1, std::memory_order_relaxed);
            if (now - next_wake > period * 100) {
                next_wake = now;  // After a long stall, resynchronize instead of bursting.
            }
        } else {
            std::this_thread::sleep_until(next_wake);
        }
    }
    acquisition_done_.store(true);
}

void Node::processing_loop() {
    while (true) {
        // Read the flag first: if acquisition has finished and the queues are empty, nothing is left.
        const bool done = acquisition_done_.load();
        bool worked = false;
        while (const auto event = health_.pop()) {
            report_health(*event);
            worked = true;
        }
        for (int i = 0; i < 256; ++i) {
            const auto sample = samples_.pop();
            if (!sample) {
                break;
            }
            process_sample(*sample);
            worked = true;
        }
        if (done && !worked) {
            break;
        }
        if (!worked) {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    }
    flush_batch();
    processing_done_.store(true);
    alert_cv_.notify_all();
}

void Node::process_sample(const Sample& sample) {
    if (have_last_ && sample.t_us - last_t_us_ > kTickUs) {
        // Data was missed, so the filter state is stale. The batch can't span a gap either,
        // because the protocol assumes samples in a packet are evenly spaced.
        flush_batch();
        for (auto& filter : filters_) {
            filter.reset();
        }
    }
    last_t_us_ = sample.t_us;
    have_last_ = true;

    batch_.push_back(sample);
    if (batch_.size() >= config_.batch_samples) {
        flush_batch();
    }

    const float x = filters_[0].process(static_cast<float>(sample.x_mg));
    const float y = filters_[1].process(static_cast<float>(sample.y_mg));
    const float z = filters_[2].process(static_cast<float>(sample.z_mg));
    const float energy = x * x + y * y + z * z;

    const auto transition = trigger_.process(energy);
    if (transition == StaLtaTrigger::Transition::Started) {
        event_start_us_ = sample.t_us;
        event_peak_mg_ = 0.0F;
        for (auto& axis : event_axes_) {
            axis.clear();
        }
        char line[128];
        std::snprintf(line, sizeof(line), R"({"type":"event_start","t_s":%.2f,"ratio":%.1f})",
                      static_cast<double>(sample.t_us) / 1e6, static_cast<double>(trigger_.ratio()));
        emit_alert(line);
    }
    if (trigger_.active()) {
        event_peak_mg_ = std::max(event_peak_mg_, std::sqrt(energy));
        if (event_axes_[0].size() < kMaxEventSamples) {
            event_axes_[0].push_back(x);
            event_axes_[1].push_back(y);
            event_axes_[2].push_back(z);
        }
    }
    if (transition == StaLtaTrigger::Transition::Ended) {
        finish_event(sample.t_us);
    }
}

void Node::finish_event(std::uint64_t t_us) {
    const double duration_s = static_cast<double>(t_us - event_start_us_) / 1e6;
    char line[160];
    if (duration_s < config_.min_event_seconds) {
        std::snprintf(line, sizeof(line), R"({"type":"transient","t_s":%.2f,"duration_s":%.2f,"peak_mg":%.0f})",
                      static_cast<double>(event_start_us_) / 1e6, duration_s,
                      static_cast<double>(event_peak_mg_));
    } else {
        // Measure frequency on the axis that shook the most.
        const auto axis_energy = [](const std::vector<float>& axis) {
            double sum = 0.0;
            for (const float v : axis) {
                sum += static_cast<double>(v) * static_cast<double>(v);
            }
            return sum;
        };
        const auto strongest = std::max_element(event_axes_.begin(), event_axes_.end(),
                                                [&](const auto& a, const auto& b) { return axis_energy(a) < axis_energy(b); });
        const float hz = dominant_frequency_hz(*strongest, kSampleRateHz).value_or(0.0F);
        std::snprintf(line, sizeof(line),
                      R"({"type":"event_end","t_s":%.2f,"duration_s":%.2f,"peak_mg":%.0f,"dominant_hz":%.2f})",
                      static_cast<double>(event_start_us_) / 1e6, duration_s,
                      static_cast<double>(event_peak_mg_), static_cast<double>(hz));
    }
    emit_alert(line);
}

void Node::flush_batch() {
    if (batch_.empty()) {
        return;
    }
    transport_.send_waveform(protocol::encode(seq_++, batch_));
    stats_.packets_sent.fetch_add(1, std::memory_order_relaxed);
    batch_.clear();
}

void Node::report_health(const HealthEvent& event) {
    const char* name = "sensor_recovered";
    if (event.kind == Health::BusError) {
        name = "sensor_fault_bus";
    } else if (event.kind == Health::Stuck) {
        name = "sensor_fault_stuck";
    }
    char line[96];
    std::snprintf(line, sizeof(line), R"({"type":"%s","t_s":%.2f})", name, static_cast<double>(event.t_us) / 1e6);
    emit_alert(line);
}

void Node::emit_alert(std::string json_line) {
    {
        const std::lock_guard lock(alert_mutex_);
        if (alerts_.size() >= kMaxQueuedAlerts) {
            alerts_.pop_front();  // Bounded memory: if the network is down for long, drop the oldest.
            stats_.alerts_dropped.fetch_add(1, std::memory_order_relaxed);
        }
        alerts_.push_back(std::move(json_line));
    }
    alert_cv_.notify_one();
}

void Node::alert_loop() {
    int failures_after_done = 0;
    std::unique_lock lock(alert_mutex_);
    while (true) {
        alert_cv_.wait_for(lock, std::chrono::milliseconds(100),
                           [this] { return !alerts_.empty() || processing_done_.load(); });
        if (alerts_.empty()) {
            if (processing_done_.load()) {
                return;
            }
            continue;
        }
        const std::string line = alerts_.front();
        lock.unlock();
        const bool sent = transport_.send_alert(line);  // May block, so it runs without the lock.
        if (!sent) {
            // Keep the alert and retry, but don't hang forever at shutdown.
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        lock.lock();
        if (sent) {
            if (!alerts_.empty() && alerts_.front() == line) {
                alerts_.pop_front();
            }
        } else if (processing_done_.load() && ++failures_after_done >= 10) {
            stats_.alerts_dropped.fetch_add(alerts_.size(), std::memory_order_relaxed);
            return;
        }
    }
}

}  // namespace seismo
