// Runs the whole node in-process with the simulated chip and a fake network.
#include <gtest/gtest.h>

#include <mutex>
#include <string>
#include <vector>

#include "seismo/node.hpp"
#include "seismo/protocol.hpp"
#include "seismo/sim_sensor.hpp"

using namespace seismo;

namespace {

class FakeTransport final : public Transport {
public:
    void send_waveform(std::span<const std::uint8_t> packet) override {
        const std::lock_guard lock(mutex);
        const auto decoded = protocol::decode(packet);
        ASSERT_TRUE(decoded.has_value());
        tracker.observe(decoded->seq);
        samples += decoded->samples.size();
    }
    bool send_alert(const std::string& line) override {
        const std::lock_guard lock(mutex);
        if (failures_remaining > 0) {
            --failures_remaining;
            return false;
        }
        alerts.push_back(line);
        return true;
    }
    int count(const std::string& type) {
        const std::lock_guard lock(mutex);
        int total = 0;
        for (const auto& alert : alerts) {
            total += alert.find("\"type\":\"" + type + "\"") != std::string::npos ? 1 : 0;
        }
        return total;
    }

    std::mutex mutex;
    protocol::GapTracker tracker;
    std::size_t samples = 0;
    std::vector<std::string> alerts;
    int failures_remaining = 0;
};

struct RunResult {
    std::uint64_t overruns = 0;
    std::uint64_t read_errors = 0;
};

RunResult run_sim(SimScenario scenario, double seconds, FakeTransport& transport) {
    SimulatedLis3dh chip(scenario);
    NodeConfig config;
    config.speed = 100.0;
    config.duration_ticks = static_cast<std::uint64_t>(seconds * 100);
    config.on_tick = [&chip](std::uint64_t tick) { chip.set_tick(tick); };
    Node node(chip, transport, config);
    node.run();
    return RunResult{node.stats().overruns.load(), node.stats().read_errors.load()};
}

}  // namespace

TEST(Node, QuietSiteRaisesNoAlerts) {
    FakeTransport transport;
    const auto result = run_sim(SimScenario{}, 60.0, transport);
    EXPECT_TRUE(transport.alerts.empty()) << transport.alerts[0];
    EXPECT_EQ(result.overruns, 0U);
    EXPECT_EQ(transport.samples, 5999U);  // The first tick is spent configuring the chip.
    EXPECT_EQ(transport.tracker.lost(), 0U);
}

TEST(Node, DetectsEarthquakeAndMeasuresItsFrequency) {
    SimScenario scenario;
    scenario.quake_at_s = 35.0;
    FakeTransport transport;
    run_sim(scenario, 80.0, transport);
    EXPECT_EQ(transport.count("event_start"), 1);
    ASSERT_EQ(transport.count("event_end"), 1);
    const std::string& end = transport.alerts.back();
    const auto pos = end.find("dominant_hz\":");
    ASSERT_NE(pos, std::string::npos);
    EXPECT_NEAR(std::stof(end.substr(pos + 13)), 3.0F, 0.5F);
}

TEST(Node, ClassifiesDoorSlamAsTransient) {
    SimScenario scenario;
    scenario.slam_at_s = 35.0;
    FakeTransport transport;
    run_sim(scenario, 50.0, transport);
    EXPECT_EQ(transport.count("transient"), 1);
    EXPECT_EQ(transport.count("event_end"), 0);
}

TEST(Node, RecoversFromBusFailure) {
    SimScenario scenario;
    scenario.bus_fault_at_s = 10.0;
    FakeTransport transport;
    const auto result = run_sim(scenario, 30.0, transport);
    EXPECT_EQ(transport.count("sensor_fault_bus"), 1);
    EXPECT_EQ(transport.count("sensor_recovered"), 1);
    EXPECT_GT(result.read_errors, 0U);
    // Lost only about 4 s of samples, and kept streaming after the failure.
    EXPECT_GT(transport.samples, 2500U);
}

TEST(Node, DetectsStuckSensor) {
    SimScenario scenario;
    scenario.stuck_at_s = 10.0;
    FakeTransport transport;
    run_sim(scenario, 25.0, transport);
    EXPECT_GE(transport.count("sensor_fault_stuck"), 1);
    EXPECT_GE(transport.count("sensor_recovered"), 1);
}

TEST(Node, StillDetectsQuakeAfterRecoveringFromFault) {
    SimScenario scenario;
    scenario.bus_fault_at_s = 5.0;
    scenario.quake_at_s = 50.0;
    FakeTransport transport;
    run_sim(scenario, 90.0, transport);
    EXPECT_EQ(transport.count("event_end"), 1);
}

TEST(Node, RetriesAlertsWhenNetworkIsDown) {
    SimScenario scenario;
    scenario.quake_at_s = 35.0;
    FakeTransport transport;
    transport.failures_remaining = 3;
    run_sim(scenario, 60.0, transport);
    EXPECT_EQ(transport.count("event_start"), 1);
}
