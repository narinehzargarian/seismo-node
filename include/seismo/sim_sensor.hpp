// A software stand-in for the LIS3DH chip. It exposes the same registers as the real part,
// so the driver and everything above it run unchanged, including injected hardware faults.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <random>

#include "seismo/lis3dh.hpp"

namespace seismo {

struct SimScenario {
    std::optional<double> quake_at_s;
    std::optional<double> slam_at_s;      // A short, sharp knock, like a door slamming.
    std::optional<double> bus_fault_at_s; // The bus stops responding.
    std::optional<double> stuck_at_s;     // The chip keeps returning the same value.
    double fault_duration_s = 4.0;
    unsigned seed = 42;
};

class SimulatedLis3dh final : public RegisterBus {
public:
    explicit SimulatedLis3dh(SimScenario scenario);

    // Sets simulated time. Each tick is 10 ms.
    void set_tick(std::uint64_t tick) { tick_ = tick; }

    bool read(std::uint8_t reg, std::uint8_t* out, std::size_t len) override;
    bool write(std::uint8_t reg, std::uint8_t value) override;

private:
    double seconds() const { return static_cast<double>(tick_) * 0.01; }
    bool in_window(const std::optional<double>& start) const;
    std::array<double, 3> acceleration_mg();

    SimScenario scenario_;
    std::mt19937 rng_;
    std::normal_distribution<double> noise_{0.0, 2.0};
    std::uint64_t tick_ = 0;
    std::uint8_t ctrl1_ = 0;
    std::uint8_t ctrl4_ = 0;
    std::array<double, 3> last_{};
};

}  // namespace seismo
