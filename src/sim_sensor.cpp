#include "seismo/sim_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace seismo {

namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;

// Synthetic earthquake: a weak, fast P wave first, then a stronger, slower S wave that decays.
void add_quake(std::array<double, 3>& a, double t) {
    constexpr double kPOnly = 3.0;
    if (t < 0.0 || t > 40.0) {
        return;
    }
    const double p = 15.0 * std::exp(-t / 6.0) * std::sin(kTwoPi * 8.0 * t);
    a[2] += p;
    if (t >= kPOnly) {
        const double s_t = t - kPOnly;
        const double envelope = 120.0 * std::min(s_t / 0.5, 1.0) * std::exp(-s_t / 4.0);
        a[0] += envelope * std::sin(kTwoPi * 3.0 * s_t);
        a[1] += 0.6 * envelope * std::cos(kTwoPi * 3.0 * s_t);
    }
}

}  // namespace

SimulatedLis3dh::SimulatedLis3dh(SimScenario scenario) : scenario_(scenario), rng_(scenario.seed) {}

bool SimulatedLis3dh::in_window(const std::optional<double>& start) const {
    return start && seconds() >= *start && seconds() < *start + scenario_.fault_duration_s;
}

std::array<double, 3> SimulatedLis3dh::acceleration_mg() {
    if (in_window(scenario_.stuck_at_s)) {
        return last_;
    }
    std::array<double, 3> a{noise_(rng_), noise_(rng_), 1000.0 + noise_(rng_)};  // 1 g of gravity on z.
    if (scenario_.quake_at_s) {
        add_quake(a, seconds() - *scenario_.quake_at_s);
    }
    if (scenario_.slam_at_s) {
        const double t = seconds() - *scenario_.slam_at_s;
        if (t >= 0.0 && t < 0.2) {
            a[0] += 200.0 * std::sin(kTwoPi * 25.0 * t);
        }
    }
    last_ = a;
    return a;
}

bool SimulatedLis3dh::write(std::uint8_t reg, std::uint8_t value) {
    if (in_window(scenario_.bus_fault_at_s)) {
        return false;
    }
    if (reg == lis3dh::kCtrlReg1) {
        ctrl1_ = value;
    } else if (reg == lis3dh::kCtrlReg4) {
        ctrl4_ = value;
    }
    return true;
}

bool SimulatedLis3dh::read(std::uint8_t reg, std::uint8_t* out, std::size_t len) {
    if (in_window(scenario_.bus_fault_at_s)) {
        return false;
    }
    if (reg == lis3dh::kWhoAmI && len == 1) {
        out[0] = lis3dh::kWhoAmIValue;
        return true;
    }
    if (reg != (lis3dh::kOutXL | lis3dh::kAutoIncrement) || len != 6) {
        return false;
    }
    // Like the real part, the chip outputs nothing useful until it has been configured.
    if (ctrl1_ != lis3dh::kCtrl1_100HzAllAxes || ctrl4_ != lis3dh::kCtrl4_BduHighRes2g) {
        std::fill_n(out, len, std::uint8_t{0});
        return true;
    }
    const auto accel = acceleration_mg();
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const double clamped = std::clamp(accel[axis], -2000.0, 1999.0);  // The chip saturates at +/-2 g.
        const auto raw = static_cast<std::int16_t>(std::lround(clamped) * 16);
        out[axis * 2] = static_cast<std::uint8_t>(raw & 0xFF);
        out[axis * 2 + 1] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
    }
    return true;
}

}  // namespace seismo
