#include "seismo/lis3dh.hpp"

#include <array>

namespace seismo {

namespace {

// In high-resolution mode each axis is a 12-bit value, left-justified in 16 bits, little-endian.
// At +/-2 g the datasheet sensitivity is 1 mg per digit.
std::int16_t to_milli_g(std::uint8_t low, std::uint8_t high) {
    const auto raw = static_cast<std::int16_t>(static_cast<std::uint16_t>(high) << 8 | low);
    return static_cast<std::int16_t>(raw / 16);
}

}  // namespace

bool Lis3dh::init() {
    std::uint8_t id = 0;
    if (!bus_.read(lis3dh::kWhoAmI, &id, 1) || id != lis3dh::kWhoAmIValue) {
        return false;
    }
    return bus_.write(lis3dh::kCtrlReg1, lis3dh::kCtrl1_100HzAllAxes) &&
           bus_.write(lis3dh::kCtrlReg4, lis3dh::kCtrl4_BduHighRes2g);
}

std::optional<Sample> Lis3dh::read_sample(std::uint64_t t_us) {
    std::array<std::uint8_t, 6> raw{};
    if (!bus_.read(lis3dh::kOutXL | lis3dh::kAutoIncrement, raw.data(), raw.size())) {
        return std::nullopt;
    }
    return Sample{t_us, to_milli_g(raw[0], raw[1]), to_milli_g(raw[2], raw[3]), to_milli_g(raw[4], raw[5])};
}

}  // namespace seismo
