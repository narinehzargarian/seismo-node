// Driver for the LIS3DH 3-axis accelerometer, written against the datasheet register map.
// It talks through RegisterBus, so the same driver runs on real I2C or a simulated chip.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace seismo {

// One acceleration reading in milli-g, stamped with microseconds since start.
struct Sample {
    std::uint64_t t_us = 0;
    std::int16_t x_mg = 0;
    std::int16_t y_mg = 0;
    std::int16_t z_mg = 0;
};

class RegisterBus {
public:
    virtual ~RegisterBus() = default;
    virtual bool read(std::uint8_t reg, std::uint8_t* out, std::size_t len) = 0;
    virtual bool write(std::uint8_t reg, std::uint8_t value) = 0;
};

namespace lis3dh {
inline constexpr std::uint8_t kWhoAmI = 0x0F;
inline constexpr std::uint8_t kWhoAmIValue = 0x33;
inline constexpr std::uint8_t kCtrlReg1 = 0x20;
inline constexpr std::uint8_t kCtrlReg4 = 0x23;
inline constexpr std::uint8_t kOutXL = 0x28;
inline constexpr std::uint8_t kAutoIncrement = 0x80;  // Set on the register address for burst reads.
inline constexpr std::uint8_t kCtrl1_100HzAllAxes = 0x57;
inline constexpr std::uint8_t kCtrl4_BduHighRes2g = 0x88;
}  // namespace lis3dh

class Lis3dh {
public:
    explicit Lis3dh(RegisterBus& bus) : bus_(bus) {}

    // Verifies the chip identity and configures 100 Hz, +/-2 g, high-resolution mode.
    bool init();

    // Reads one sample in milli-g. Returns nullopt on a bus error.
    std::optional<Sample> read_sample(std::uint64_t t_us);

private:
    RegisterBus& bus_;
};

}  // namespace seismo
