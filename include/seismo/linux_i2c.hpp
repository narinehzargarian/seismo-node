// Real I2C access through the Linux i2c-dev interface (for example /dev/i2c-1 on a Raspberry Pi).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "seismo/lis3dh.hpp"

namespace seismo {

class LinuxI2cBus final : public RegisterBus {
public:
    LinuxI2cBus(const std::string& device, std::uint8_t address);
    ~LinuxI2cBus() override;
    LinuxI2cBus(const LinuxI2cBus&) = delete;
    LinuxI2cBus& operator=(const LinuxI2cBus&) = delete;

    bool read(std::uint8_t reg, std::uint8_t* out, std::size_t len) override;
    bool write(std::uint8_t reg, std::uint8_t value) override;

private:
    // Unused in the non-Linux stub build.
    [[maybe_unused]] std::uint8_t address_;
    [[maybe_unused]] int fd_ = -1;
};

}  // namespace seismo
