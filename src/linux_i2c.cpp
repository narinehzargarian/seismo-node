#include "seismo/linux_i2c.hpp"

#ifdef __linux__

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <stdexcept>

namespace seismo {

LinuxI2cBus::LinuxI2cBus(const std::string& device, std::uint8_t address) : address_(address) {
    fd_ = open(device.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
        throw std::runtime_error("could not open " + device);
    }
}

LinuxI2cBus::~LinuxI2cBus() {
    if (fd_ >= 0) {
        close(fd_);
    }
}

bool LinuxI2cBus::read(std::uint8_t reg, std::uint8_t* out, std::size_t len) {
    // Write the register address, then read with a repeated start, as one atomic transaction.
    std::array<i2c_msg, 2> messages{};
    messages[0] = {address_, 0, 1, &reg};
    messages[1] = {address_, I2C_M_RD, static_cast<__u16>(len), out};
    i2c_rdwr_ioctl_data transaction{messages.data(), static_cast<__u32>(messages.size())};
    return ioctl(fd_, I2C_RDWR, &transaction) == static_cast<int>(messages.size());
}

bool LinuxI2cBus::write(std::uint8_t reg, std::uint8_t value) {
    std::array<std::uint8_t, 2> payload{reg, value};
    i2c_msg message{address_, 0, static_cast<__u16>(payload.size()), payload.data()};
    i2c_rdwr_ioctl_data transaction{&message, 1};
    return ioctl(fd_, I2C_RDWR, &transaction) == 1;
}

}  // namespace seismo

#else

#include <stdexcept>

namespace seismo {

LinuxI2cBus::LinuxI2cBus(const std::string&, std::uint8_t) : address_(0) {
    throw std::runtime_error("I2C is only available on Linux; use --sim on this platform");
}
LinuxI2cBus::~LinuxI2cBus() = default;
bool LinuxI2cBus::read(std::uint8_t, std::uint8_t*, std::size_t) { return false; }
bool LinuxI2cBus::write(std::uint8_t, std::uint8_t) { return false; }

}  // namespace seismo

#endif
