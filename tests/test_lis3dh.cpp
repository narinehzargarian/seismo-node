#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <vector>

#include "seismo/lis3dh.hpp"
#include "seismo/sim_sensor.hpp"

using namespace seismo;

namespace {

// Records exactly what the driver asks of the bus, to check it against the datasheet.
class FakeBus final : public RegisterBus {
public:
    bool read(std::uint8_t reg, std::uint8_t* out, std::size_t len) override {
        if (fail) {
            return false;
        }
        last_read_reg = reg;
        for (std::size_t i = 0; i < len; ++i) {
            out[i] = i < next_read.size() ? next_read[i] : 0;
        }
        return true;
    }
    bool write(std::uint8_t reg, std::uint8_t value) override {
        if (fail) {
            return false;
        }
        writes[reg] = value;
        return true;
    }
    bool fail = false;
    std::uint8_t last_read_reg = 0;
    std::vector<std::uint8_t> next_read;
    std::map<std::uint8_t, std::uint8_t> writes;
};

}  // namespace

TEST(Lis3dh, InitConfiguresChipPerDatasheet) {
    FakeBus bus;
    bus.next_read = {lis3dh::kWhoAmIValue};
    Lis3dh driver(bus);
    ASSERT_TRUE(driver.init());
    EXPECT_EQ(bus.writes[0x20], 0x57);  // CTRL_REG1: 100 Hz, X/Y/Z enabled.
    EXPECT_EQ(bus.writes[0x23], 0x88);  // CTRL_REG4: block data update, high resolution, +/-2 g.
}

TEST(Lis3dh, InitRejectsWrongChip) {
    FakeBus bus;
    bus.next_read = {0x11};
    Lis3dh driver(bus);
    EXPECT_FALSE(driver.init());
    EXPECT_TRUE(bus.writes.empty());
}

TEST(Lis3dh, ReadsAndScalesSignedSamples) {
    FakeBus bus;
    // +1000 mg = 16000 (0x3E80), -250 mg = -4000 (0xF060), 0. Little-endian, left-justified.
    bus.next_read = {0x80, 0x3E, 0x60, 0xF0, 0x00, 0x00};
    Lis3dh driver(bus);
    const auto sample = driver.read_sample(1234);
    ASSERT_TRUE(sample.has_value());
    EXPECT_EQ(bus.last_read_reg, 0x28 | 0x80);  // Burst read with auto-increment.
    EXPECT_EQ(sample->t_us, 1234U);
    EXPECT_EQ(sample->x_mg, 1000);
    EXPECT_EQ(sample->y_mg, -250);
    EXPECT_EQ(sample->z_mg, 0);
}

TEST(Lis3dh, BusErrorReturnsNothing) {
    FakeBus bus;
    bus.fail = true;
    Lis3dh driver(bus);
    EXPECT_FALSE(driver.read_sample(0).has_value());
}

TEST(SimulatedChip, ReportsGravityAfterDriverConfiguresIt) {
    SimulatedLis3dh chip(SimScenario{});
    Lis3dh driver(chip);
    ASSERT_TRUE(driver.init());
    const auto sample = driver.read_sample(0);
    ASSERT_TRUE(sample.has_value());
    EXPECT_NEAR(sample->z_mg, 1000, 15);
    EXPECT_LT(std::abs(sample->x_mg), 15);
}

TEST(SimulatedChip, OutputsZerosUntilConfigured) {
    SimulatedLis3dh chip(SimScenario{});
    Lis3dh driver(chip);
    const auto sample = driver.read_sample(0);
    ASSERT_TRUE(sample.has_value());
    EXPECT_EQ(sample->z_mg, 0);
}
