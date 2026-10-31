#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "seismo/protocol.hpp"

using namespace seismo;
using namespace seismo::protocol;

namespace {
std::vector<Sample> sample_batch() {
    return {{5'000'000, 12, -34, 1000}, {5'010'000, -32768, 32767, 0}};
}
}  // namespace

TEST(Protocol, Crc32MatchesStandardCheckValue) {
    const std::string text = "123456789";
    const std::vector<std::uint8_t> bytes(text.begin(), text.end());
    EXPECT_EQ(crc32(bytes), 0xCBF43926U);
}

TEST(Protocol, RoundTripPreservesEverything) {
    const auto samples = sample_batch();
    const auto decoded = decode(encode(77, samples));
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->seq, 77U);
    EXPECT_EQ(decoded->t_us, 5'000'000U);
    ASSERT_EQ(decoded->samples.size(), 2U);
    EXPECT_EQ(decoded->samples[0].x_mg, 12);
    EXPECT_EQ(decoded->samples[0].y_mg, -34);
    EXPECT_EQ(decoded->samples[1].x_mg, -32768);
    EXPECT_EQ(decoded->samples[1].y_mg, 32767);
    EXPECT_EQ(decoded->samples[1].t_us, 5'010'000U);
}

TEST(Protocol, EncodesInNetworkByteOrder) {
    const auto bytes = encode(1, sample_batch());
    EXPECT_EQ(bytes[0], 'S');
    EXPECT_EQ(bytes[1], 'E');
    EXPECT_EQ(bytes[2], 'I');
    EXPECT_EQ(bytes[3], 'S');
}

TEST(Protocol, RejectsAnySingleFlippedBit) {
    const auto good = encode(5, sample_batch());
    for (std::size_t byte = 0; byte < good.size(); ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            auto bad = good;
            bad[byte] = static_cast<std::uint8_t>(bad[byte] ^ (1U << bit));
            EXPECT_FALSE(decode(bad).has_value()) << "byte " << byte << " bit " << bit;
        }
    }
}

TEST(Protocol, RejectsTruncatedAndOversizedPackets) {
    const auto good = encode(5, sample_batch());
    for (std::size_t len = 0; len < good.size(); ++len) {
        EXPECT_FALSE(decode({good.data(), len}).has_value());
    }
    auto longer = good;
    longer.push_back(0);
    EXPECT_FALSE(decode(longer).has_value());
}

TEST(GapTracker, CountsLostPackets) {
    GapTracker tracker;
    tracker.observe(10);
    tracker.observe(11);
    tracker.observe(14);
    EXPECT_EQ(tracker.received(), 3U);
    EXPECT_EQ(tracker.lost(), 2U);
}

TEST(GapTracker, LatePacketIsNotCountedAsLost) {
    GapTracker tracker;
    tracker.observe(1);
    tracker.observe(3);
    EXPECT_EQ(tracker.lost(), 1U);
    tracker.observe(2);
    EXPECT_EQ(tracker.lost(), 0U);
    EXPECT_EQ(tracker.reordered(), 1U);
}

TEST(GapTracker, CountsDuplicates) {
    GapTracker tracker;
    tracker.observe(1);
    tracker.observe(2);
    tracker.observe(2);
    EXPECT_EQ(tracker.duplicates(), 1U);
    EXPECT_EQ(tracker.received(), 2U);
}

TEST(GapTracker, HandlesSequenceWrapAround) {
    GapTracker tracker;
    tracker.observe(0xFFFFFFFFU);
    tracker.observe(0);
    tracker.observe(1);
    EXPECT_EQ(tracker.lost(), 0U);
    EXPECT_EQ(tracker.duplicates(), 0U);
}
