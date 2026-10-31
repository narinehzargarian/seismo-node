#include <gtest/gtest.h>

#include <cstdint>
#include <thread>

#include "seismo/ring_buffer.hpp"

using seismo::RingBuffer;

TEST(RingBuffer, EmptyPopReturnsNothing) {
    RingBuffer<int, 4> ring;
    EXPECT_FALSE(ring.pop().has_value());
}

TEST(RingBuffer, RejectsPushWhenFull) {
    RingBuffer<int, 4> ring;
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(ring.push(i));
    }
    EXPECT_FALSE(ring.push(99));
    EXPECT_EQ(ring.size(), 4U);
}

TEST(RingBuffer, KeepsOrderAcrossWrapAround) {
    RingBuffer<int, 4> ring;
    for (int i = 0; i < 100; ++i) {
        ASSERT_TRUE(ring.push(i));
        const auto value = ring.pop();
        ASSERT_TRUE(value.has_value());
        EXPECT_EQ(*value, i);
    }
}

// Run under ThreadSanitizer in CI to catch data races.
TEST(RingBuffer, TwoThreadsTransferEveryValueInOrder) {
    constexpr std::uint64_t kCount = 500'000;
    RingBuffer<std::uint64_t, 1024> ring;

    std::thread producer([&] {
        for (std::uint64_t i = 0; i < kCount; ++i) {
            while (!ring.push(i)) {
                std::this_thread::yield();
            }
        }
    });

    std::uint64_t expected = 0;
    while (expected < kCount) {
        if (const auto value = ring.pop()) {
            ASSERT_EQ(*value, expected);
            ++expected;
        } else {
            std::this_thread::yield();
        }
    }
    producer.join();
}
