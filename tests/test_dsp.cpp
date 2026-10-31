#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>
#include <vector>

#include "seismo/dsp.hpp"

using namespace seismo;

TEST(DcBlocker, RemovesConstantOffsetWithoutStartupSpike) {
    DcBlocker filter;
    for (int i = 0; i < 100; ++i) {
        EXPECT_NEAR(filter.process(1000.0F), 0.0F, 1e-3F);
    }
}

TEST(DcBlocker, PassesFastShaking) {
    DcBlocker filter;
    float peak = 0.0F;
    for (int i = 0; i < 1000; ++i) {
        const float shake = 100.0F * std::sin(2.0F * std::numbers::pi_v<float> * 5.0F * static_cast<float>(i) / 100.0F);
        peak = std::max(peak, filter.process(1000.0F + shake));
    }
    EXPECT_GT(peak, 85.0F);
}

namespace {
StaLtaTrigger::Config short_config() {
    StaLtaTrigger::Config config;
    config.sta_samples = 10;
    config.lta_samples = 100;
    return config;
}
}  // namespace

TEST(StaLta, DoesNotTriggerDuringWarmup) {
    StaLtaTrigger trigger(short_config());
    // Warm-up needs both windows full: 10 + 100 samples.
    for (int i = 0; i < 109; ++i) {
        EXPECT_EQ(trigger.process(1000.0F), StaLtaTrigger::Transition::None);
    }
    EXPECT_FALSE(trigger.warmed_up());
    trigger.process(1000.0F);
    EXPECT_TRUE(trigger.warmed_up());
}

// Regression test: a knock's first samples used to enter the baseline before the trigger fired,
// which hid the weak start of an earthquake that came shortly after.
TEST(StaLta, SharpKnockDoesNotDesensitizeTheDetector) {
    StaLtaTrigger trigger(short_config());
    for (int i = 0; i < 110; ++i) {
        trigger.process(10.0F);
    }
    EXPECT_EQ(trigger.process(100000.0F), StaLtaTrigger::Transition::Started);
    while (trigger.active()) {
        trigger.process(10.0F);
    }
    for (int i = 0; i < 10; ++i) {
        trigger.process(10.0F);
    }
    // 5x the true background must still trigger.
    bool started = false;
    for (int i = 0; i < 10 && !started; ++i) {
        started = trigger.process(50.0F) == StaLtaTrigger::Transition::Started;
    }
    EXPECT_TRUE(started);
}

TEST(StaLta, SteadyNoiseNeverTriggers) {
    StaLtaTrigger trigger(short_config());
    std::mt19937 rng(7);
    std::normal_distribution<float> noise(0.0F, 2.0F);
    for (int i = 0; i < 20000; ++i) {
        const float n = noise(rng);
        // Average three axes' worth of energy to reflect how the node feeds this.
        EXPECT_EQ(trigger.process(n * n + 4.0F), StaLtaTrigger::Transition::None);
    }
}

TEST(StaLta, StartsOnShakingAndEndsWhenItStops) {
    StaLtaTrigger trigger(short_config());
    for (int i = 0; i < 100; ++i) {
        trigger.process(10.0F);
    }
    bool started = false;
    for (int i = 0; i < 20 && !started; ++i) {
        started = trigger.process(1000.0F) == StaLtaTrigger::Transition::Started;
    }
    EXPECT_TRUE(started);
    EXPECT_TRUE(trigger.active());

    bool ended = false;
    for (int i = 0; i < 20 && !ended; ++i) {
        ended = trigger.process(10.0F) == StaLtaTrigger::Transition::Ended;
    }
    EXPECT_TRUE(ended);
    EXPECT_FALSE(trigger.active());
}

TEST(StaLta, LongEventDoesNotRaiseItsOwnBaseline) {
    StaLtaTrigger trigger(short_config());
    for (int i = 0; i < 100; ++i) {
        trigger.process(10.0F);
    }
    // Longer than the LTA window. If the baseline adapted, the event would end by itself.
    for (int i = 0; i < 500; ++i) {
        trigger.process(1000.0F);
    }
    EXPECT_TRUE(trigger.active());
}

TEST(DominantFrequency, FindsSineFrequency) {
    for (const float hz : {1.0F, 3.0F, 8.0F, 20.0F}) {
        std::vector<float> signal(1024);
        for (std::size_t i = 0; i < signal.size(); ++i) {
            signal[i] = std::sin(2.0F * std::numbers::pi_v<float> * hz * static_cast<float>(i) / 100.0F);
        }
        const auto measured = dominant_frequency_hz(signal, 100.0F);
        ASSERT_TRUE(measured.has_value());
        EXPECT_NEAR(*measured, hz, 0.15F);
    }
}

TEST(DominantFrequency, NeedsEnoughSamples) {
    EXPECT_FALSE(dominant_frequency_hz(std::vector<float>(8, 1.0F), 100.0F).has_value());
}
