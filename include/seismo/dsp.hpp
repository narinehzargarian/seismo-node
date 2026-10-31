// Signal processing: gravity removal, STA/LTA event detection, and dominant frequency.
#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace seismo {

// One-pole high-pass filter. Removes gravity and slow drift, keeps the shaking.
class DcBlocker {
public:
    explicit DcBlocker(float alpha = 0.01F) : alpha_(alpha) {}
    float process(float x);
    void reset() { primed_ = false; }

private:
    float alpha_;
    float mean_ = 0.0F;
    bool primed_ = false;
};

// STA/LTA trigger, the classic seismology detector: compares short-term energy to the
// long-term background, so it adapts to how noisy the site is.
class StaLtaTrigger {
public:
    struct Config {
        std::size_t sta_samples = 100;   // 1 s at 100 Hz
        std::size_t lta_samples = 3000;  // 30 s at 100 Hz
        float on_ratio = 4.0F;
        float off_ratio = 1.5F;
        float lta_floor = 1.0F;  // Stops the ratio exploding on a very quiet sensor.
    };

    enum class Transition { None, Started, Ended };

    explicit StaLtaTrigger(Config config);

    Transition process(float energy);
    bool active() const { return active_; }
    bool warmed_up() const { return lta_filled_ == config_.lta_samples; }
    float ratio() const;

private:
    Config config_;
    std::vector<float> sta_buf_;
    std::vector<float> lta_buf_;
    std::size_t sta_pos_ = 0;
    std::size_t sta_filled_ = 0;
    std::size_t lta_pos_ = 0;
    std::size_t lta_filled_ = 0;
    double sta_sum_ = 0.0;
    double lta_sum_ = 0.0;
    bool active_ = false;
};

// Strongest frequency in Hz, using a Hann window and a radix-2 FFT.
// Returns nullopt if there are fewer than 16 samples.
std::optional<float> dominant_frequency_hz(const std::vector<float>& signal, float sample_rate_hz);

}  // namespace seismo
