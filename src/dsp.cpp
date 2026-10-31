#include "seismo/dsp.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>

namespace seismo {

float DcBlocker::process(float x) {
    if (!primed_) {
        // Start at the first value so gravity doesn't look like a huge step.
        mean_ = x;
        primed_ = true;
    }
    mean_ += alpha_ * (x - mean_);
    return x - mean_;
}

StaLtaTrigger::StaLtaTrigger(Config config)
    : config_(config), sta_buf_(config.sta_samples, 0.0F), lta_buf_(config.lta_samples, 0.0F) {}

float StaLtaTrigger::ratio() const {
    if (!warmed_up()) {
        return 0.0F;
    }
    const double sta = sta_sum_ / static_cast<double>(config_.sta_samples);
    const double lta = std::max(lta_sum_ / static_cast<double>(config_.lta_samples),
                                static_cast<double>(config_.lta_floor));
    return static_cast<float>(sta / lta);
}

StaLtaTrigger::Transition StaLtaTrigger::process(float energy) {
    const float leaving_sta = sta_buf_[sta_pos_];
    const bool sta_was_full = sta_filled_ == config_.sta_samples;
    sta_sum_ += energy - leaving_sta;
    sta_buf_[sta_pos_] = energy;
    sta_pos_ = (sta_pos_ + 1) % config_.sta_samples;
    sta_filled_ = std::min(sta_filled_ + 1, config_.sta_samples);

    // A sample joins the background only after leaving the short window, and not during an event.
    // Otherwise the first samples of a sharp knock would inflate the baseline for the next 30 s
    // and hide the weak start of a later earthquake.
    if (sta_was_full && !active_) {
        lta_sum_ += leaving_sta - lta_buf_[lta_pos_];
        lta_buf_[lta_pos_] = leaving_sta;
        lta_pos_ = (lta_pos_ + 1) % config_.lta_samples;
        lta_filled_ = std::min(lta_filled_ + 1, config_.lta_samples);
    }

    if (!warmed_up()) {
        return Transition::None;
    }
    const float current = ratio();
    if (!active_ && current >= config_.on_ratio) {
        active_ = true;
        return Transition::Started;
    }
    if (active_ && current <= config_.off_ratio) {
        active_ = false;
        return Transition::Ended;
    }
    return Transition::None;
}

namespace {

void fft(std::vector<std::complex<float>>& a) {
    const std::size_t n = a.size();
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        std::size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap(a[i], a[j]);
        }
    }
    for (std::size_t len = 2; len <= n; len <<= 1) {
        const float angle = -2.0F * std::numbers::pi_v<float> / static_cast<float>(len);
        const std::complex<float> step(std::cos(angle), std::sin(angle));
        for (std::size_t i = 0; i < n; i += len) {
            std::complex<float> w(1.0F, 0.0F);
            for (std::size_t k = 0; k < len / 2; ++k) {
                const auto even = a[i + k];
                const auto odd = a[i + k + len / 2] * w;
                a[i + k] = even + odd;
                a[i + k + len / 2] = even - odd;
                w *= step;
            }
        }
    }
}

}  // namespace

std::optional<float> dominant_frequency_hz(const std::vector<float>& signal, float sample_rate_hz) {
    if (signal.size() < 16) {
        return std::nullopt;
    }
    std::size_t n = 1;
    while (n * 2 <= signal.size()) {
        n *= 2;
    }

    std::vector<std::complex<float>> bins(n);
    for (std::size_t i = 0; i < n; ++i) {
        const float hann = 0.5F - 0.5F * std::cos(2.0F * std::numbers::pi_v<float> * static_cast<float>(i) /
                                                  static_cast<float>(n - 1));
        bins[i] = signal[i] * hann;
    }
    fft(bins);

    std::size_t best = 1;  // Skip bin 0, which is the DC offset.
    for (std::size_t i = 1; i < n / 2; ++i) {
        if (std::norm(bins[i]) > std::norm(bins[best])) {
            best = i;
        }
    }
    return static_cast<float>(best) * sample_rate_hz / static_cast<float>(n);
}

}  // namespace seismo
