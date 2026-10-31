#include "seismo/protocol.hpp"

#include <algorithm>
#include <array>

namespace seismo::protocol {

namespace {

constexpr std::size_t kCrcOffset = 20;
constexpr std::size_t kMaxTrackedGaps = 1024;  // Bounded, so a long outage can't grow memory.

template <typename T>
void put_be(std::vector<std::uint8_t>& out, T value) {
    for (int shift = (sizeof(T) - 1) * 8; shift >= 0; shift -= 8) {
        out.push_back(static_cast<std::uint8_t>(static_cast<std::uint64_t>(value) >> shift));
    }
}

template <typename T>
T get_be(const std::uint8_t* in) {
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        value = (value << 8) | in[i];
    }
    return static_cast<T>(value);
}

constexpr std::array<std::uint32_t, 256> make_crc_table() {
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t i = 0; i < 256; ++i) {
        std::uint32_t c = i;
        for (int bit = 0; bit < 8; ++bit) {
            c = (c & 1U) != 0U ? 0xEDB88320U ^ (c >> 1) : c >> 1;
        }
        table[i] = c;
    }
    return table;
}

constexpr auto kCrcTable = make_crc_table();

std::uint32_t packet_crc(std::span<const std::uint8_t> bytes) {
    std::uint32_t c = 0xFFFFFFFFU;
    const auto update = [&c](std::uint8_t byte) { c = kCrcTable[(c ^ byte) & 0xFFU] ^ (c >> 8); };
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        // The CRC field itself counts as zeros.
        update(i >= kCrcOffset && i < kCrcOffset + 4 ? std::uint8_t{0} : bytes[i]);
    }
    return c ^ 0xFFFFFFFFU;
}

}  // namespace

std::uint32_t crc32(std::span<const std::uint8_t> data) {
    std::uint32_t c = 0xFFFFFFFFU;
    for (const std::uint8_t byte : data) {
        c = kCrcTable[(c ^ byte) & 0xFFU] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFU;
}

std::vector<std::uint8_t> encode(std::uint32_t seq, std::span<const Sample> samples) {
    const std::size_t count = std::min(samples.size(), kMaxSamples);
    std::vector<std::uint8_t> out;
    out.reserve(kHeaderSize + count * kSampleSize);

    put_be<std::uint32_t>(out, kMagic);
    out.push_back(kVersion);
    out.push_back(0);
    put_be<std::uint16_t>(out, static_cast<std::uint16_t>(count));
    put_be<std::uint32_t>(out, seq);
    put_be<std::uint64_t>(out, count > 0 ? samples[0].t_us : 0);
    put_be<std::uint32_t>(out, 0);  // CRC placeholder, filled in below.

    for (std::size_t i = 0; i < count; ++i) {
        put_be<std::uint16_t>(out, static_cast<std::uint16_t>(samples[i].x_mg));
        put_be<std::uint16_t>(out, static_cast<std::uint16_t>(samples[i].y_mg));
        put_be<std::uint16_t>(out, static_cast<std::uint16_t>(samples[i].z_mg));
    }

    const std::uint32_t crc = packet_crc(out);
    for (std::size_t i = 0; i < 4; ++i) {
        out[kCrcOffset + i] = static_cast<std::uint8_t>(crc >> (24 - 8 * i));
    }
    return out;
}

std::optional<Packet> decode(std::span<const std::uint8_t> bytes) {
    if (bytes.size() < kHeaderSize || get_be<std::uint32_t>(bytes.data()) != kMagic || bytes[4] != kVersion) {
        return std::nullopt;
    }
    const auto count = get_be<std::uint16_t>(bytes.data() + 6);
    if (count > kMaxSamples || bytes.size() != kHeaderSize + count * kSampleSize) {
        return std::nullopt;
    }
    if (get_be<std::uint32_t>(bytes.data() + kCrcOffset) != packet_crc(bytes)) {
        return std::nullopt;
    }

    Packet packet;
    packet.seq = get_be<std::uint32_t>(bytes.data() + 8);
    packet.t_us = get_be<std::uint64_t>(bytes.data() + 12);
    packet.samples.reserve(count);
    const std::uint8_t* p = bytes.data() + kHeaderSize;
    for (std::size_t i = 0; i < count; ++i, p += kSampleSize) {
        packet.samples.push_back(Sample{packet.t_us + i * 10'000,
                                        get_be<std::int16_t>(p),
                                        get_be<std::int16_t>(p + 2),
                                        get_be<std::int16_t>(p + 4)});
    }
    return packet;
}

void GapTracker::observe(std::uint32_t seq) {
    if (!started_) {
        started_ = true;
        next_ = seq + 1;
        ++received_;
        return;
    }

    // Signed difference handles sequence numbers wrapping around at 2^32.
    const auto delta = static_cast<std::int32_t>(seq - next_);
    if (delta >= 0) {
        for (std::int32_t i = 0; i < delta; ++i) {
            ++missing_;
            if (gaps_.size() < kMaxTrackedGaps) {
                gaps_.push_back(next_ + static_cast<std::uint32_t>(i));
            }
        }
        next_ = seq + 1;
        ++received_;
        return;
    }

    const auto gap = std::find(gaps_.begin(), gaps_.end(), seq);
    if (gap != gaps_.end()) {
        // A packet we counted as lost showed up late.
        gaps_.erase(gap);
        --missing_;
        ++reordered_;
        ++received_;
    } else {
        ++duplicates_;
    }
}

}  // namespace seismo::protocol
