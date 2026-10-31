// Binary UDP packet format for waveform data, and loss tracking on the receiving side.
//
// Layout (big-endian):
//   magic u32 | version u8 | reserved u8 | count u16 | seq u32 | t_us u64 | crc32 u32
//   then `count` samples of x, y, z as i16 milli-g.
// All samples in a packet are 10 ms apart, starting at t_us. The CRC covers everything else.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "seismo/lis3dh.hpp"

namespace seismo::protocol {

inline constexpr std::uint32_t kMagic = 0x53454953;  // "SEIS"
inline constexpr std::uint8_t kVersion = 1;
inline constexpr std::size_t kHeaderSize = 24;
inline constexpr std::size_t kSampleSize = 6;
inline constexpr std::size_t kMaxSamples = 200;  // Stays well under a typical 1500-byte MTU.

struct Packet {
    std::uint32_t seq = 0;
    std::uint64_t t_us = 0;
    std::vector<Sample> samples;
};

std::uint32_t crc32(std::span<const std::uint8_t> data);

std::vector<std::uint8_t> encode(std::uint32_t seq, std::span<const Sample> samples);

// Returns nullopt for anything malformed: wrong magic or version, bad length, or bad CRC.
std::optional<Packet> decode(std::span<const std::uint8_t> bytes);

// UDP can lose, duplicate, or reorder packets. This counts each using sequence numbers.
class GapTracker {
public:
    void observe(std::uint32_t seq);
    std::uint64_t received() const { return received_; }
    std::uint64_t lost() const { return missing_; }
    std::uint64_t duplicates() const { return duplicates_; }
    std::uint64_t reordered() const { return reordered_; }

private:
    bool started_ = false;
    std::uint32_t next_ = 0;
    std::vector<std::uint32_t> gaps_;  // Sequence numbers skipped that might still arrive late.
    std::uint64_t received_ = 0;
    std::uint64_t missing_ = 0;
    std::uint64_t duplicates_ = 0;
    std::uint64_t reordered_ = 0;
};

}  // namespace seismo::protocol
