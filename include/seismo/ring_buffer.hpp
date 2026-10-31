// Lock-free single-producer, single-consumer ring buffer.
//
// The acquisition thread must never block on the processing thread, or it would miss
// sensor deadlines. This buffer needs no locks and never allocates after construction.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace seismo {

template <typename T, std::size_t Capacity>
class RingBuffer {
    static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");

public:
    // Producer thread only. Returns false when full, so the caller can count the overrun.
    bool push(const T& value) {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        if (head - tail_.load(std::memory_order_acquire) == Capacity) {
            return false;
        }
        slots_[head & (Capacity - 1)] = value;
        // Release pairs with the consumer's acquire: it publishes the slot written above.
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    // Consumer thread only.
    std::optional<T> pop() {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return std::nullopt;
        }
        T value = slots_[tail & (Capacity - 1)];
        tail_.store(tail + 1, std::memory_order_release);
        return value;
    }

    std::size_t size() const {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }

private:
    std::array<T, Capacity> slots_{};
    // Separate cache lines so the two threads don't slow each other down (false sharing).
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
};

}  // namespace seismo
