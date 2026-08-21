// spsc_ring.h
#pragma once
#include <atomic>
#include <cstddef>

// Single-Producer Single-Consumer lock-free ring buffer
// Capacity ต้องเป็นเลขยกกำลังสอง (power of two)
template <typename T, size_t Capacity>
class SpscRing {
public:
    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next = (head + 1) & (Capacity - 1);
        if (next == tail_.load(std::memory_order_acquire)) {
            return false; // buffer เต็ม
        }
        buf_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false; // buffer ว่าง
        }
        item = buf_[tail];
        tail_.store((tail + 1) & (Capacity - 1), std::memory_order_release);
        return true;
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

private:
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be power of two");
    T buf_[Capacity];
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
};