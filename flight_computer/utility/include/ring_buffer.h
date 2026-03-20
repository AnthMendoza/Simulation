#pragma once
#include <cstddef>
#include <atomic>
#include <optional>

namespace utility::buffer {

template<typename T, std::size_t N>
// Ring buffer: overwrites unread data when the buffer is full.
class ring_buffer {
private:
    T buffer[N];
    std::atomic<std::size_t> read_index{0};
    std::atomic<std::size_t> write_index{0};

    std::size_t next_index(std::size_t current_index) const {
        return (current_index + 1) % N;
    }

public:
    void push(const T& data) {
        std::size_t write = write_index.load(std::memory_order_relaxed);
        buffer[write] = data;
        write_index.store(next_index(write), std::memory_order_release);
    }

    std::optional<T> pop() {
        std::size_t read  = read_index.load(std::memory_order_relaxed);
        std::size_t write = write_index.load(std::memory_order_acquire);

        if (read == write) {
            return std::nullopt; 
        }

        T value = buffer[read];
        read_index.store(next_index(read), std::memory_order_release);
        return value;
    }
};

} 