#ifndef DOUBLE_BUFFER_H
#define DOUBLE_BUFFER_H

#include <atomic>
namespace utility::buffer{

template<typename T>
class double_buffer{
    private:
    T buffers[2];

    std::atomic<int> read_index{0};

    public:

    T& writeBuffer(){
        int current = read_index.load(std::memory_order_relaxed);
        int write_index = current == 1 ? 0:1;
        return buffers[write_index];
    }


    void publish(){
        int current = read_index.load(std::memory_order_relaxed);
        int write_index = current == 1 ? 0:1;
        read_index.store(write_index, std::memory_order_release);
    }

    T read() const{
        int index = read_index.load(std::memory_order_acquire);
        return buffers[index];  
    }
};

}

#endif