#ifndef THREAD_MANGER_H
#define THREAD_MANGER_H
#include <thread>
#include <chrono>
#include <atomic>

class thread_manager {
private:
    void thread_loop() {
        auto next_time = std::chrono::steady_clock::now();
        thread_startup_proccess();
        while (running.load()) {
            thread_proccess();
            
            next_time += interval;
            std::this_thread::sleep_until(next_time);
        }
    }

protected:
    std::atomic<bool> running{false};
    std::thread worker_thread;
    std::chrono::milliseconds interval{100};
    
    virtual void thread_proccess() = 0;
    virtual void thread_startup_proccess() = 0;
    
public:
    thread_manager(int ms = 100): interval(std::chrono::milliseconds(ms)){

    }

    void start() {
        if (!running.load()) {
            running.store(true);
            worker_thread = std::thread(&thread_manager::thread_loop, this);
        }
    }
    
    void stop() {
        running.store(false);
        if (worker_thread.joinable()) {
            worker_thread.join();
        }
    }
    
    virtual ~thread_manager() {
        stop();
    }

    void change_rate(int ms) {
        interval = std::chrono::milliseconds(ms);
    }
};

#endif