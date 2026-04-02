#ifndef THREAD_MANGER_H
#define THREAD_MANGER_H
#include "scheduler.h"
#include <thread>
#include <chrono>
#include <atomic>



class thread_manager {
private:
    std::function<float(float)>  next_time_callback;
    float total_time_f = 0.0f;
    bool schedule_driven = false;

    //original did not have a re-sync after initalization
    virtual void thread_loop() {
        start_time = std::chrono::steady_clock::now();
        thread_startup_proccess();
        auto next_time = std::chrono::steady_clock::now();

        while (running.load()) {
            
            auto now = std::chrono::steady_clock::now();
            auto total_time = now - start_time;
            total_time_f = std::chrono::duration<float>(total_time).count();
            thread_proccess();
            
            if(schedule_driven == true){
                auto next_by_duration = std::chrono::duration<float>(next_time_callback(total_time_f));
                next_time = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(next_by_duration);
            }else{
                next_time = now + interval;
            }
            std::this_thread::sleep_until(next_time);
        }
    }

protected:
    std::atomic<bool> running{false};
    std::thread worker_thread;
    std::chrono::milliseconds interval{100};

    std::__1::chrono::steady_clock::time_point start_time;
    
    virtual void thread_proccess() = 0;
    virtual void thread_startup_proccess() = 0;

    void set_scheduler(scheduler& schedule){
        schedule_driven = true;
        next_time_callback = schedule.get_next_time_callback();
    }
    
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

    const float start_to_recent_call_time() const{
        return total_time_f;
    }
};

#endif