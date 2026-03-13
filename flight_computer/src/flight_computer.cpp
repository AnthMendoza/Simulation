#include "../include/flight_computer.h"

avionics::flight_computer::flight_computer(float interval_ms): thread_manager(interval_ms){

}

void avionics::flight_computer::initialize(std::vector<avionics::control_task> tasks){

    for (auto& task : tasks){
        controller_scheduler.add_manager(task.interval_s,task.callback);
    }

}

void avionics::flight_computer::thread_proccess(){
    controller_scheduler(start_to_recent_call_time());
}

void avionics::flight_computer::thread_startup_proccess(){
    
}