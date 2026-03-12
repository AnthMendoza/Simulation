#include "../include/flight_computer.h"


void avionics::flight_computer::initialize(std::vector<avionics::control_tasks> tasks){

    for (auto& task : tasks){
        controller_scheduler.add_manager(task.interval_s,task.callback);
    }

}

void avionics::flight_computer::run(){

}