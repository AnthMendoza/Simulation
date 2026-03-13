#include "../../simulation/include/sim/toml.h"
#include "../../simulation/include/utility/utility.h"
#include "../include/thread_manager.h"
#include "scheduler.h"
#include "control_tasks.h"
#include <string>
#include <iostream>
#include <cstring>


#pragma once
namespace avionics{

class flight_computer : public thread_manager{
private:
    scheduler controller_scheduler;
    
    
public:
    void initialize(std::vector<control_task> tasks);

    flight_computer(float interval_ms = 1);

    void thread_proccess() override;

    void thread_startup_proccess() override;
    

};

}