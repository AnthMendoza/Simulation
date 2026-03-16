#include "../include/state_estimation_bypass.h"
#include "../../include/avionics_states.h"
#include <iostream>

std::vector<avionics::scheduler_tasks> avionics::estimation::estimation_bypass::get_routine(){
    std::vector<avionics::scheduler_tasks> tasks{
        {"est",  0.03f, [this](float) { calculate_state(); }}
    };

    return tasks;
}


void avionics::estimation::estimation_bypass::calculate_state(){
    std::cout<< "\nhere\n";
}