#include "../include/state_estimation_bypass.h"
#include "../../include/avionics_states.h"


std::vector<avionics::scheduler_tasks> avionics::estimation::estimation_bypass::get_routine(){
    std::vector<avionics::scheduler_tasks> tasks{
        {"update estimation",  0.03f, [this](float) { calculate_state(); }}
    };

    return tasks;
}


void avionics::estimation::estimation_bypass::calculate_state(){
   
}


void avionics::estimation::estimation_bypass::get_actual_state(){

    

}

void avionics::estimation::estimation_bypass::get_sensor_data(){


    
}