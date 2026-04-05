#include "../include/estimation.h"
#include "../../include/avionics_states.h"



std::vector<avionics::scheduler_tasks> avionics::estimation::estimation::get_routine(){
    std::vector<avionics::scheduler_tasks> tasks{
        {"update estimation",  0.03f, [this](float) {calculate_state();}},
        {"get sensor data",  0.03f, [this](float) { get_sensor_data();}}
    };

    return tasks;
}


void avionics::estimation::estimation::calculate_state(){
   
    


}


void avionics::estimation::estimation::get_sensor_data(){

    
    
}