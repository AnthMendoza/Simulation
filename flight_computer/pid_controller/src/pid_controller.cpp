#include "../include/pid_controller.h"


avionics::pid::controller_pid::controller_pid(pid_config& config){

    for(int i = 0; i < 3 ; i++){

        auto p_gain = config.position[i];
        position_pid[i] = PIDController(p_gain.kp,p_gain.ki,p_gain.kd); 
        position_pid[i]->setOutputLimits(p_gain.min,p_gain.max);

    }

    for(int i = 0 ; i < 3 ; i++){

        auto v_gain = config.velocity[i];
        velocity_pid[i] = PIDController(v_gain.kp,v_gain.ki,v_gain.kd);
        velocity_pid[i]->setOutputLimits(v_gain.min,v_gain.max);

    }
    
}

std::vector<avionics::control_task> avionics::pid::controller_pid::get_routine(){

    std::vector<avionics::control_task> tasks{
        {"position" ,           1.0f , [this] () {controller_pid::position();}},
        {"velocity" ,           0.5f , [this] () {controller_pid::velocity();}},
        {"acceleration" ,       0.01f , [this] () {controller_pid::acceleration();}},
        {"angle_of_attack" ,    0.001f , [this] () {controller_pid::angle_of_attack();}}
    };

    return tasks;

}


void avionics::pid::controller_pid::position(){

    for(int i = 0 ; i < 3 ; i++){
        subroutine.delta_position[i] =  vehicle_desired_state.position[i] - vehicle_state.position[i];
    }
    
}


void avionics::pid::controller_pid::velocity(){

    for(int i = 0 ; i < 3 ; i++){
        subroutine.delta_position[i] =  vehicle_desired_state.position[i] - vehicle_state.position[i];
    }

}


void avionics::pid::controller_pid::acceleration(){

}


void avionics::pid::controller_pid::angle_of_attack(){

}
