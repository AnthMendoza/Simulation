#include "../include/pid_controller.h"
#include <drone_control.h>

avionics::pid::controller_pid::controller_pid(
    state_dbuf& state_buff,
    telem_ring& tel_buff,
    nav_ring& nav_buff,
    const airframe_config& air_config,
    const pid_config& config
): controller_base(state_buff , tel_buff, nav_buff, air_config) {
    
    allocate_thrust = std::make_shared<controlAllocator>(
        configuration.motorPositions(),
        configuration.thrustDirections(),
        configuration.rotationDirections()
    );
    
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


void avionics::pid::controller_pid::initialize_implementation(){

}


std::vector<avionics::scheduler_tasks> avionics::pid::controller_pid::get_routine(){

    std::vector<avionics::scheduler_tasks> tasks{
        {"position" ,           1.0f , [this] (float time) {controller_pid::position(time);}},
        {"velocity" ,           0.5f , [this] (float time) {controller_pid::velocity(time);}},
        {"acceleration" ,       0.01f , [this] (float time) {controller_pid::acceleration(time);}},
        {"angle_of_attack" ,    0.001f , [this] (float time) {controller_pid::angle_of_attack(time);}},
        {"telemetry" ,    0.1f , [this] (float time) {controller_pid::set_telemetry(time);}}
    };

    return tasks;

}


void avionics::pid::controller_pid::position(float time){

    if(last_call_time.position == 0){
        last_call_time.position = time;
        return;
    }

    float dt = time - last_call_time.position;

    desired nav = get_navigation();

    for(int i = 0 ; i < 3 ; i++){
        subroutine.delta_position[i] =  nav.position[i] - vehicle_state.position[i];

        position_pid[i]->setTarget(nav.position[i]);
        subroutine.req_velocity[i] = position_pid[i]->update(vehicle_state.position[i],dt);

    }

    last_call_time.position = time;

}


void avionics::pid::controller_pid::velocity(float time){

    if(last_call_time.velocity == 0){
        last_call_time.velocity = time;
        return;
    }

    float dt = time - last_call_time.velocity;

    for(int i = 0 ; i < 3 ; i++){
        subroutine.delta_velocity[i] = subroutine.req_velocity[i] - vehicle_state.velocity[i];

        velocity_pid[i]->setTarget(subroutine.req_velocity[i]);
        subroutine.req_acceleration[i] = velocity_pid[i]->update(vehicle_state.velocity[i],dt);

    }

    last_call_time.velocity = time;
}


void avionics::pid::controller_pid::acceleration(float time){
    float dt = time - last_call_time.acceleration;

    for(int i = 0 ; i < 3 ; i++){
        subroutine.delta_acceleration[i] =  subroutine.req_acceleration[i] - vehicle_state.velocity[i];

    }

    last_call_time.acceleration = time;
}


void avionics::pid::controller_pid::angle_of_attack(float time){
    float dt = time - last_call_time.angle_of_attack;



    last_call_time.angle_of_attack = time;
}


void avionics::pid::controller_pid::set_telemetry(float time){
    subroutine.print(logger);
}

void avionics::pid::controller_pid::get_hardware(){
    avionics::estimated_state state = state_buffer.read();
}


avionics::controller_feedback avionics::pid::controller_pid::allocate(){
}