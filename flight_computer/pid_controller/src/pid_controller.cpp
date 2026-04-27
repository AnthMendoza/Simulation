#include "../include/pid_controller.h"
#include <drone_control.h>
#include <yaml-cpp/yaml.h>
#include "../include/pid_config_parse.hpp"
#include <base/conversions.h>

avionics::pid::controller_pid::controller_pid(
    flight_health& health_buff,
    state_dbuf& state_buff,
    telem_ring& tel_buff,
    nav_ring& nav_buff,
    const airframe_config& air_config,
    const std::string path
): controller_base(health_buff,state_buff , tel_buff, nav_buff, air_config) {
    
    config_ = pid::load_pid_config(path);
    
    allocate_thrust = std::make_shared<controlAllocator>(
        configuration.motorPositions(),
        configuration.thrustDirections(),
        configuration.rotationDirections()
    );

    auto pid_helper = [](const pid_con& con){
        return PIDController(con.kp, con.ki, con.kd, -con.output_limit, con.output_limit, con.i_limit);
    };

    velocity_pid[0] = pid_helper(config_.velocity.x);
    velocity_pid[1] = pid_helper(config_.velocity.y);
    velocity_pid[2] = pid_helper(config_.velocity.z);


}


void avionics::pid::controller_pid::initialize_implementation(){

}


std::vector<avionics::scheduler_tasks> avionics::pid::controller_pid::get_routine(){
    auto rates = config_.controller.update_rates_hz;

    std::vector<avionics::scheduler_tasks> tasks{
        {"position" ,           utility::hz_sec(rates.position), [this] (float time) {controller_pid::position(time);}},
        {"velocity" ,           utility::hz_sec(rates.velocity), [this] (float time) {controller_pid::velocity(time);}},
        {"acceleration" ,       utility::hz_sec(rates.acceleration), [this] (float time) {controller_pid::acceleration(time);}},
        {"angle_of_attack" ,    utility::hz_sec(rates.attitude), [this] (float time) {controller_pid::angle_of_attack(time);}},
        {"telemetry" ,          utility::hz_sec(rates.telemetry), [this] (float time) {controller_pid::set_telemetry(time);}}
    };

    return tasks;

}

void avionics::pid::controller_pid::parse_gains(const std::string path){
    YAML::Node gain_con = YAML::LoadFile(path);
    
}


void avionics::pid::controller_pid::position(float time){

    if(!initialized_position) [[unlikely]] {
        last_call_time.position = time;
        initialized_position = true;
        return;
    }

    float dt = time - last_call_time.position;

    desired nav = get_navigation();

    for(int i = 0 ; i < 3 ; i++){
        subroutine.delta_position[i] =  nav.position[i] - vehicle_state.position[i];
        
        subroutine.req_velocity[i] =  config_.position.x.kp * subroutine.delta_position[i];
    }

    last_call_time.position = time;

}


void avionics::pid::controller_pid::velocity(float time){

    if(!initialized_velocity) [[unlikely]] {
        last_call_time.velocity = time;
        initialized_velocity = true;
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
    if(!initialized_acceleration) [[unlikely]]{
        float dt = time - last_call_time.acceleration;
        initialized_acceleration = true;
    }

    for(int i = 0 ; i < 3 ; i++){
        subroutine.delta_acceleration[i] =  subroutine.req_acceleration[i] - vehicle_state.velocity[i];

    }

    

    last_call_time.acceleration = time;
}


void avionics::pid::controller_pid::angle_of_attack(float time){
    if(!initialized_aot) [[unlikely]]{
        float dt = time - last_call_time.angle_of_attack;
        initialized_aot = true;
    }


    last_call_time.angle_of_attack = time;
}


void avionics::pid::controller_pid::set_telemetry(float time){

}

void avionics::pid::controller_pid::get_hardware(){
    avionics::estimated_state state = state_buffer.read();
}


avionics::controller_feedback avionics::pid::controller_pid::allocate(){
    
}
