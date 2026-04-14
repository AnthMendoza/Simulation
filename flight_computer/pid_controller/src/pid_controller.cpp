#include "../include/pid_controller.h"
#include <drone_control.h>
#include <yaml-cpp/yaml.h>

avionics::pid::controller_pid::controller_pid(
    state_dbuf& state_buff,
    telem_ring& tel_buff,
    nav_ring& nav_buff,
    const airframe_config& air_config,
    const pid_config& config
): controller_base(state_buff , tel_buff, nav_buff, air_config) {
    
    config_ = config;

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

    std::vector<avionics::scheduler_tasks> tasks{
        {"position" ,           1.0f , [this] (float time) {controller_pid::position(time);}},
        {"velocity" ,           0.5f , [this] (float time) {controller_pid::velocity(time);}},
        {"acceleration" ,       0.01f , [this] (float time) {controller_pid::acceleration(time);}},
        {"angle_of_attack" ,    0.001f , [this] (float time) {controller_pid::angle_of_attack(time);}},
        {"telemetry" ,    0.1f , [this] (float time) {controller_pid::set_telemetry(time);}}
    };

    return tasks;

}

void avionics::pid::controller_pid::parse_gains(const std::string path){
    YAML::Node gain_con = YAML::LoadFile(path);
    
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
        
        subroutine.req_velocity[i] =  config_.position.x.kp * subroutine.delta_position[i];
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
