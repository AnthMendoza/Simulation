#include "../include/MySim.h"

#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include "../include/control.h"
#include <filesystem>
#include <stdexcept>

namespace SimCore{
  
void unreal::iterator(float deltaTime){
    while(unrealVehicle.getTime() < deltaTime){
        if(unrealVehicle.getPositionVector()[2]> 0 ){
            unrealVehicle.drag();
            unrealVehicle.lift();
            unrealVehicle.reentryBurn();
            unrealVehicle.landingBurn();
            //data->logRocketPosition(rocket); //future simulate in unreal and display logs. not logging for now
            unrealVehicle.updateState();
            unrealVehicle++;
        }else break;
    }
}


unreal::unreal(const char* ConfigPath){
    if (!std::filesystem::exists(ConfigPath)) {
        throw std::runtime_error("Config file not found");
    }
    constants::configFile = ConfigPath;
    Rocket unrealVehicle;
    unrealData packet;
}

unrealData* unreal::simFrameRequest(float deltaTime){
    iterator(deltaTime);    
    setPacket();
    return &packet;
}

unreal::~unreal() = default;

void unreal::setPacket(){
    std::array<float,3> pos = unrealVehicle.getPositionVector();
    packet.position[0] = pos[0];
    packet.position[1] = pos[1];
    packet.position[2] = pos[2];
    std::array<float,3> velo = unrealVehicle.getVelocityVector();
    packet.velocity[0] = velo[0];
    packet.velocity[1] = velo[1];
    packet.velocity[2] = velo[2];
    std::array<float,3> state = unrealVehicle.getState();
    packet.rotation[0] = state[0];
    packet.rotation[1] = state[1];
    packet.rotation[2] = state[2];
}


/*

unreal *unrealVehicle = new unreal;


*/
}
