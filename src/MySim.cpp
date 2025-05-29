#include "../include/MySim.h"


#include <cstring>

#include "../include/control.h"
#include <filesystem>
#include <stdexcept>
#include <memory>

namespace SimCore{
  
void unrealRocket::iterator(float totalTime){
    while(unrealVehicle->getTime() < totalTime &&  unrealVehicle->getPositionVector()[2]> 0 ){
        unrealVehicle->drag();
        unrealVehicle->lift();
        //unrealVehicle->reentryBurn();
        //unrealVehicle->landingBurn();
        //data->logRocketPosition(rocket); //future simulate in unreal and display logs. not logging for now
        unrealVehicle->updateState();
        unrealVehicle->operator++();
    }
}


unrealRocket::unrealRocket(const std::string tomlData){
    unrealVehicle =  std::make_unique<Rocket> (tomlData);
    unrealDataRocket packet;
    unrealVehicle->init();
    totalTime = 0;
}

unrealDataRocket* unrealRocket::simFrameRequest(float deltaTime){
    totalTime += deltaTime;
    iterator(totalTime);    
    setPacket();
    return &packet;
}

unrealRocket::~unrealRocket() = default;

void unrealRocket::setPacket(){
    std::array<float,3> pos = unrealVehicle->getPositionVector();
    packet.position[0] = pos[0];
    packet.position[1] = pos[1];
    packet.position[2] = pos[2];
    std::array<float,3> velo = unrealVehicle->getVelocityVector();
    packet.velocity[0] = velo[0];
    packet.velocity[1] = velo[1];
    packet.velocity[2] = velo[2];
    std::array<float,3> state = unrealVehicle->getState();
    packet.rotation[0] = state[0];
    packet.rotation[1] = state[1];
    packet.rotation[2] = state[2];
    packet.timeStamp = unrealVehicle->getTime();
}

}