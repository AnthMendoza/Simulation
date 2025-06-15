#include "../include/MySim.h"
#include <cstring>
#include "../include/control.h"
#include "../include/propeller.h"
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


unrealRocket::unrealRocket(std::string tomlData){
    unrealVehicle =  std::make_unique<Rocket> (tomlData);
    unrealDataRocket packet;
    unrealVehicle->init(tomlData);
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

unrealDataDrone* unrealDrone::simFrameRequest(float deltaTime){
    totalTime += deltaTime;
    iterator(totalTime);
    setPacket();
    return &packet;
}

unrealDrone::unrealDrone(std::string motorConfig , std::string batteryConfig , std::string droneConfig,std::string propellerConfig){
    drone = std::make_unique<droneBody>();
    drone->init(motorConfig,batteryConfig,droneConfig);
    propeller prop;
    initPropeller(prop,propellerConfig);
    drone->setSquare(0.3f,0.3f,prop);
}

void unrealDrone::setTargetPosition(float x , float y , float z , float yaw){

    drone->controller->setpidControl(x,y,z);

}

void unrealDrone::iterator(float totalTime,bool display){
    while(drone->getTime() < totalTime &&  drone->getPositionVector()[2]> 0 ){
        drone->updateState();
        if(display) drone->display();
    }
}

void unrealDrone::setPacket(){
    std::array<float,3> pos = drone->getPositionVector();
    packet.position[0] = pos[0];
    packet.position[1] = pos[1];
    packet.position[2] = pos[2];
    std::array<float,3> velo = drone->getVelocityVector();
    packet.velocity[0] = velo[0];
    packet.velocity[1] = velo[1];
    packet.velocity[2] = velo[2];
    std::array<std::array<float,3>,3> state = drone->getPose();
    packet.dirVector[0] = state[0][0];
    packet.dirVector[1] = state[0][1];
    packet.dirVector[2] = state[0][2];

    packet.fwdVector[0] = state[1][0];
    packet.fwdVector[1] = state[1][1];
    packet.fwdVector[2] = state[1][2];

    packet.rightVector[0] = state[2][0]; 
    packet.rightVector[1] = state[2][1];
    packet.rightVector[2] = state[2][2];
    packet.timeStamp = drone->getTime();
}



}