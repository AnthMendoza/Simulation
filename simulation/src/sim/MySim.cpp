#include "../../include/sim/MySim.h"
#include <cstring>
#include "../../include/control/control.h"
#include "../../include/subsystems/propeller.h"
#include "../../include/subsystems/motor.h"
#include "../../include/dynamics/aero.h"
#include "../../include/subsystems/droneDependencyInjector.h"
#include <filesystem>
#include <stdexcept>
#include <memory>

namespace SimCore{
  
void unrealRocket::iterator(float totalTime){
    while(unrealVehicle->getTime() < totalTime &&  unrealVehicle->getPositionVector()[2]> 0 ){
        unrealVehicle->drag(aeroAreaRocket,coefOfDragRocket);
        unrealVehicle->lift(aeroAreaRocket,coefOfLiftRocket);
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
#define POLLING_RATE 0.01
unrealDrone::unrealDrone(std::string motorConfig , std::string batteryConfig , std::string droneConfig,std::string propellerConfig){
    auto controller = std::make_unique<PIDDroneController>(static_cast<float> (POLLING_RATE));
    auto bat = std::make_unique<battery>(batteryConfig);
    drone = std::make_unique<droneBody>(
        std::move(bat),
        std::move(controller));
    
    drone->initDrone(droneConfig);

    drone->controller->initController(droneConfig,*drone);

    drone->offsetCOG({0,0,0});

    propeller prop(propellerConfig);
    motor mot(motorConfig,drone->getTimeStep());

    auto motorPropPair = setSquare(0.3f,0.3f,prop,mot);
    drone->addMotorsAndProps(motorPropPair);
}

void unrealDrone::setTargetPosition(float x , float y , float z , float yaw){

    drone->controller->setTargetPosition(x,y,z);

} 

void unrealDrone::iterator(float totalTime){
    while(drone->getTime() < totalTime /*&&  drone->getPositionVector()[2]> 0 */){
        drone->updateState();
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
    poseState stateVectors = drone->getPose();
    packet.dirVector[0] = stateVectors.dirVector[0];
    packet.dirVector[1] = stateVectors.dirVector[1];
    packet.dirVector[2] = stateVectors.dirVector[2];

    packet.fwdVector[0] = stateVectors.fwdVector[0];
    packet.fwdVector[1] = stateVectors.fwdVector[1];
    packet.fwdVector[2] = stateVectors.fwdVector[2];

    packet.rightVector[0] = stateVectors.rightVector[0]; 
    packet.rightVector[1] = stateVectors.rightVector[1];
    packet.rightVector[2] = stateVectors.rightVector[2];
    packet.timeStamp = drone->getTime();
}



}