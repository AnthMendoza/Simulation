#ifndef PIDGAINS_H
#define PIDGAINS_H
#include "../dynamics/drone.h"
#include <tuple>
#include "PIDGainsConnector.h"
#include "../sim/utility.h"
#include "../core/vectorMath.h"
#include "PIDTypes.h"
#include "droneCostFunction.h"

namespace SimCore{


//##########################################################################
//hover single PID
//***inital test, not used***

void hoverSetGain(droneControl* testController , dataPID PID){
    testController->setPIDZGains(PID);
}

void hoverSetVeloGain(droneControl* testController , dataPID PID){
    testController->setPIDVZGains(PID);
}



void hoverTestSetup(droneBody* testDrone,droneControl* testController, dataPID PID){
    //set drone vertically 
    testDrone->resetMotors();
    testDrone->setPositionVector(0.0f,0.0f,25.0f);
    testDrone->setStateVector(0.0f,0.0f,1.0f);
    testDrone->setVelocity(0.0f,0.0f,0.0f);
    testDrone->setMotorHover();

    testController->setpidControl(0.0f,0.0f,4.0f);
}


void hoverTestIterator(droneBody* testDrone,calibratePID* calibrate){
    testDrone->setStateVector(0.0f,0.0f,1.0f);
    calibrate->logStep(testDrone->getEstimatedPosition()[2] , 4.0f ,testDrone->getTimeStep());
}

PIDFunctionGroup<dataPID> hoverGroup = {
    hoverTestIterator,
    hoverTestSetup,
    hoverSetGain
};

using PIDPair = std::pair<dataPID,dataPID>;
//##########################################################################
//hover Double

//PID.first positional Gains
//PID.second velocity Gains


void hoverSetGainDuel(droneControl* testController , PIDPair PID){
    testController->setPIDZGains(PID.first);
    testController->setPIDVZGains(PID.second);
}

static constexpr float wayPoint = 12; 

void hoverTestSetupDuel(droneBody* testDrone,droneControl* testController, PIDPair PID){
    //set drone vertically 
    testDrone->resetMotors();
    testDrone->setPositionVector(0.0f,0.0f,10.0f);
    testDrone->setStateVector(0.0f,0.0f,1.0f);
    testDrone->setVelocity(0.0f,0.0f,0.0f);
    testDrone->setMotorHover();

    testController->setpidControl(0.0f,0.0f,wayPoint);
}


void hoverTestIteratorDuel(droneBody* testDrone,calibratePID* calibrate){
    testDrone->setStateVector(0.0f,0.0f,1.0f);
    calibrate->logStep(testDrone->getEstimatedPosition()[2] , wayPoint ,testDrone->getTimeStep());
}

PIDFunctionGroup<PIDPair> hoverGroupDuel = {
    hoverTestIteratorDuel,
    hoverTestSetupDuel,
    hoverSetGainDuel
};

//##########################################################################
//Angle of Attack(aot)



void aotSetGain(droneControl* testController , dataPID PID){
    testController->setAPIDGains(PID);
}
 

void aotTestSetup(droneBody* testDrone,droneControl* testController, dataPID PID){
    //set drone vertically 
    testDrone->resetMotors();
    testDrone->setPositionVector(0.0f,0.0f,10.0f);
    testDrone->setStateVector(0.0f,0.0f,1.0f);
    testDrone->setVelocity(0.0f,0.0f,0.0f);
    testDrone->setMotorHover();

    testController->setpidControl(0.0f,0.0f,10.0f);
}


void aotTestIterator(droneBody* testDrone,calibratePID* calibrate){
    std::array<float,3> desiredAngle = {1,1,1};
    testDrone->controller->aotControl(desiredAngle,testDrone->getState());
    testDrone->setPositionVector(0.0f,0.0f,10.0f);
    float angle = vectorAngleBetween(testDrone->getState(), desiredAngle);
    auto printVec = [](const std::array<float, 3>& vec, const std::string& name) {
    std::cout << name << ": [" 
              << vec[0] << ", " 
              << vec[1] << ", " 
              << vec[2] << "]\n";
};

//printVec(testDrone->getState(), "Current");
//printVec(desiredAngle, "Desired");
    calibrate->logStep(angle,0,testDrone->getTimeStep());
}

PIDFunctionGroup<dataPID> aotGroup = {
    aotTestIterator,
    aotTestSetup,
    aotSetGain
};

//##########################################################################
//roll/pitch
//PID.first positional Gains
//PID.second velocity Gains

void rollPitchSetGain(droneControl* testController , PIDPair PID){
    testController->setPIDXGains(PID.first);
    testController->setPIDYGains(PID.first);
    testController->setPIDVXGains(PID.second);
    testController->setPIDVYGains(PID.second);
}

static constexpr float horizontalWayPoint = 3;

void rollPitchTestSetup(droneBody* testDrone,droneControl* testController, PIDPair PID){
    //set drone vertically 
    testDrone->resetMotors();
    testDrone->setPositionVector(0.0f,0.0f,10.0f);
    testDrone->setStateVector(0.0f,0.0f,1.0f);
    testDrone->setVelocity(0.0f,0.0f,0.0f);
    testDrone->setMotorHover();

    testController->setpidControl(0.0f,horizontalWayPoint,testDrone->getPositionVector()[2]);
}


void rollPitchTestIterator(droneBody* testDrone,calibratePID* calibrate){
    calibrate->logStep(testDrone->getEstimatedPosition()[1] , horizontalWayPoint ,testDrone->getTimeStep());
}

PIDFunctionGroup<PIDPair> rollPitchGroup = {
    rollPitchTestIterator,
    rollPitchTestSetup,
    rollPitchSetGain
};

//##################################################


}

#endif