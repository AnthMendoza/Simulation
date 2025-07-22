#ifndef PIDGAINS_H
#define PIDGAINS_H
#include "../dynamics/drone.h"
#include <tuple>
#include "../core/pythonConnector.h"
#include "../utility/utility.h"
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
    testDrone->setStateVector({0,0,1},{1,0,0});
    testDrone->setVelocity(0.0f,0.0f,0.0f);
    testDrone->setMotorHover();

    testController->setpidControl(0.0f,0.0f,4.0f);
}


bool hoverTestIterator(droneBody* testDrone,calibratePID* calibrate){
    testDrone->setStateVector({0,0,1},{1,0,0});
    testDrone->updateState();
    return calibrate->logStep(testDrone->getEstimatedPosition()[2] , 4.0f ,testDrone->getTimeStep());
}

PIDFunctionGroup<dataPID> hoverGroup = {
    hoverTestIterator,
    hoverTestSetup,
    hoverSetGain,
    0.2f,
    0.1f,
    1000.0f
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
    testDrone->setStateVector({0,0,1},{1,0,0});
    testDrone->setVelocity(0.0f,0.0f,0.0f);
    testDrone->setMotorHover();

    testController->setpidControl(0.0f,0.0f,wayPoint);
}


bool hoverTestIteratorDuel(droneBody* testDrone,calibratePID* calibrate){
    testDrone->setStateVector({0,0,1},{1,0,0});
    testDrone->updateState();
    return calibrate->logStep(testDrone->getEstimatedPosition()[2] , wayPoint ,testDrone->getTimeStep());
}

PIDFunctionGroup<PIDPair> hoverGroupDuel = {
    hoverTestIteratorDuel,
    hoverTestSetupDuel,
    hoverSetGainDuel,
    0.2f,
    0.1f,
    1000.0f
};

//##########################################################################
//Angle of Attack(aot)

static constexpr float ALTITUDE = 10.0f;
static constexpr float arbitraryLargeDistance = 10000;

void aotSetGain(droneControl* testController , dataPID PID){
    testController->setAPIDXGains(PID);
    testController->setAPIDYGains(PID);
}

void aotTestSetup(droneBody* testDrone,droneControl* testController, dataPID PID){
    //set drone vertically 
    testDrone->resetMotors();
    testDrone->setPositionVector(0.0f,0.0f,ALTITUDE);
    testDrone->setStateVector({0,0,1},{1,0,0});
    testDrone->setVelocity(0.0f,0.0f,0.0f);
    testDrone->setMotorHover();
    testDrone->controller->setpidControl(0.0f,arbitraryLargeDistance,ALTITUDE);
}


bool aotTestIterator(droneBody* testDrone,calibratePID* calibrate){
    testDrone->controller->setpidControl(0.0f,arbitraryLargeDistance,ALTITUDE);
    
    testDrone->updateState();
    
    float angleFromRefrance = vectorAngleBetween({0,0,1},testDrone->getState());
    float error = angleFromRefrance - testDrone->maxAngleAOT;
    return calibrate->logStep(error,0,testDrone->getTimeStep());
}

PIDFunctionGroup<dataPID> aotGroup = {
    aotTestIterator,
    aotTestSetup,
    aotSetGain,
    0.02f,
    0.1f,
    1000.0f
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
    testDrone->setStateVector({0,0,1},{1,0,0});
    testDrone->setVelocity(0.0f,0.0f,0.0f);
    testDrone->setMotorHover();

    testController->setpidControl(0.0f,horizontalWayPoint,testDrone->getPositionVector()[2]);
}


bool rollPitchTestIterator(droneBody* testDrone,calibratePID* calibrate){
    testDrone->updateState();
    return calibrate->logStep(testDrone->getEstimatedPosition()[1] , horizontalWayPoint ,testDrone->getTimeStep());
}

PIDFunctionGroup<PIDPair> rollPitchGroup = {
    rollPitchTestIterator,
    rollPitchTestSetup,
    rollPitchSetGain,
    0.2f,
    0.1f,
    1000.0f
};

//##################################################


}

#endif