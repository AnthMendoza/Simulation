#ifndef PIDGAINS_H
#define PIDGAINS_H
#include "../dynamics/drone.h"
#include <tuple>
#include "pythonConnector.h"
#include "../utility/utility.h"
#include "../core/vectorMath.h"
#include "PIDTypes.h"
#include "droneCostFunction.h"
using namespace SimCore;

namespace parameterEstimation{

void startUp();


//##########################################################################
//hover single PID
//***inital test, not used***

void hoverSetGain(droneAssets asset , dataPID PID){
    testController->setPIDZGains(PID);
}

void hoverSetVeloGain(droneAssets asset , dataPID PID){
    testController->setPIDVZGains(PID);
}



void hoverTestSetup(droneAssets asset, dataPID PID){
    //set drone vertically 
    testDrone->resetMotors();
    testDrone->setPositionVector(0.0f,0.0f,25.0f);
    testDrone->setStateVector({0,0,1},{1,0,0});
    testDrone->setVelocity(0.0f,0.0f,0.0f);
    testDrone->setMotorHover();

    testController->setTargetPosition(0.0f,0.0f,4.0f);
}




}

#endif