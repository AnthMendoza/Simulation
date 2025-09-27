#ifndef MOTORDYNO_H
#define MOTORDYNO_H
#include "motor.h"
#include "propeller.h"
#include "../dynamics/aero.h"
#include <vector>
#include <algorithm>
#include <iostream>
namespace SimCore{
/// @brief Not passing as pointers or refrance to intentionally create copys for issolated testing
/// @param mot 
/// @param prop 
/// @param bat 
/// @return return in pair. first is the lower limit second is the upper limit of thrust.
std::pair<float,float> thrustLimits(motor mot, propeller prop, battery bat , float timeStep){
    if(timeStep <= 0) throw std::runtime_error("timeStep in thrustLimits cannot be <= 0");
    //air Density at sea level.
    float density = SimCore::airDensity(0);
    float max = 0;
    float maxCurrent = 0;
    float maxAngularVeocity = 0;
    float maxDrag = 0;
    for(int i = 0 ; i < 20.0f/timeStep;i++){
        if(max <  prop.thrustForce(density,mot.getCurrentAngularVelocity())) max = prop.thrustForce(density,mot.getCurrentAngularVelocity());
        if(maxCurrent < mot.getCurrentCurrent()) maxCurrent = mot.getCurrentCurrent();
        if(maxAngularVeocity < mot.getCurrentAngularVelocity()) maxAngularVeocity = mot.getCurrentAngularVelocity();
        float currentDrag = prop.dragTorque(density, mot.getCurrentAngularVelocity());
        if(maxDrag < currentDrag) maxDrag = currentDrag;
        float altitude = 10;
        mot.updateMotor(timeStep,prop.dragTorque(density,mot.getCurrentAngularVelocity()),bat.getBatVoltage());
        float current = std::abs(mot.getCurrentCurrent());
        float thrust = prop.thrustForce(density,mot.getCurrentAngularVelocity());
        bat.updateBattery(current,timeStep * i);
    }
    std::pair<float,float> result = {0,max}; 
    //for now 0 is the minimum. can add reverse if propeller is bidirectional
    return result;
}

}

#endif