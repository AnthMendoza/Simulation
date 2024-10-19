#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <array>

namespace constants{
    extern float gravitationalAcceleration; 

    extern std::array<float,3> initPosition;
    //std::array<float,3> initPosition = {0,0,1000};
    extern std::array<float,3> wind; //  m/s x,y,z  this will be stacked on top of whatever the current velocity is on an absolute cordinate system.
    extern std::array<float,3> initVehicleState; // this the vehicle vector, its direction is the directionthe nose points. it will be normilized, magnatiude is meain
    extern std::array<float , 3> MOI;
    extern std::array<float,3> initVelocity; // m/s

    extern float maxGAllowedEntry;
    extern float centerOfPressure;
    extern const float timeStep; //seconds
    extern float maxThrust;//netwons
    extern float minThrust; //newtons, 65 percent of max;
    extern float mass; //kg booster mass
    extern float maxGimbleAngle;// degrees
    extern float cogToEngine;
    extern float landingVelocity;
 


    extern std::array<float,3> LandingTarget;
}

#endif