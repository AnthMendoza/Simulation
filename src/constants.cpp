#include "../include/constants.h"


namespace constants{
    float gravitationalAcceleration = -9.81; 

    std::array<float,3> initPosition = {0,295528.164398,110000};
    //std::array<float,3> initPosition = {0,0,1000};
    std::array<float,3> wind = {0,0,0}; //  m/s x,y,z  this will be stacked on top of whatever the current velocity is on an absolute cordinate system.
    std::array<float,3> initVehicleState = {0,1,0}; // this the vehicle vector, its direction is the directionthe nose points. it will be normilized, magnatiude is meain
    std::array<float , 3> MOI = {3614090,3614090,3614090};
    std::array<float,3> initVelocity = {0,-2000,0}; // m/s

    
    float maxGAllowedEntry = 4.5;
    float centerOfPressure = 15;
    float cogToEngine = 8; // meters use this to calculate the moment created by the engine this is negative becuase center of pressure on the opposite side of the COG is positive
    const float timeStep = .0001; //seconds
    float maxThrust = 854000;//netwons
    float minThrust = maxThrust *.65; //newtons, 65 percent of max;
    float mass = 50000; //kg booster mass
    float maxGimbleAngle = 20;// degrees
   


    std::array<float,3> LandingTarget = {};
}