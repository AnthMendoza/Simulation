#include "../include/constants.h"


namespace constants{
    #ifdef __linux__
        bool isLinux = true;
    #else
        bool isLinux = false;
    #endif

    float gravitationalAcceleration = -9.81; 

    //std::array<float,3> initPosition = {0,295528.164398,110000};
   std::array<float,3> initPosition = {0,0,110000};
    std::array<float,3> wind = {0,0,0}; //  m/s x,y,z  this will be stacked on top of whatever the current velocity is on an absolute cordinate system.
    std::array<float,3> initVehicleState = {.5,0.8660254038,0}; // this the vehicle vector, its direction is the directionthe nose points. it will be normilized, magnatiude is meain
    std::array<float , 3> MOI = {3614090,3614090,3614090};
    std::array<float,3> initVelocity = {-1000,-1732.050808,0}; // m/s

    
    float maxGAllowedEntry = 4.5;
    float centerOfPressure = 15;
    float cogToEngine = -8; // meters use this to calculate the moment created by the engine this is negative becuase center of pressure on the opposite side of the COG is positive
    const float timeStep = .01; //seconds
    float maxThrust = 854000;//netwons
    float minThrust = maxThrust *.65; //newtons, 65 percent of max;
    float dryMass = 22200; //kg booster mass
    float maxGimbalAngle = 10  * 3.1415926535f / 180.0f;// degrees
    float landingVelocity = 10;
    float consumptionRateAtFullPowerPerEngine = 328.8; //kg/s this is lox plus RP1

    float initFuel = 10000; // kg
    float initLOX = 10000; // kg
    float consumtionRateLOX = 100; //kg/s
    float consumtionRateFuel = 100; //kg/s


    std::array<float,3> LandingTarget = {0,-257939.2813,0};



    //bounds
    



}