#include <iostream>

#include "../include/vehicle.h"
#include "../include/odeIterator.h"





Vehicle::Vehicle(float x, float y, float z, float mroll, float mpitch, float myaw , float mMOI[3])
    : Xposition(x), Yposition(y), Zposition(z), roll(mroll), pitch(mpitch), yaw(myaw){
    memcpy(MOI, mMOI, 3 * sizeof(float));

    Xvelocity = 0;
    Yvelocity = 0;
    Zvelocity = 0;
    rollvelocity = 0;
    pitchvelocity = 0;
    yawvelocity = 0;
    sumOfForces[1] = 0;
    sumOfForces[2] = 0;
    sumOfForces[3] = 0;
    sumOfMoments[1] = 0;
    sumOfMoments[2] = 0;
    sumOfMoments[3] = 0;
    }



// Method implementation to display the vehicle's state
void Vehicle::display() {
    std::cout << "Position: (" << Xposition << ", " << Yposition << ", " << Zposition << ")\n"
              << "Orientation (Roll, Pitch, Yaw): (" << roll << ", " << pitch << ", " << yaw << ")\n"<< MOI[2] << ", " << Xvelocity;
}   

void  Vehicle::addForce(float stateVector[3] , float forceIncident , float forceVector[3]){
    sumOfForces = 
}

void Vehicle::updateState(){        
    Ode(-9.80665*10, 10 ,.001 , Zvelocity,Zposition);
    ;
}