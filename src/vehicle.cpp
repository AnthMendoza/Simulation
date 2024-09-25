#include <iostream>

#include "../include/vehicle.h"




Vehicle::Vehicle(float x, float y, float z, float mroll, float mpitch, float myaw , float mMOI[3])
    : Xposition(x), Yposition(y), Zposition(z), roll(mroll), pitch(mpitch), yaw(myaw){
    memcpy(MOI, mMOI, 3 * sizeof(float));

    Xvelocity = 0;
    Yvelocity = 0;
    Zvelocity = 0;
    rollvelocity = 0;
    pitchvelocity = 0;
    yawvelocity = 0;
    }



// Method implementation to display the vehicle's state
void Vehicle::display() {
    std::cout << "Position: (" << Xposition << ", " << Yposition << ", " << Zposition << ")\n"
              << "Orientation (Roll, Pitch, Yaw): (" << roll << ", " << pitch << ", " << yaw << ")\n"<< MOI[2] << ", " << Xvelocity;
}

void Vehicle::updateState(){

}