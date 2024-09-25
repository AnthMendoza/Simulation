#include <iostream>

#include "../include/vehicle.h"




Vehicle::Vehicle(float x, float y, float z, float mroll, float mpitch, float myaw , float mMOI[3])
    : Xposition(x), Yposition(y), Zposition(z), roll(mroll), pitch(mpitch), yaw(myaw){

    memcpy(MOI, mMOI, 3 * sizeof(float));
    }



// Method implementation to display the vehicle's state
void Vehicle::display() {
    std::cout << "Position: (" << Xposition << ", " << Yposition << ", " << Zposition << ")\n"
              << "Orientation (Roll, Pitch, Yaw): (" << roll << ", " << pitch << ", " << yaw << ")\n"<< MOI[2];
}

void Vehicle::updateState(){

}