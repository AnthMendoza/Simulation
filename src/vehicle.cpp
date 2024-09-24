#include <iostream>
#include "../include/vehicle.h"




Vehicle::Vehicle(float x, float y, float z, float mroll, float mpitch, float myaw)
    : Xposition(x), Yposition(y), Zposition(z), roll(mroll), pitch(mpitch), yaw(myaw) {}

// Method implementation to display the vehicle's state
void Vehicle::display() {
    std::cout << "Position: (" << Xposition << ", " << Yposition << ", " << Zposition << ")\n"
              << "Orientation (Roll, Pitch, Yaw): (" << roll << ", " << pitch << ", " << yaw << ")\n";
}