#ifndef VEHICLE_H
#define VEHICLE_H


#include <iostream>  // Include necessary libraries



class Vehicle{
    public:
        float   Xposition , Yposition , Zposition;     // position
        float    roll , pitch, yaw ;    // rotation

    Vehicle(float x, float y, float z, float mroll, float mpitch, float myaw);

    void display();


};



#endif