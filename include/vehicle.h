#ifndef VEHICLE_H
#define VEHICLE_H


#include <array>


class Vehicle{
    public:
        float   Xposition , Yposition , Zposition;     // position 
        float Xvelocity , Yvelocity , Zvelocity;
        
        //distance between the center of gravity to the center of pressure, this allows us to not have COG defined explicitly.
        //all forces will be in refrance to the Center of gravity
        float mass;  
        float centerOfPressure;

        std::array<float,3> angularVelocity;
        std::array<float,3> vehicleState;
        std::array<float,3> MOI;
        std::array<float,3> sumOfForces;
        std::array<float,3> sumOfMoments;

        Vehicle();

        void display();

        void drag();

        void lift();

        void addForce(std::array<float,3> forceVector);

        void  addMoment(std::array<float,3> moments);

        void updateState();

};



#endif