#ifndef VEHICLE_H
#define VEHICLE_H


#include <array>


class Vehicle{
    public:
        float   Xposition , Yposition , Zposition;     // position
        float mass;   
        float Xvelocity , Yvelocity , Zvelocity;
        
        //distance between the center of gravity to the center of pressure, this allows us to not have COG defined explicitly.
        //all forces will be in refrance to the Center of gravity

        float centerOfPressure;

        std::array<float,3> angularVelocity;
        std::array<float,3> vehicleState;
        std::array<float,3> MOI;
        std::array<float,3> sumOfForces;
        std::array<float,3> sumOfMoments;

        Vehicle(float x, float y, float z, std::array<float, 3> mMOI , float mMass);

        void display();

        void drag();

        void lift();

        void addForce(std::array<float,3> forceVector);

        void updateState();

};



#endif