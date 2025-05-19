#ifndef VEHICLE_H
#define VEHICLE_H

#pragma once
#include <array>
#include <memory>
#include <cmath>
#include "sensors.h"
#include "constants.h"
#include "vectorMath.h"
#include "control.h"
#include "logs.h"
namespace SimCore{
class stateEstimation;
class StanleyController;


class Vehicle : public stateEstimation{
    private:

    protected:
    float Xposition , Yposition , Zposition;     // position 
    float Xvelocity , Yvelocity , Zvelocity;

    int iterations;

    float mass; 
    float centerOfPressure;
    float gForce;

    std::array<float,3> angularVelocity;
    std::array<float,3> vehicleState;
    std::array<float,3> MOI;
    std::array<float,3> sumOfForces;
    std::array<float,3> sumOfMoments;
    std::array<float,3> acceleration;

    public:
    Vehicle();

    Vehicle(const Vehicle& vehicle);

    virtual void init();

    inline void display() {
        std::cout << "Position: (" << Xposition << ", " << Yposition << ", " << Zposition << ")\n"
                  << "Orientation (Roll, Pitch, Yaw): (" << vehicleState[0] << ", " << vehicleState[1]  << ", " << vehicleState[2]  << ")\n";
    }   

    virtual void operator++(int);

    void addForce(std::array<float,3> forceVector);

    void addMoment(std::array<float,3> moments);

    virtual void updateState();

    virtual void drag();

    virtual void lift();

    virtual void initSensors();

    float getVelocity();

    float getGForce();

    void getAccel(std::array<float,3> &accel);

    float PID(float target , float currentState , float &previousError , float &sumOfError , float timeStep, float Pgain , float Igain , float Dgain);

    inline void updateAcceleration(){
        acceleration = {sumOfForces[0]/mass , sumOfForces[1]/mass , sumOfForces[2]/mass};
    }

    //iteratoins * timestep
    inline float getTime(){
        return iterations * constants::timeStep;
    }
    //Not based off sensor data. Actual Simulation Position
    inline std::array<float,3> getVelocityVector(){
        return {Xvelocity,Yvelocity,Zvelocity};
    }
    inline std::array<float,3> getPositionVector(){
        return {Xposition,Yposition,Zposition};
    }
    //Vehicle state is the direction vector of the vehicle. 
    inline std::array<float,3> getState(){
        return vehicleState;
    }
    inline int getIterations(){
        return iterations;
    }
    inline void setIterations(int it){
        if(it < 0) std::cout<<"Warning : Iterations Cannot be less than 0";
        iterations = it;
    }

    inline float getMass(){
        return mass;
    }



};
}

#endif