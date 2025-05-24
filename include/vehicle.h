#ifndef VEHICLE_H
#define VEHICLE_H

#pragma once
#include <array>
#include <memory>
#include <cmath>
#include "sensors.h"
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
    float timeStep;
    int iterations;

    float mass; 
    float centerOfPressure;
    float gForce;
    
    std::array<float,3> wind;
    std::array<float,3> angularVelocity;
    std::array<float,3> vehicleState;
    std::array<float,3> MOI;
    std::array<float,3> sumOfForces;
    std::array<float,3> sumOfMoments;
    std::array<float,3> acceleration;
    float gravitationalAcceleration;
    public:
    std::string configFile;
    std::string outputFile;
    Vehicle();

    Vehicle(const Vehicle& vehicle) = delete;

    virtual void init();
    
    virtual Vehicle& operator=(const Vehicle& other);

    void operator++();

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
        return iterations * timeStep;
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
        iterations = it;
    }

    inline float getMass(){
        return mass;
    }
    inline float getTimeStep(){
        return timeStep;
    }



};
}

#endif