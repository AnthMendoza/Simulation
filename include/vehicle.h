#ifndef VEHICLE_H
#define VEHICLE_H

#pragma once
#include <array>
#include <memory>
#include "../include/sensors.h"
class stateEstimation;
class Vehicle : public stateEstimation {
    public:
        float   Xposition , Yposition , Zposition;     // position 
        float Xvelocity , Yvelocity , Zvelocity;

        float gimbalErrorX;
        float gimbalErrorY;
        float gimbalX;
        float gimbalY;
        float sumOfGimbalErrorX;
        float sumOfGimbalErrorY;

        float gimbalVelocityX;
        float gimbalVelocityY;
        float gimbalDamping;
        float gimbalPGain;
        float gimbalIGain;
        float gimbalDGain;

        std::array<float ,2 > landingGimbalDirection;
        float landingRequiredThrust;
        float  PIDOutputY;
        float  PIDOutputX; 

        float maxGimbalAcceleration;
        float maxGimbalVelocity;

        float error;
        std::array<float,2> twoDAngle;
        
        //distance between the center of gravity to the center of pressure, this allows us to not have COG defined explicitly.
        //all forces will be in refrance to the Center of gravity
        float mass;  

        float centerOfPressure;

        float cogToEngine;

        float liftAngleLog;

        bool reentry;

        bool glidePhase;

        bool landingInProgress;

        std::array<float,3> engineState;

        int iterations;

        float gForce;

        float engineForce;
        float enginePower; //used to send data to simulation for fire animation, no physics is done

        float dryMass , fuel , LOX , fuelConsumptionRate , LOXConsumptionRate ;

        float vehicleYError;
        float sumOfVehicleYError;

        float vehicleXError;
        float sumOfVehicleXError;

        float maxGimbalAngle;

        float logXInput;
        float logYInput;

        float logPosX;
        float logPosY;

        std::array<float,3> dragLog;

        std::array<float , 3> appliedVector;


        std::array<std::array<float,3> , 2> finVectors;

        std::array<float,3> targetLandingPosition;
        std::array<float,3> angularVelocity;
        std::array<float,3> vehicleState;
        std::array<float,3> MOI;
        std::array<float,3> sumOfForces;
        std::array<float,3> sumOfMoments;
        std::array<float,3> acceleration;
        std::array<float,3> logEngineVector;
        std::array<float,3> logMoment;
        
        

        Vehicle();

        Vehicle(const Vehicle& vehicle);

        void display();

        float getVelocity();

        float getGForce();

        void getAccel(std::array<float,3> &accel);

        void drag();

        void lift();

        void applyEngineForce(std::array<float,2> twoDEngineRadians , float thrust);

        void addForce(std::array<float,3> forceVector);

        void addMoment(std::array<float,3> moments);

        void updateState();

        std::array<std::array<float , 3> , 2> getFinForceVectors();

        void applyFinForce(std::array<std::array<float,3>,2>);

        float getCurvature();

        bool fuelConsumption(float thrust);

        void engineGimbal(float gimbalTargetX , float gimbalTargetY);

        float getTime();

        void initSensors();
    

};



#endif