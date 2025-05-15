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


class Rocket : public Vehicle{
    private:

    float finErrorX;
    float finErrorY;
    float finX;
    float finY;
    float sumOfFinErrorX;
    float sumOfFinErrorY;
    
    float finVelocityX;
    float finVelocityY;
    float finDamping;
    float finPGain;
    float finIGain;
    float finDGain;
    float maxFinAcceleration;


    protected:
    float gimbalErrorX;
    float gimbalErrorY;

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

    float cogToEngine;

    float liftAngleLog;

    bool reentry;

    bool glidePhase;

    bool landingInProgress;


    float vehicleYError;
    float sumOfVehicleYError;

    float vehicleXError;
    float sumOfVehicleXError;

    float maxGimbalAngle;

    float logXInput;
    float logYInput;

    float logPosX;
    float logPosY;

    std::array<float , 3> appliedVector;


    std::array<std::array<float,3> , 2> finVectors;

    std::array<float,3> targetLandingPosition;
    std::array<float,3> logEngineVector;
    std::array<float,3> logMoment;
    
    std::shared_ptr<StanleyController> Stanley;
    public:
    float gimbalX;
    float gimbalY;
    float dryMass , fuel , LOX , fuelConsumptionRate , LOXConsumptionRate ;

    std::array<float,3> engineState;
    float engineForce;
    //used to send data to simulation for fire animation, no physics is done
    float enginePower;

        Rocket();

        Rocket(const Rocket& vehicle);

        void init() override;

        void operator++(int) override;

        float getVelocity();

        void drag() override;

        void lift() override;

        void applyEngineForce(std::array<float,2> twoDEngineRadians , float thrust);

        void updateState() override;

        std::array<std::array<float , 3> , 2> getFinForceVectors();

        void applyFinForce(float xForce , float yForce);

        float getCurvature();

        bool fuelConsumption(float thrust);

        void engineGimbal(float gimbalTargetX , float gimbalTargetY);

        void initSensors() override;

        void updateFinPosition(std::pair<float,float> commands);

        inline std::array<float,3> getRadarPosition(){
            std::array<float,3> radar = {0,0,0};
            return radar;
        }
        
        //kenetic + potential energy per unit mass
        inline float getSpecificEnergy(){
            std::array<float,3> estimatedVelo =getEstimatedVelocity();
            float velo = vectorMag(estimatedVelo);
            return  (velo * velo)/2 + std::abs(constants::gravitationalAcceleration * Zposition);
        }
        // kenetic + potential energy per unit mass in vector form {x,y,z}
        inline std::array<float,3> getVectorizedEnergy(){
            std::array<float,3> estimatedVelo = getEstimatedVelocity();
            return {estimatedVelo[0]*estimatedVelo[0]/2 , estimatedVelo[1]*estimatedVelo[1]/2 , (estimatedVelo[2]*estimatedVelo[2] / 2) + std::abs(constants::gravitationalAcceleration * getEstimatedPosition()[2])};
        }


    //Vehicle &lookAhead( float lookAheadTime);

        
    std::vector<float> lookAhead(Rocket &rocket,float lookAheadTime , std::function<float(Rocket&)> valueToLog);

    void reentryBurn(loggedData *data = nullptr);

    void glideToTarget();

    void landingBurn();

    //only use prior to simulation
    //not a hard rule, wont be enforced
    inline void setPosition(float Xpos,float Ypos,float Zpos){
        Xposition = Xpos;
        Yposition = Ypos;
        Zposition = Zpos;
    }
    //only use prior to simulation
    //not a hard rule, wont be enforced
    inline void setVelocity(float Xvelo,float Yvelo,float Zvelo){
        Xvelocity = Xvelo;
        Yvelocity = Yvelo;
        Zvelocity = Zvelo;
    }

};



class drone :  public Vehicle{


};






#endif