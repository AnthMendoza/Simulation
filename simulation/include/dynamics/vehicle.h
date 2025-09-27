#ifndef VEHICLE_H
#define VEHICLE_H

#pragma once
#include <array>
#include <memory>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include "../subsystems/sensors.h"
#include "../core/vectorMath.h"
#include "../control/control.h"
#include "../core/quaternion.h"
#include "../sim/logs.h"
#include "windGenerator.h"
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
    float dt; //delta time

    std::unique_ptr<quaternionVehicle> pose;
   
    std::array<float,3> wind;
    std::array<float,3> angularVelocity;
    std::array<float,3> vehicleState;
    std::array<float,3> MOI;
    std::array<float,3> sumOfForces;
    std::array<float,3> sumOfMoments;
    std::array<float,3> logSumOfMoments;
    float yawMoment;
    std::array<float,3> acceleration;
    float gravitationalAcceleration;
    public:
    std::string configFile;
    std::string outputFile;
    Vehicle();

    Vehicle(const Vehicle& other);


    virtual void init(string& vehicleConfig);
    
    virtual Vehicle& operator=(const Vehicle& other);

    void operator++();

    void addForce(std::array<float,3> forceVector);

    void addMoment(std::array<float,3> moments);
    //Positive moment about the direction vector is a rotation from x to y.
    //Negative from y to x.
    void addYawMoment(float moment);

    virtual void rotateLocalEntities(const Quaternion& quant);

    virtual void updateState();

    virtual void drag(float (*aeroArea)(float),float (*coefOfDrag)(float));

    virtual void lift(float (*aeroArea)(float),float (*coefOfLift)(float));

    virtual void initSensors();

    float getVelocity();

    float getGForce();

    void turbulantWind();

    std::unique_ptr<turbulence> turbulantX;
    std::unique_ptr<turbulence> turbulantY;
    std::unique_ptr<turbulence> turbulantZ;

    void getAccel(std::array<float,3> &accel);

    float PID(float target , float currentState , float &previousError , float &sumOfError , float timeStep, float Pgain , float Igain , float Dgain);

    inline void updateAcceleration(){
        acceleration = {sumOfForces[0]/mass , sumOfForces[1]/mass , sumOfForces[2]/mass};
    }

    //#############################################################################
    //SETTERS
    inline void setDeltaTime(float seconds){
        dt = seconds;
    }
    inline void setPositionVector(float x ,float y, float z){
        Xposition = x;
        Yposition = y;
        Zposition = z;
    }
    inline void setVelocity(float vx ,float vy, float vz){
        Xvelocity = vx;
        Yvelocity = vy;
        Zvelocity = vz; 
    }
    virtual void setEntityPose(quaternionVehicle pose) = 0;

    inline void setStateVector(std::array<float,3> dirVector, std::array<float,3> fwdVector){
        if(isZeroVector(dirVector)){
            std::cerr<< "\nsetStateVector was given a Zero vector as the new Vehicle State dirVector.\n Command was skipped.\n";
            return;
        }
        if(isZeroVector(fwdVector)){
            std::cerr<< "\nsetStateVector was given a Zero vector as the new Vehicle State fwdVector.\n Command was skipped.\n";
            return;
        }
        normalizeVectorInPlace(dirVector);
        normalizeVectorInPlace(fwdVector);
        pose->setVehicleQuaternionState(dirVector,fwdVector);
        quaternionVehicle newPose = *pose;
        setEntityPose(newPose);
    }

    inline void setIterations(int it){
        iterations = it;
    }

    //#############################################################################
    //GETTERS

    //iteratoins * timestep
    inline float getTime() const {
        return iterations * timeStep;
    }
    inline float getDeltaTime(){
        return dt;
    }
    //Not based off sensor data. Actual Simulation Position
    inline std::array<float,3> getVelocityVector() const{
        return {Xvelocity,Yvelocity,Zvelocity};
    }
    inline std::array<float,3> getMoment() const{
        return logSumOfMoments;
    }
    inline std::array<float,3> getPositionVector() const{
        return {Xposition,Yposition,Zposition};
    }
    //Vehicle state is the direction vector of the vehicle. 
    inline std::array<float,3> getState() const{
        return vehicleState;
    }
    inline int getIterations() const{
        return iterations;
    }
    inline float getMass() const{
        return mass;
    }
    inline float getTimeStep() const{
        return timeStep;
    }
    inline poseState getPose() const{
        return pose->getPose();
    }
    inline float getGravitationalAcceleration() const{
        return gravitationalAcceleration;
    }

    
    //#############################################################################

    inline std::string display() const {
        std::ostringstream buffer;

        buffer << std::fixed << std::setprecision(2);
        buffer << "Drone Analytics : Time(S) " << getTime() << "\n";
        buffer << "-----Vehicle State-----\n";
        auto pos = getPositionVector();
        buffer << "Position     (x, y, z):      (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")\n";

        auto velo = getVelocityVector();
        buffer << "Velocity     (vx, vy, vz):   (" << velo[0] << ", " << velo[1] << ", " << velo[2] << ")\n";

        buffer << "wind    (vx, vy, vz):   (" << wind[0] << ", " << wind[1] << ", " << wind[2] << ")\n";

        buffer << "Acceleration (ax, ay, az):   (" << acceleration[0] << ", " << acceleration[1] << ", " << acceleration[2] << ")\n";

        auto displayPose = getPose();
        buffer << "Orientation (top vector):   (" << displayPose.dirVector[0] << ", " << displayPose.dirVector[1] << ", " << displayPose.dirVector[2] << ")\n";
        buffer << "Orientation (front vector): (" << displayPose.fwdVector[0] << ", " << displayPose.fwdVector[1] << ", " << displayPose.fwdVector[2] << ")\n";

        auto moments = getMoment();
        buffer << "Moments (mx,my,mz): (" << moments[0] << ", " << moments[1] << ", " << moments[2] << ")\n";
        
        buffer << "-----State Estimation-----\n";

        auto estimatedPos = getEstimatedPosition();
        buffer << "Position     (x, y, z):      (" << estimatedPos[0] << ", " << estimatedPos[1] << ", " << estimatedPos[2] << ")\n";

        auto estimatedVelo = getEstimatedVelocity();
        buffer << "Velocity     (vx, vy, vz):   (" << estimatedVelo[0] << ", " << estimatedVelo[1] << ", " << estimatedVelo[2] << ")\n";
        return buffer.str();
    }

};
}

#endif