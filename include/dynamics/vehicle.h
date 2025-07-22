#ifndef VEHICLE_H
#define VEHICLE_H

#pragma once
#include <array>
#include <memory>
#include <cmath>
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

    virtual void rotateLocalEntities(Quaternion& quant);

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
    inline void setStateVector(std::array<float,3> dirVector, std::array<float,3> fwdVector){
        if(isZeroVector(dirVector)){
            std::cerr<< "\nsetStateVector was given a Zero vector as the new Vehicle State dirVector.\n Command was skipped.\n";
            return;
        }
        if(isZeroVector(fwdVector)){
            std::cerr<< "\nsetStateVector was given a Zero vector as the new Vehicle State fwdVector.\n Command was skipped.\n";
            return;
        }
        pose->setVehicleQuaternionState(dirVector,fwdVector);
    }

    inline void setIterations(int it){
        iterations = it;
    }

    //#############################################################################
    //GETTERS

    //iteratoins * timestep
    inline float getTime(){
        return iterations * timeStep;
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

    
    //#############################################################################

    inline void display() const {
        static const int linesToClear = 11; // number of lines in display

        // Move cursor up to overwrite previous lines
        for (int i = 0; i < linesToClear; ++i)
            std::cout << "\x1b[1A" << "\x1b[2K";  // move up 1 line + clear line

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Drone Analytics : Iteration" << iterations<<"\n";
        std::array<float,3> pos = getPositionVector();
        std::cout << "Position     (x, y, z):      ("
                  << pos[0] << ", "
                  << pos[1] << ", "
                  << pos[2] << ")\n";
        std::array<float,3> velo = getVelocityVector();
        std::cout << "Velocity     (vx, vy, vz):   ("
                  << velo[0] << ", "
                  << velo[1] << ", "
                  << velo[2] << ")\n";
        std::cout << "wind    (vx, vy, vz):   ("
                  << wind[0] << ", "
                  << wind[1] << ", "
                  << wind[2] << ")\n";
        std::cout << "Acceleration (ax, ay, az):   ("
                  << acceleration[0] << ", "
                  << acceleration[1] << ", "
                  << acceleration[2] << ")\n";

        std::cout << "Orientation  (roll, pitch, yaw): ("
                  << vehicleState[0] << ", "
                  << vehicleState[1] << ", "
                  << vehicleState[2] << ")\n";
        auto moments = getMoment();
        std::cout << "Moments (mx,my,mz): ("
                  << moments[0] << ", "
                  << moments[1] << ", "
                  << moments[2] << ")\n";
        std::cout << std::flush;
    }

};
}

#endif