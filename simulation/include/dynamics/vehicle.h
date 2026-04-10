#ifndef VEHICLE_H
#define VEHICLE_H


#include <array>
#include <memory>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <string>
#include <optional>
#include <variant>
#include <base/base.h>
#include <util/thread_manager.h>
#include "../subsystems/sensors.h"
#include "../sim/logs.h"
#include "windGenerator.h"
#include "../subsystems/sensors.h"
#include <controlRequestStructs.h>
#include "../subsystems/droneSensorSuite.h"
#include "../subsystems/sensorPacket.h"
#include <util/time_manager.h>
#include <base/transposeSet.h>

namespace SimCore{
class stateEstimation;
class StanleyController;
 

class Vehicle : public thread_manager{
    private:
    protected:
    float Xposition , Yposition , Zposition;     // position 
    float Xvelocity , Yvelocity , Zvelocity;
    float timeStep;
    float lastTime;

    float mass; 
    float centerOfPressure;
    float gForce;

    std::unique_ptr<quaternionVehicle> pose;
    std::unique_ptr<timeManager> manager;
   
    units::vec3 wind;
    units::vec3 angularVelocity;
    units::vec3 vehicleState;
    units::vec3 MOI;
    units::vec3 sumOfForces;
    units::vec3 sumOfMoments;
    units::vec3 logSumOfMoments;
    float yawMoment;
    units::vec3 acceleration;
    float gravitationalAcceleration;

    virtual void init(string& vehicleConfig);

    transposeSet entities;

    public:

    std::unique_ptr<sensorSuite<simpleSensorPacket>> sensors;
    std::string configFile;
    std::string outputFile;

    Vehicle();

    virtual ~Vehicle() = default;

    Vehicle(const Vehicle& other);

    virtual std::unique_ptr<Vehicle> clone() const = 0;

    template<typename T>
    void setSensorSuite(std::unique_ptr<T> setSensors){
        static_assert(std::is_base_of_v<sensorSuite<simpleSensorPacket>, T>, "T must derive from sensorSuite");
        sensors = std::move(setSensors);
    }

    virtual Vehicle& operator=(const Vehicle& other);


    void addForce(units::vec3 forceVector);

    void addMoment(units::vec3 moments);
    //Positive moment about the direction vector is a rotation from x to y.
    //Negative from y to x.
    void addYawMoment(float moment);

    virtual void rotateLocalEntities(const Quaternion& quant);

    virtual void updateState(float time,std::optional<controlPacks::variantPackets> controlInput = nullopt);

    virtual void drag(float (*aeroArea)(float),float (*coefOfDrag)(float));

    virtual void lift(float (*aeroArea)(float),float (*coefOfLift)(float));

    virtual void initSensors();

    units::vec3 getAcceleration();

    float getVelocity();

    float getGForce();

    void turbulantWind();

    std::unique_ptr<turbulence> turbulantX;
    std::unique_ptr<turbulence> turbulantY;
    std::unique_ptr<turbulence> turbulantZ;

    void getAccel(units::vec3 &accel);

    float PID(float target , float currentState , float &previousError , float &sumOfError, float timeStep, float Pgain , float Igain , float Dgain);

    inline void updateAcceleration(){
        acceleration = {sumOfForces[0]/mass , sumOfForces[1]/mass , sumOfForces[2]/mass};
    }

    virtual void publish_simulation() = 0;

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
    virtual void setEntitiesPose(const poseState& pose) = 0;

    inline void setStateVector(units::vec3 dirVector, units::vec3 fwdVector){
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
        setEntitiesPose(pose->getPose());
    }

    inline void setPose(poseState state){
        setStateVector(state.dirVector,state.fwdVector);
    }

    //#############################################################################
    //GETTERS

    //iteratoins * timestep
    inline float getTime() const {
        return lastTime;
    }
    inline float getDeltaTime(){
        return timeStep;
    }
    //Not based off sensor data. Actual Simulation Position
    inline units::vec3 getVelocityVector() const{
        return {Xvelocity,Yvelocity,Zvelocity};
    }
    inline units::vec3 getMoment() const{
        return logSumOfMoments;
    }
    inline units::vec3 getPositionVector() const{
        return {Xposition,Yposition,Zposition};
    }
    //Vehicle state is the direction vector of the vehicle. 
    inline units::vec3 getState() const{
        return vehicleState;
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

    void thread_process() override;

    void thread_startup_process() override;


    
    //#############################################################################

    virtual std::string display() const {
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
        buffer << std::fixed << std::setprecision(4);
        auto moments = getMoment();
        buffer << "Moments (mx,my,mz): (" << moments[0] << ", " << moments[1] << ", " << moments[2] << ")\n";
        

        return buffer.str();
    }


};

}


#endif