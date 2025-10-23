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
#include "../subsystems/sensors.h"
#include "../core/vectorMath.h"
#include "../control/control.h"
#include "../core/quaternion.h"
#include "../sim/logs.h"
#include "windGenerator.h"
#include "../subsystems/sensors.h"
#include "../control/controlRequestStructs.h"
#include "../subsystems/droneSensorSuite.h"
#include "../subsystems/sensorPacket.h"
#include "../utility/time_manager.h"

namespace SimCore{
class stateEstimation;
class StanleyController;
 

class Vehicle{
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

    virtual void init(string& vehicleConfig);
    public:
    std::unique_ptr<sensorSuite<simpleSensorPacket>> sensors;
    std::string configFile;
    std::string outputFile;
    Vehicle();

    virtual ~Vehicle() = default;

    Vehicle(const Vehicle& other);

    template<typename T>
    void setSensorSuite(std::unique_ptr<T> setSensors){
        static_assert(std::is_base_of_v<sensorSuite<simpleSensorPacket>, T>, "T must derive from sensorSuite");
        sensors = std::move(setSensors);
    }

    virtual Vehicle& operator=(const Vehicle& other);


    void addForce(std::array<float,3> forceVector);

    void addMoment(std::array<float,3> moments);
    //Positive moment about the direction vector is a rotation from x to y.
    //Negative from y to x.
    void addYawMoment(float moment);

    virtual void rotateLocalEntities(const Quaternion& quant);

    virtual void updateState(float time,std::optional<controlPacks::variantPackets> controlInput = nullopt);

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

    float PID(float target , float currentState , float &previousError , float &sumOfError, float timeStep, float Pgain , float Igain , float Dgain);

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
    virtual void setEntitiesPose(const poseState& pose) = 0;

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
        setEntitiesPose(pose->getPose());
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

        auto moments = getMoment();
        buffer << "Moments (mx,my,mz): (" << moments[0] << ", " << moments[1] << ", " << moments[2] << ")\n";
        

        return buffer.str();
    }

};
}

#endif