#ifndef DRONEPIDCONTROL_H
#define DRONEPIDCONTROL_H
#pragma once
#include "../dynamics/vehicle.h"
#include "../subsystems/motor.h"
#include "../subsystems/battery.h"
#include "../core/quaternion.h"
#include "../core/indexVectors.h"
#include "PIDController.h"
#include "../subsystems/propeller.h"
#include "../dynamics/aero.h"
#include <utility>
#include <memory>
#include <string>
#include "../core/vectorMath.h"
#include "../utility/utility.h"
#include "droneControllerBase.h"
#include "../core/poseRotation.h"
#include "../utility/graphing.h"

namespace SimCore{
    

struct accelerations{
    float xAccel;
    float yAccel;
    float zAccel;

    accelerations():xAccel(0.0f),
    yAccel(0.0f),
    zAccel(0.0f) {}
};
struct requestedVehicleState{
    threeDState vehicleState;
    float force;
};

struct momentForceRequest{
    threeDState moments = {0,0,0};
    threeDState forces = {0,0,0};
};


//accelerationFromPose takes the current pose of the vehicle and calculates the required thrust given the CURRENT conditions.
//if aotFeedForwad calculates the thrust needed at the desired angle of attack for the desired postion and not the current position the motors will generate the thrust required faster than the vehicle can rotate to the position.
// resulting in over delay from thrust to angle requested
static float accelerationFromPose(const threeDState accels,poseState& pose){
    threeDState newAccelVector;
    auto dir = pose.dirVector;

    newAccelVector = scaleVectorToZ(dir,accels[2]);
    
    return vectorMag(newAccelVector);
}

class droneControllerBase;

//Gives a high level state request that is handled down stream
class PIDDroneController : public droneControllerBase{
    private:
    //high level positional control loop
    unique_ptr<PIDController> PIDX;
    unique_ptr<PIDController> PIDY;
    unique_ptr<PIDController> PIDZ;
    //mid level velocity Control.
    unique_ptr<PIDController> PIDVX;
    unique_ptr<PIDController> PIDVY;
    unique_ptr<PIDController> PIDVZ;
    //low level vehicle angle control loop
    //only a single APID is needed to control the vehicles normal axis becuase the cross product is used to find the moment axis.
    unique_ptr<PIDController> APIDVX;
    unique_ptr<PIDController> APIDVY;

    unique_ptr<PIDController> APIDX;
    unique_ptr<PIDController> APIDY;
    float gravitationalAcceleration;
    float mass; 
    float maxAcceleration;

    stateInfo logEstimationPacket;

    poseState lastState;
    poseAngleDifference poseDifference;
    bool firstPose = true;

    controlPacks::motorOnlyPacket computedControlPacket;    


    utility::grapher graph;

    struct logData{
        threeDState requestedAOT;
        threeDState reportedAOT;
        float AOTerror_Rad;
        threeDState requestedMoment;
        float rollDesiredVelo;
        float pitchDesiredVelo;
        float rollVelo;
        float pitchVelo;
        
    }logger{};

    flight_computer::telemetry telemetry_manager;

    void telemetry() override;



    protected:

    //init

    void initVehicleParams(const std::string& droneConfig);

    void initPositionLoop(const std::string& droneConfig);

    void initVelocityLoop(const std::string& droneConfig);

    void initAngularVelocityLoop(const std::string& droneConfig);

    void initAngularPositionLoop(const std::string& droneConfig);


    struct eulerAOT{
        float angleX;
        float angleY;
        eulerAOT(): angleX(0.0f) , angleY(0.0f) {

        }
    };

    struct eulerMoments {
        std::array<float, 3> moments;

        eulerMoments() : moments{0.0f, 0.0f, 0.0f} {
            
        }
    };

    accelerations computeAccelerationRequest(const std::array<float,3>& pos ,const std::array<float,3>& velo ,float actualDeltaTime);

    eulerAOT computeVehicleBasisAOT(poseState& state , requestedVehicleState& request);

    rotationRate computeAngularVelocityFromAOT(eulerAOT euler, float actualDeltaTime);

    eulerMoments computeMomentFromAOT( poseState& state,rotationRate desiredAngularVelo , float actualDeltaTime);
    
    float computeThrustForce(float requestForce);
    
    public:

    PIDDroneController(float frequency);

    PIDDroneController(const PIDDroneController& other);

    virtual std::unique_ptr<droneControllerBase> clone() const override {
        return std::make_unique<PIDDroneController>(*this);
    }

    //goal is to move towards a predicive model
    void initController(string droneConfig) override;
    virtual void updateCalculatedValues() override;
    /// @brief 
    /// @param pos m
    /// @param velo m/s
    /// @param state Direction vector
    /// @param maxAngleAOT rads
    /// @return std::pair first and second is moments about x and y respectivly. Note this is not global, x and y are realtive to the drone.
    controlPacks::forceMoments pidControl(const std::array<float,3> pos ,const std::array<float,3> velo  ,poseState& state);
    void setTargetPosition(float xTarget , float yTarget , float zTarget) override;
    //feedForward function for windprediction and gravity offset.
    requestedVehicleState aotFeedForward(accelerations& accelerationCommand, float mass,poseState& pose);
    //std::array<float,3> thrustMoment(const propeller& prop ,const motor& mot, std::array<float,3>& cogLocation ,const float& airDensity);


    controlPacks::variantPackets update(float time,stateInfo statePacket) override;
    //AOT HOLD neglects any side ways movement cause by the AOT.
    controlPacks::forceMoments AOTHold(float actualDeltaTime, poseState state , poseState requestedPose);

    controlPacks::variantPackets updateAOTHold(float time,stateInfo statePacket , poseState requestedPose);

    //return Moment
    float yawControl();

    float yawAngleDifference(const poseState& currentPose ,const poseState& desiredPose);

    controlPacks::forceMoments updateWithoutAllocator(float time,stateInfo statePacket) override;

    /**
     * @return pair first is the axis of rotation the second is the pid return clamped 1 to -1.The output should be converted into a deired moment.
     */
    std::pair<std::array<float,3> , float> aotControl(requestedVehicleState request, std::array<float,3> currentState);

    void generateGraph(){
        graph.plot();
    }

    logData getLogs(){
        return logger;
    }

    inline void reset() {
        if (PIDX) PIDX->reset();
        if (PIDY) PIDY->reset(); 
        if (PIDZ) PIDZ->reset();

        if (PIDVX) PIDVX->reset();
        if (PIDVY) PIDVY->reset();
        if (PIDVZ) PIDVZ->reset();

        if (APIDX) APIDX->reset();
        if (APIDY) APIDY->reset();
        controlPacks::motorOnlyPacket newPacket;
        computedControlPacket = newPacket;
        controlOutputVelocity = {0.0f, 0.0f, 0.0f};
    }

    inline void setPIDXGains(const std::tuple<float, float, float>& pid) {
        if (PIDX) PIDX->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setPIDYGains(const std::tuple<float, float, float>& pid) {
        if (PIDY) PIDY->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setPIDZGains(const std::tuple<float, float, float>& pid) {
        if (PIDZ) PIDZ->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setPIDVXGains(const std::tuple<float, float, float>& pid) {
        if (PIDVX) PIDVX->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setPIDVYGains(const std::tuple<float, float, float>& pid) {
        if (PIDVY) PIDVY->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setPIDVZGains(const std::tuple<float, float, float>& pid) {
        if (PIDVZ) PIDVZ->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }
    
    inline void setAPIDXGains(const std::tuple<float, float, float>& pid) {
        if (APIDX) APIDX->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }

    inline void setAPIDYGains(const std::tuple<float, float, float>& pid) {
        if (APIDY) APIDY->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }

    inline void setAPIDVXGains(const std::tuple<float, float, float>& pid) {
        if (APIDVX) APIDVX->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }

    inline void setAPIDVYGains(const std::tuple<float, float, float>& pid) {
        if (APIDVY) APIDVY->setGains(std::get<0>(pid), std::get<1>(pid), std::get<2>(pid));
    }

    virtual std::string display() override{
        std::ostringstream buffer;
        buffer << "-----Controller State-----\n";
        
        buffer << "Requested AOT: ["
           << logger.requestedAOT[0] << ", "
           << logger.requestedAOT[1] << ", "
           << logger.requestedAOT[2] << "]\n";

        buffer << "Reported AOT: ["
                << logger.reportedAOT[0] << ", "
                << logger.reportedAOT[1] << ", "
                << logger.reportedAOT[2] << "]\n";
        
        buffer <<"Error : [" <<logger.AOTerror_Rad << "]\n";


        buffer << "Requsted Moment: ["
                << logger.requestedMoment[0] << ", "
                << logger.requestedMoment[1] << ", "
                << logger.requestedMoment[2] << "]\n";


        std::string bufferString = buffer.str() + "\n" + droneControllerBase::display();
        return bufferString;

    }

};

/// @brief Converts quaternion style axis + moment to eular moments.
/// @param axisOfRotation unitless
/// @param moment n*m
/// @return array where x,y,z represet moments around the axis {1,0,0},{0,1,0},and {0,0,1}.
inline std::array<float,3> axisMomentToEulerMoment(std::array<float,3> axisOfRotation, float moment ){

    axisOfRotation = normalizeVector(axisOfRotation);
    for(int i = 0 ; i < axisOfRotation.size() ; i++){
        axisOfRotation[i] = axisOfRotation[i] * moment;
    }
    return axisOfRotation;
}

inline std::array<float, 3> limitMagnitudeWithFixedZ(std::array<float, 3> vect, float magLimit) {
    float currentMag = vectorMag(vect);
    if (currentMag <= magLimit) return vect;
    
    if (magLimit <= vect[2]) {
        return {0.0f, 0.0f, vect[2]};
    }
    
    float maxXY2 = magLimit * magLimit - vect[2] * vect[2];  
    
    // Add safety check in case of floating point errors
    if (maxXY2 <= 0) {
        return {0.0f, 0.0f, vect[2]};
    }
    
    float xyMag = std::sqrt(vect[0] * vect[0] + vect[1] * vect[1]);
    if (xyMag == 0.0f) {
        return {0.0f, 0.0f, vect[2]};
    }
    
    float scale = std::sqrt(maxXY2) / xyMag;
    return {vect[0] * scale, vect[1] * scale, vect[2]};
}




}

#endif