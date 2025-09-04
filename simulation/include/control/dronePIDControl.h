#ifndef DRONEPIDCONTROL_H
#define DRONEPIDCONTROL_H
#pragma once
#include "../dynamics/vehicle.h"
#include "../subsystems/motor.h"
#include "../subsystems/battery.h"
#include "../core/quaternion.h"
#include "../core/indexVectors.h"
#include "../control/PIDController.h"
#include "../subsystems/propeller.h"
#include "../control/droneControl.h"
#include "../dynamics/aero.h"
#include <utility>
#include <memory>
#include <string>
#include "../core/vectorMath.h"
#include "../utility/utility.h"
#include "../control/droneControllerBase.h"

namespace SimCore{

struct accelerations{
    float xAccel;
    float yAccel;
    float zAccel;
};
struct requestedVehicleState{
    threeDState vehicleState;
    float force;
};

struct momentForceRequest{
    threeDState moments = {0,0,0};
    threeDState forces = {0,0,0};
};

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
    unique_ptr<PIDController> APIDX;
    unique_ptr<PIDController> APIDY;
    float gravitationalAcceleation;
    float mass; 
    float maxThrust;
    float maxAcceleration;
    
    public:

    PIDDroneController();
    PIDDroneController(const PIDDroneController& other);

    virtual std::unique_ptr<droneControllerBase> clone() const override {
        return std::make_unique<PIDDroneController>(*this);
    }

    //goal is to move towards a predicive model
    void initController(string droneConfig,const droneBody& drone) override;
    virtual void updateCalculatedValues(const droneBody& drone) override;
    /// @brief 
    /// @param pos m
    /// @param velo m/s
    /// @param state Direction vector
    /// @param maxAngleAOT rads
    /// @return std::pair first and second is moments about x and y respectivly. Note this is not global, x and y are realtive to the drone.
    momentForceRequest pidControl(const std::array<float,3> pos , std::array<float,3> velo  ,poseState& state, float maxThrust);
    void setTargetPosition(float xTarget , float yTarget , float zTarget) override;
    //feedForward function for windprediction and gravity offset.
    requestedVehicleState aotFeedForward(accelerations& accels,float gravitaionalAcceleration , float mass,float maxThrust);

    std::array<float,3> thrustMoment(const propeller& prop ,const motor& mot, std::array<float,3>& cogLocation ,const float& airDensity);
    /// @brief 
    /// @param axisOfRotation 
    /// @param moment n*m

    vector<float> update(const std::array<float,3>& estimatedPostion,poseState state,const std::array<float,3>& estimatedVelocity ,float time ) override;
    /**
     * @return pair first is the axis of rotation the second is the pid return clamped 1 to -1.The output should be converted into a deired moment.
     */
    std::pair<std::array<float,3> , float> aotControl(requestedVehicleState request, std::array<float,3> currentState);


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

    float xyMag = std::sqrt(vect[0] * vect[0] + vect[1] * vect[1]);
    if (xyMag == 0.0f){
        return {0.0f, 0.0f, vect[2]}; 
    }
    
    float scale = std::sqrt(maxXY2) / xyMag;

    return {vect[0] * scale, vect[1] * scale, vect[2]};
}


}

#endif