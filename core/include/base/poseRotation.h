#ifndef POSEROTATION_H
#define POSEROTATION_H
#include <optional>
#include "quaternion.h"
#include "vectorMath.h"
#include <cmath>
#include <base/units.h>


namespace SimCore{
//rad
struct rotations{
    units::scalar roll;
    units::scalar pitch;
    units::scalar yaw;
};
//rad/s
struct rotationRate{
    units::scalar rollRate; 
    units::scalar pitchRate;
    units::scalar yawRate;
};


class poseAngleDifference {
private:
    std::optional<poseState> startPose;
    std::optional<poseState> endPose;
public:
poseAngleDifference() = default;
poseAngleDifference(const poseState& startPose, const poseState& endPose):startPose(startPose) , endPose(endPose){

}


inline void setStartPose(const poseState& pose){
    startPose = pose;
    startPose->normalize();
}
inline void setEndPose(const poseState& pose){
    endPose = pose;
    endPose->normalize();
}


rotations getDifference() {
    if(!startPose || !endPose){
        rotations emptyRotations{0,0,0};
        return emptyRotations;
    }

    rotations result;

    units::scalar rollCos = vectorDotProduct(startPose->rightVector, endPose->rightVector);
    std::array<units::scalar,3> rollCross;
    vectorCrossProduct( startPose->rightVector, endPose->rightVector,rollCross);
    units::scalar rollSin = vectorDotProduct(rollCross, startPose->fwdVector);
    result.roll = std::atan2(rollSin, rollCos);
    

    units::scalar pitchCos = vectorDotProduct(startPose->dirVector, endPose->dirVector);
    std::array<units::scalar,3> pitchCross;
    vectorCrossProduct( startPose->dirVector,endPose->dirVector,pitchCross);
    units::scalar pitchSin = vectorDotProduct(pitchCross, startPose->rightVector);
    result.pitch = std::atan2(pitchSin, pitchCos);
    
    units::scalar yawCos = vectorDotProduct(startPose->fwdVector, endPose->fwdVector);
    std::array<units::scalar,3> yawCross;
    vectorCrossProduct(startPose->fwdVector,endPose->fwdVector,yawCross);
    units::scalar yawSin = vectorDotProduct(yawCross, startPose->dirVector);
    result.yaw = std::atan2(yawSin, yawCos);
    
    return result;
}

inline rotationRate getRotationRate(units::scalar deltaTime){
    if(deltaTime <= 0 || !startPose  || !endPose){
        rotationRate rate{0,0,0};
        return rate;
    }
    rotations poseDifference = getDifference();
    rotationRate rate;
    rate.pitchRate = poseDifference.pitch / deltaTime;
    rate.rollRate = poseDifference.roll / deltaTime;
    rate.yawRate = poseDifference.yaw / deltaTime;

    return rate;
}

};

}

#endif