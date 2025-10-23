#ifndef POSEROTATION_H
#define POSEROTATION_H
#include <optional>
#include "quaternion.h"
#include "vectorMath.h"

namespace SimCore{
//rad
struct rotations{
    float roll;
    float pitch;
    float yaw;
};
//rad/s
struct rotationRate{
    float rollRate; 
    float pitchRate;
    float yawRate;
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
}
inline void setEndPose(const poseState& pose){
    endPose = pose;
}


rotations getDifference() {
    if(!startPose || !endPose){
        rotations emptyRotations{0,0,0};
        return emptyRotations;
    }

    rotations result;

    float rollCos = vectorDotProduct(startPose->rightVector, endPose->rightVector);
    std::array<float,3> rollCross;
    vectorCrossProduct(endPose->rightVector, startPose->rightVector,rollCross);
    float rollSin = vectorDotProduct(rollCross, startPose->fwdVector);
    result.roll = -std::atan2(rollSin, rollCos);
    

    float pitchCos = vectorDotProduct(startPose->dirVector, endPose->dirVector);
    std::array<float,3> pitchCross;
    vectorCrossProduct(endPose->dirVector, startPose->dirVector,pitchCross);
    float pitchSin = vectorDotProduct(pitchCross, startPose->rightVector);
    result.pitch = std::atan2(pitchSin, pitchCos);
    
    float yawCos = vectorDotProduct(startPose->fwdVector, endPose->fwdVector);
    std::array<float,3> yawCross;
    vectorCrossProduct(endPose->fwdVector, startPose->fwdVector,yawCross);
    float yawSin = vectorDotProduct(yawCross, startPose->dirVector);
    result.yaw = std::atan2(yawSin, yawCos);
    
    return result;
}

inline rotationRate getRotationRate(float deltaTime){
    if(deltaTime <= 0 ){
        rotationRate rate{0,0,0};
        return rate;
    }
    rotations posDifference = getDifference();
    rotationRate rate;
    rate.pitchRate = posDifference.pitch / deltaTime;
    rate.rollRate = -posDifference.roll / deltaTime;
    rate.yawRate = posDifference.yaw / deltaTime;
    return rate;
}

};

}

#endif