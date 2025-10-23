#include "../../include/control/stateEstimation.h"
#include "../../include/control/stateInfo.h"
#include "../../include/control/stateEstimationBase.h"
#include "../../include/core/quaternion.h"

namespace SimCore{

stateEstimation::stateEstimation(float timeBetweenUpdates)
    :stateEstimationBase(timeBetweenUpdates),
    maxTimeGap(2.0f),
    timeConstant(0.5f),  
    gravityVectorFilter(timeConstant, maxTimeGap)
    {

}


void stateEstimation::predictionState(float time ,simpleSensorPacket& packet) {
    float dt = time - statePacket.timestamp;
    if (dt <= 0.0f) return; 
    stateInfo predictionOnly;
    threeDState accel = packet.accelerometer.data;
    
    for(int i = 0; i < 3; ++i) {
        predictionOnly.position[i] += statePacket.velocity[i] * dt + 0.5f * accel[i] * dt * dt;
        predictionOnly.velocity[i] += accel[i] * dt;
    }
    
    

}

//call at main clock speed
void stateEstimation::updateEstimation(float time, simpleSensorPacket packet){
    if(!manager->shouldTrigger(time)){
        return;
    }
    auto gyro = packet.gyro.rotationRate;
    auto accel = packet.accelerometer.data;
    auto gpsRelative = packet.gps.relativePosition;
    gravityVectorEstimation(accel,time);
    predictionState(time , packet);
}

void stateEstimation::calculateEstimatedRotationRate(){
    poseState startPose;
    poseState endPose;
    
}

//if resultant Acceleration is less than threashold the value is discarded. Confidence is low that the resultant is gravity
static constexpr float gravitationalAcceleration = -9.8f;
static constexpr float threshold = 0.4f;

threeDState stateEstimation::gravityVectorEstimation(threeDState accel,float time){

    auto controllerPacket = std::get<onlyThrustAccel>(control);
    threeDState thrustAccel = controllerPacket.thrustAccelerationVector;
    threeDState resultantAcceleration = subtractVectors(accel,thrustAccel);
    if(vectorMag(resultantAcceleration) > std::fabs(gravitationalAcceleration) * threshold){
        if(!lastGravityVectorFilter){
            resultantAcceleration = gravityVectorFilter.filter(resultantAcceleration,0.0f);
        }else{
            float dt = time - *lastGravityVectorFilter;
            resultantAcceleration = gravityVectorFilter.filter(resultantAcceleration,dt);
        }
        lastGravityVectorFilter = time;

    }
    return resultantAcceleration;
}

}