#pragma once
#include <array>
#include "../core/vectorMath.h"
#include "stateEstimationBase.h"
#include "../core/quaternion.h"
#include "../subsystems/droneSensorSuite.h"
#include "../core/filtering.h"
namespace SimCore{

class stateEstimation : public stateEstimationBase{

    protected:
    threeDState gravityVector;
    controllerToEstimationVariant control;
    float maxTimeGap; //seconds
    float timeConstant;
    filtering::lowPassFilterVector3<float> gravityVectorFilter; //time seconds
    //relative positoin calculated by the most recent GPS position
    bool firstGPSSample = true;
    std::optional<float> lastGravityVectorFilter;
    protected:
    void predictionState(float time, simpleSensorPacket& packet);
    public:
    stateEstimation(float timeBetweenUpdates);
    ~stateEstimation() = default;
    stateEstimation(const stateEstimation& other) = default;
    //Update Estimated positions based on the most recent velocity and acceleration.
    //Update will run at simulation clock speed unlike sensors which are tied to hardware spec sample rates.

    void updateEstimation(float time, simpleSensorPacket packet) override;

    threeDState gravityVectorEstimation(threeDState accel , float time);

    void calculateEstimatedRotationRate();

    void controllerData(controllerToEstimationVariant controller) override{
        control = controller;
    }  


};

}