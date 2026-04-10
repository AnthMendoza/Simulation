#pragma once
#include <array>
#include "stateEstimationBase.h"
#include "../subsystems/droneSensorSuite.h"
#include <base/base.h>

namespace SimCore{

class stateEstimation : public stateEstimationBase{

    protected:
    units::vec3 gravityVector;
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

    virtual std::unique_ptr<stateEstimationBase> clone() const override{
        return std::make_unique<stateEstimation>(*this);
    }

    //Update Estimated positions based on the most recent velocity and acceleration.
    //Update will run at simulation clock speed unlike sensors which are tied to hardware spec sample rates.

    void updateEstimation(float time, simpleSensorPacket packet) override;

    units::vec3 gravityVectorEstimation(units::vec3 accel , float time);

    void calculateEstimatedRotationRate();

    void controllerData(controllerToEstimationVariant controller) override{
        control = controller;
    }  


};

}