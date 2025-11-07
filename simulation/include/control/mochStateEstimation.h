#pragma once
#include "stateEstimationBase.h"
#include "stateInfo.h"

namespace SimCore{
class mochEstimation : public stateEstimationBase{
    

    protected:


    public:
    mochEstimation(float interval):
    stateEstimationBase(interval)
    {

    }
    void updateEstimation(float time, simpleSensorPacket packet) override{
        statePacket = testGlobals::actualVehicleState::state;
    }
    ~mochEstimation() = default;

    virtual std::unique_ptr<stateEstimationBase> clone() const override{
        return std::make_unique<mochEstimation>(*this);
    }
    
    void controllerData(controllerToEstimationVariant controller) override{

    }  


};

}