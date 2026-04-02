#pragma once

#include "../sim/droneSimulation.h"
#include "PIDTypes.h"
#include "../control/mochStateEstimation.h"
#include <util/util.h>
#include "PIDTypes.h"

using namespace SimCore;

template<class PIDType>
class PIDSimulation : public droneSimulation{
    std::unique_ptr<timeManager> manager;


public:
    PIDFunctionGroup<PIDType> functions;

    PIDSimulation() = delete;
    PIDSimulation(droneAssets assets):droneSimulation(      std::make_unique<droneBody>(assets.drone),
                                                            std::make_unique<SimCore::droneControllerBase>(assets.controller),
                                                            std::make_unique<SimCore::stateEstimationBase>(assets.estimator)){
    manager = std::make_unique<timeManager>(config.logPostInterval);
    }

    

    droneControllerBase* getController() const {
        return controller.get();
    }

    stateEstimationBase* getEstimator() const {
        return estimator.get();
    }

    Vehicle* getVehicle() const {
        return vehicle.get();
    }    


} ;