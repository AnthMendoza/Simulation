#ifndef PIDFUNCTIONGROUP_H
#define PIDFUNCTIONGROUP_H

#include <tuple>
#include "../dynamics/drone.h"
#include "../control/dronePIDControl.h"
#include "../control/stateEstimationBase.h"

namespace SimCore{

//Kp,Ki,Kd respectively
using dataPID = std::tuple<float,float,float>;
using PIDPair = std::pair<dataPID,dataPID>;

class droneBody;
class droneControl;
class calibratePID;

struct droneAssets{
    droneBody* drone;
    PIDDroneController* controller;
    stateEstimationBase* estimator;
};

template<typename PIDType>
struct PIDFunctionGroup {
    bool (*iteration)(droneAssets);
    void (*setUp)(droneAssets, PIDType);
    void (*setGains)(droneAssets, PIDType);
    float stdThreshold;
    float errorThreshold;
    float maxDuration;
};

void printPIDPair(const PIDPair& pidPair) {
    auto printTuple = [](const dataPID& pid) {
        std::cout << "(" 
                  << std::get<0>(pid) << ", " 
                  << std::get<1>(pid) << ", " 
                  << std::get<2>(pid) << ")";
    };

    std::cout << "PID 1: ";
    printTuple(pidPair.first);
    std::cout << " | PID 2: ";
    printTuple(pidPair.second);
    std::cout << std::endl;
}

}

#endif