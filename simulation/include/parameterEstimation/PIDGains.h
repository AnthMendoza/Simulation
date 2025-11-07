#ifndef PIDGAINS
#define PIDGAINS
#include <tuple>
#include <limits>
#include <stdexcept>
#include "../dynamics/drone.h"
#include "pythonConnector.h"
#include "../utility/utility.h"
#include "../core/vectorMath.h"
#include "droneCostFunction.h"
#include "PIDTestScenarios.h"
#include "PIDTypes.h"
#include "../sim/droneSimulation.h"


namespace SimCore{



/// @brief @class simTemplate is the framework for running an optimzation test. Guided by dedicatied setUp and constValues.
/// @param basisDrone Preconfigured Drone for testing.
/// @param basisController Preconfigured controller for testing.
/// @param PID tuple of floats with values Kp,ki,kd for testing.
/// @param constValues Called on every iteration allowing const values to be set for isotlated testing.
/// @param setUp Called once prior to the simulation run. Use this function to put the drone in the desired state.
/// @return cost

template<typename PIDType>
float simTemplate( droneAssets asset , PIDType PID, PIDFunctionGroup<PIDType>& func){
    calibratePID calibrate(func.stdThreshold,func.errorThreshold);
    calibrate.setConstants(10.0f,10.0f);

    for(){

    }
    

    return cost;
    
}

template<typename PIDType>
PIDType optimize( droneAssets asset , int numberOfRuns,PIDFunctionGroup<PIDType>& func ){
    if(numberOfRuns <= 0) throw std::runtime_error("numberOfRuns in optimize cannot be <= 0\n");
    initPython();
    float lowestCost = std::numeric_limits<float>::max();
    PIDType bestPID;
    PIDType testPID = getNextPID<PIDType>();

    for(int i = 0 ; i < numberOfRuns ; i++){
        progressBar(static_cast<float>(i)/static_cast<float>(numberOfRuns));
        float cost = simTemplate(basisDrone,basisController,testPID,func);
        if(cost < lowestCost){
            lowestCost = cost;
            bestPID = testPID;
        }
        costToPython(testPID , static_cast<double>(cost));
        displayPID(testPID);
        std::cout << "Cost: " << cost << std::endl;   
        testPID = getNextPID<PIDType>();
    }
    resetOptimizer();
    resetOptimizerDuel();
    return bestPID;
}



}//SimCore

#endif //PIDGAINS