#ifndef PIDGAINS
#define PIDGAINS
#include "../dynamics/drone.h"
#include <tuple>
#include <unordered_map>
#include "PIDGainsConnector.h"
#include "../sim/utility.h"

namespace SimCore{


/**
 * @class calibratePID
 * @brief Utility class for evaluating and tuning PID controller performance.
 *
 * This class provides a generalized framework for calculating the cost of a given set of PID gains
 * based on simulation results.
 *
 * Usage:
 *  - Call reset() before each simulation run.
 *  - During each timestep, call logStep(time, output, reference).
 *  - After the run, call evaluate() to get the total cost.
 *
 * Source paper: https://skoge.folk.ntnu.no/prost/proceedings/PID-12/papers/0030.pdf?utm_source=chatgpt.c
 * 
 * Cost function highlighted in the paper is slightly simplified due to open loop requests. Open loop
 * implies that the tested parameter has feedforward control.
 */

class calibratePID {
private:

    float alpha = 0.0f;
    float beta = 0.0f;
    float costFunctionSum = 0.0f;
    float tolerance = 0.02f;
    float currentTime = 0;
    std::vector<float> timeLog;
    std::vector<float> outputLog;
    std::vector<float> referenceLog;

    void costFunction(float error ,float dt) {
        costFunctionSum += (error * error) * dt;
    }

    float computeOvershoot(const std::vector<float>& output, float reference) {
        float maxValue = *std::max_element(output.begin(), output.end());
        return std::max(0.0f, maxValue - reference);
    }

    float computeSettlingTime(const std::vector<float>& output, float reference, const std::vector<float>& time) {
        float threshold = std::abs(reference * tolerance);
        for (int i = output.size() - 1; i >= 0; --i) {
            if (std::abs(output[i] - reference) > threshold)
                return time[i];
        }
        return 0.0f;
    }

public:
    calibratePID(){

    }

    void reset() {
        costFunctionSum = 0.0f;
        currentTime = 0;
        timeLog.clear();
        outputLog.clear();
        referenceLog.clear();
    }

    void logStep(float output, float reference , float dt) {
        currentTime += dt;
        float error = reference - output;
        costFunction(error , dt);
        timeLog.push_back(currentTime);
        outputLog.push_back(output);
        referenceLog.push_back(reference);
    }

    float evaluate() {
        float finalReference = referenceLog.empty() ? 0.0f : referenceLog.back();
        float overshoot = computeOvershoot(outputLog, finalReference);
        float settlingTime = computeSettlingTime(outputLog, finalReference, timeLog);
        costFunctionSum = costFunctionSum + (alpha * overshoot + beta * settlingTime);
        return costFunctionSum;
    }
    
    inline void setConstants(float _alpha, float _beta){
        alpha = _alpha;
        beta = _beta;
    }
};



//Kp,Ki,Kd respectively
using dataPID = std::tuple<float,float,float>;



/// @brief @class simTemplate is the framework for running an optimzation test. Guided by dedicatied setUp and constValues.
/// @param basisDrone Preconfigured Drone for testing.
/// @param basisController Preconfigured controller for testing.
/// @param PID tuple of floats with values Kp,ki,kd for testing.
/// @param constValues Called on every iteration allowing const values to be set for isotlated testing.
/// @param setUp Called once prior to the simulation run. Use this function to put the drone in the desired state.
/// @return 

float simTemplate(  droneBody* basisDrone, droneControl* basisController, dataPID PID,
                    void (*iteration)(droneBody*,calibratePID*), 
                    void (*setUp)(droneBody*,droneControl*, dataPID&)){
    calibratePID calibrate;
    calibrate.setConstants(1.0f,1.0f);
    float duration = 10;
    droneControl* testController = new droneControl(*basisController);
    droneBody* testDrone = new droneBody(*basisDrone);
    testController->setPIDZGains(PID);
    setUp(testDrone,testController,PID);
    //emplacing testController into testDrone
    testDrone->setController(testController);
    for(int i = 0 ; i < duration/testDrone->getTimeStep() ; i++){
        testDrone->updateState();
        iteration(testDrone, &calibrate);

    }
    float cost = calibrate.evaluate();
    return cost;
    
}


dataPID optimize(   droneBody* basisDrone, droneControl* basisController, int numberOfRuns,
                    void (*iteration)(droneBody*,calibratePID*), 
                    void (*setUp)(droneBody*,droneControl*, dataPID& )){
    if(numberOfRuns <= 0) throw runtime_error("numberOfRuns in optimize cannot be <= 0\n");
    initPython();
    float lowestCost = std::numeric_limits<float>::max();
    dataPID bestPID;
    dataPID testPID = getNextPID();
    for(int i = 0 ; i < numberOfRuns ; i++){
        progressBar(static_cast<float>(i)/static_cast<float>(numberOfRuns));
        float cost = simTemplate(basisDrone,basisController,testPID,iteration,setUp);
        if(cost < lowestCost){
            lowestCost = cost;
            bestPID = testPID;
        }
        costToPython(testPID , static_cast<double>(cost));
        testPID = getNextPID();
            std::cout << "PID Values: "
              << std::get<0>(testPID) << ", "
              << std::get<1>(testPID) << ", "
              << std::get<2>(testPID) << std::endl;

        std::cout << "Cost: " << cost << std::endl;   
    }
    return bestPID;
}



void hoverTestSetup(droneBody* testDrone,droneControl* testController, dataPID& PID){
    //set drone vertically 
    testDrone->resetMotors();
    testDrone->setPositionVector(0.0f,0.0f,25.0f);
    testDrone->setStateVector(0.0f,0.0f,1.0f);
    testDrone->setVelocity(0.0f,0.0f,0.0f);
    testDrone->setMotorHover();

    testController->setpidControl(0.0f,0.0f,4.0f);
}


void hoverTestIterator(droneBody* testDrone,calibratePID* calibrate){
    testDrone->setStateVector(0.0f,0.0f,1.0f);
    calibrate->logStep(testDrone->getEstimatedPosition()[2] , 4.0f ,testDrone->getTimeStep());
}




}//SimCore

#endif //PIDGAINS