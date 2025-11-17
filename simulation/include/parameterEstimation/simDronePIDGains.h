#pragma once
#include "../sim/droneSimulation.h"
#include "PIDTypes.h"
#include "../utility/graphing.h"
#include "pythonConnector.h"


using namespace SimCore;


namespace parameterEstimation{

class AOTPIDEstimation : public droneSimulation{
private:
using droneSimulation::run;
using droneSimulation::stop;
using droneSimulation::wait;

std::unique_ptr<droneControllerBase> controllerStaticCopy;
std::unique_ptr<stateEstimationBase> estimatorStaticCopy;
std::unique_ptr<Vehicle> vehicleStaticCopy;

std::unique_ptr<utility::grapher> bestResultGraph;

std::unique_ptr<utility::grapher> graph;

float timeStep = 0.001;
float time = 0;
float testDuration = 0;
float cost = 0;
float bestCost = 0;
poseState desiredPose;
threeDState bestFinalPosition = {0,0,1};
protected:
    template<typename PIDType>
    PIDType optimizeHandler(int numberOfRuns){
        if(numberOfRuns <= 0) throw std::runtime_error("numberOfRuns in optimize cannot be <= 0\n");
        initPython();
        float lowestCost = std::numeric_limits<float>::max();
        PIDType bestPID;
        PIDType testPID = getNextPID<PIDType>();
    
        for(int i = 0 ; i < numberOfRuns ; i++){
            progressBar(static_cast<float>(i)/static_cast<float>(numberOfRuns));
            reset();
            simulationTest(testPID);
            if(cost < lowestCost && nearResult(desiredPose)){
                lowestCost = cost;
                bestPID = testPID;
                poseState bestPose= vehicle->getPose();
                threeDState directionVector = bestPose.dirVector;
                bestFinalPosition = directionVector;
                bestCost = lowestCost;
                bestResultGraph = std::move(graph);
            }
            std::cout << "Cost: " << cost << "\n"; 
            costToPython(testPID , static_cast<double>(cost));
            displayPID(testPID);  
            testPID = getNextPID<PIDType>();
            printPIDPair(testPID);
            cost = 0;
            reset();
        }
        resetOptimizer();
        resetOptimizerDuel();
        return bestPID;
    }

    void costFunction(float error){
        cost += error;
    }

    bool nearResult(poseState desired){
        float angle = vectorAngleBetween(vehicle->getPose().dirVector,desired.dirVector);
        float EPSILON = 0.1; //rad
        if(std::abs(angle) > EPSILON){
            return false;
        }

        return true;
    }

    //return cost
    void simulationTest(PIDPair pair){
        quaternionVehicle vehiclePose;

        float angle = 0.4;
        vehiclePose.eulerRotation(angle,0.0f,0.0f);

        desiredPose = vehiclePose.getPose();
        
        PIDDroneController* PIDControllerObj = dynamic_cast<PIDDroneController*>(controller.get());

        PIDControllerObj->setAPIDXGains(pair.first);
        PIDControllerObj->setAPIDYGains(pair.first);
        PIDControllerObj->setAPIDVXGains(pair.second);
        PIDControllerObj->setAPIDVYGains(pair.second);

        graph.reset();
        graph = std::make_unique<utility::grapher>();


        while(time < testDuration){
            time += timeStep;
            vehicle->setPositionVector(0,0,10);
            vehicle->setVelocity(0,0,0);
            auto sensorPacket = vehicle->sensors->getSensorData();
            estimator->updateEstimation(time,sensorPacket);
            auto controlPacket = PIDControllerObj->updateAOTHold(time,estimator->getStateInfo(),desiredPose);
            vehicle->updateState(time,controlPacket);
            auto pose = vehicle->getPose();

            auto loggedData = PIDControllerObj->getLogs();

            graph->dataEntry(time,loggedData.pitchDesiredVelo,loggedData.pitchVelo,loggedData.rollDesiredVelo,loggedData.rollVelo);

            float error = vectorAngleBetween(pose.dirVector , desiredPose.dirVector);
            costFunction(error);
        }
        time = 0;
        print(desiredPose.dirVector,"desired");
        print(vehicle->getPose().dirVector,"endState");
    }

    void reset(){
        controller = controllerStaticCopy->clone();
        estimator = estimatorStaticCopy->clone();
        vehicle = vehicleStaticCopy->clone();
    }
    

public:
    using droneSimulation::droneSimulation;

    void test(){
        controllerStaticCopy = controller->clone();
        estimatorStaticCopy = estimator->clone();
        vehicleStaticCopy = vehicle->clone();

        testDuration = 4.0;
        auto result = optimizeHandler<PIDPair>(500);
        std::cout<< "Best PIDs : \n";
        printPIDPair(result);
        print(bestFinalPosition,"Best Final Vector");
        print(bestCost,"lowestCost");
        bestResultGraph->plot();
    }


};



}