#pragma once
#include "../sim/droneSimulation.h"
#include "PIDTypes.h"
#include "../utility/graphing.h"
#include "pythonConnector.h"
#include "../subsystems/motor.h"
#include "../subsystems/propeller.h"
#include "../subsystems/battery.h"
#include "../utility/utility.h"
#include <cmath>



using namespace SimCore;


namespace parameterEstimation{

class motorEstimation: public simulation{
private:
using simulation::run;
using simulation::stop;
using simulation::wait;

std::unique_ptr<motor> staticMotorCopy;
std::unique_ptr<battery> staticBatteryCopy;
std::unique_ptr<propeller> staticPropellerCopy;

std::unique_ptr<motor> motor_;
std::unique_ptr<battery> battery_;
std::unique_ptr<propeller> propeller_;

std::unique_ptr<utility::grapher> bestResultGraph;

std::unique_ptr<utility::grapher> graph;

float timeStep = 0.001;
float time = 0;
float testDuration = 0;
float cost = 0;
float bestCost = 0;

float airDensity = 1.225; //at sea level kg/m^3

float requestedAngularVelocity = 200;

    void step(float time) override {
    }

    void startUp() override {
    }

    void logs() override {
    }

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
            if(cost < lowestCost ){
                lowestCost = cost;
                bestPID = testPID;
                bestCost = lowestCost;
                bestResultGraph = std::move(graph);
            }
            std::cout << "Cost: " << cost << "\n"; 
            costToPython(testPID , static_cast<double>(cost));
            displayPID(testPID);  
            testPID = getNextPID<PIDType>();
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

    //return cost
    void simulationTest(dataPID pair){

        motor_->setPIDGains(pair);

        graph.reset();
        graph = std::make_unique<utility::grapher>();


        while(time < testDuration){
            time += timeStep;

            float dragTorque = propeller_->dragTorque(airDensity,motor_->getCurrentAngularVelocity());

            motor_->updateMotorAngularVelocity(timeStep,dragTorque,*battery_.get(),requestedAngularVelocity);
            
            battery_->updateBattery(motor_->getCurrentCurrent(),time);

            graph->dataEntry(time,motor_->getCurrentAngularVelocity(),motor_->getAppliedVoltage(),dragTorque);
            
            costFunction(std::fabs(requestedAngularVelocity - motor_->getCurrentAngularVelocity()));

        }
        graph->plot();
        time = 0;
    }

    void reset(){
        motor_ = std::make_unique<motor>(*staticMotorCopy);
        battery_ = std::make_unique<battery>(*staticBatteryCopy);
        propeller_ = std::make_unique<propeller>(*staticPropellerCopy);
    }
    

public:
    
    motorEstimation(const std::string& motorConfigPath , const std::string& batteryConfigPath,const std::string& propellerConfigPath): simulation(){

        std::string configMotor = utility::readFileAsString(motorConfigPath);
        staticMotorCopy = std::make_unique<motor>(configMotor);
        motor_ = std::make_unique<motor>(*staticMotorCopy);

        std::string configBattery = utility::readFileAsString(batteryConfigPath);
        staticBatteryCopy = std::make_unique<battery>(configBattery);
        battery_ = std::make_unique<battery>(*staticBatteryCopy);

        std::string configPropeller = utility::readFileAsString(propellerConfigPath);
        staticPropellerCopy = std::make_unique<propeller>(configPropeller);
        propeller_ = std::make_unique<propeller>(*staticPropellerCopy);
    }

    void test(){

        testDuration = 4.0;
        auto result = optimizeHandler<dataPID>(100);
        std::cout<< "Best PID : \n";

    }


};



}