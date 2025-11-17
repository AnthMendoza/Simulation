#include <gtest/gtest.h>
#include "../include/sim/droneSimulation.h"
#include "../include/sim/droneSimulation.h"
#include "../include/dynamics/vehicle.h"
#include "../include/dynamics/drone.h"
#include "../include/control/dronePIDControl.h"
#include "../include/dynamics/drone.h"
#include "../include/subsystems/battery.h"
#include "../include/subsystems/droneDependencyInjector.h"
#include "../include/subsystems/propeller.h"
#include "../include/subsystems/motor.h"
#include "../include/control/stateEstimation.h"
#include "../include/control/mochStateEstimation.h"
#include "../include/utility/graphing.h"

using namespace SimCore;

#define CONTROLLER_POLLING_RATE 0.002
#define ESTIMATOR_POLLING_RATE 0.002


class droneSystemTest : public ::testing::Test {
protected:
    std::string configDrone = "../tests/testConfigs/drone_test_config.toml";    
    std::string configMotor = "../tests/testConfigs/motor_test_config.toml"; 
    std::string configBattery = "../tests/testConfigs/battery_test_config.toml"; 
    std::string configPropeller = "../tests/testConfigs/propeller_test_config.toml"; 
    

    std::unique_ptr<droneControllerBase> controller;
    std::unique_ptr<stateEstimationBase> estimator;
    std::unique_ptr<droneBody> drone;



    void testRotation(poseState& desiredPose ,float testDuration){
        float time = 0;
        float timeStep = 0.001;

        PIDDroneController* PIDControllerPtr = dynamic_cast<PIDDroneController*>(controller.get());
        PIDControllerPtr->setAPIDVXGains({0.1,1.0,0.0});
        PIDControllerPtr->setAPIDVYGains({0.1,1.0,0.0});
        PIDControllerPtr->setAPIDXGains( {400.0,0,0});
        PIDControllerPtr->setAPIDYGains( {400.0,0,0});
        while(time < testDuration){
            time += timeStep;
            drone->setPositionVector(0,0,10);
            drone->setVelocity(0,0,0);
            auto sensorPacket = drone->sensors->getSensorData();
            estimator->updateEstimation(time,sensorPacket);
            auto controlPacket = PIDControllerPtr->updateAOTHold(time,estimator->getStateInfo(),desiredPose);
            drone->updateState(time,controlPacket);
            auto pose = drone->getPose();

            float error = vectorAngleBetween(pose.dirVector , desiredPose.dirVector);
        }
        //PIDControllerPtr->generateGraph();

    }


    void SetUp() override {
        configMotor = readFileAsString(configMotor);    
        configBattery = readFileAsString(configBattery);   
        configPropeller = readFileAsString(configPropeller);
        configDrone = readFileAsString(configDrone);
        controller = std::make_unique<SimCore::PIDDroneController>(static_cast<float> (CONTROLLER_POLLING_RATE));
        controller->initController(configDrone);
        estimator = std::make_unique<SimCore::mochEstimation>(static_cast<float> (ESTIMATOR_POLLING_RATE));
        auto bat = std::make_unique<SimCore::battery>(configBattery);
        drone = std::make_unique<SimCore::droneBody>(std::move(bat));
        drone->initDrone(configDrone);
        SimCore::propeller prop(configPropeller);
        SimCore::motor mot(configMotor);

        auto motorProp = SimCore::setSquare(0.3f,0.3f,prop,mot);

        drone->offsetCOG({0,0,0});
        controller->initAllocatorWithProps(motorProp.second);
        drone->addMotorsAndProps(motorProp);
        controller->setTargetPosition(0,0,30);


    }

    void TearDown() override {

    }   
};



TEST_F(droneSystemTest , validateXRotattion_Postive){
    quaternionVehicle vehiclePose;

    float angle = 0.4;
    vehiclePose.eulerRotation(angle,0.0f,0.0f);

    auto desiredPose = vehiclePose.getPose();
    float testDuration = 0.4f;
    testRotation(desiredPose,testDuration);
    print(desiredPose.dirVector,"desried Dir");

    print(drone->getPose().dirVector,"dirVector Drone");

}

TEST_F(droneSystemTest , validateXRotattion_Negative){
    quaternionVehicle vehiclePose;

    float angle = -0.4;
    vehiclePose.eulerRotation(angle,0.0f,0.0f);

    auto desiredPose = vehiclePose.getPose();
    float testDuration = 0.4f;
    testRotation(desiredPose,testDuration);
    print(desiredPose.dirVector,"desried Dir");

    print(drone->getPose().dirVector,"dirVector Drone");

}

TEST_F(droneSystemTest , validateYRotattion_Positive){
    quaternionVehicle vehiclePose;

    float angle = 0.2;
    vehiclePose.eulerRotation(0.0f,angle,0.0f);

    auto desiredPose = vehiclePose.getPose();
    float testDuration = 1.0f;
    testRotation(desiredPose,testDuration);
    print(desiredPose.dirVector,"desried Dir");

    print(drone->getPose().dirVector,"dirVector Drone");

}

TEST_F(droneSystemTest , validateYRotattion_Negative){
    quaternionVehicle vehiclePose;

    float angle = -0.2;
    vehiclePose.eulerRotation(0.0f,angle,0.0f);

    auto desiredPose = vehiclePose.getPose();
    float testDuration = 1.0f;
    testRotation(desiredPose,testDuration);
    print(desiredPose.dirVector,"desried Dir");

    print(drone->getPose().dirVector,"dirVector Drone");

}

