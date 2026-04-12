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

using namespace SimCore;

#define CONTROLLER_POLLING_RATE 0.004
#define ESTIMATOR_POLLING_RATE 0.002


class droneSystemTest : public ::testing::Test {
protected:
    std::string configDrone = "../tests/testConfigs/drone_test_config.toml";    
    std::string configMotor = "../tests/testConfigs/motor_test_config.toml"; 
    std::string configBattery = "../tests/testConfigs/battery_test_config.toml"; 
    std::string configPropeller = "../tests/testConfigs/propeller_test_config.toml"; 
    
    std::unique_ptr<droneSimulation> sim;


    void SetUp() override {
        configMotor = utility::readFileAsString(configMotor);    
        configBattery = utility::readFileAsString(configBattery);   
        configPropeller = utility::readFileAsString(configPropeller);
        configDrone = utility::readFileAsString(configDrone);
        auto controller = std::make_unique<SimCore::PIDDroneController>(static_cast<float> (CONTROLLER_POLLING_RATE));
        controller->initController(configDrone);
        auto estimator = std::make_unique<SimCore::mochEstimation>(static_cast<float> (ESTIMATOR_POLLING_RATE));
        auto bat = std::make_unique<SimCore::battery>(configBattery);
        auto drone = std::make_unique<SimCore::droneBody>(std::move(bat));
        drone->initDrone(configDrone);
        SimCore::propeller prop(configPropeller);
        SimCore::motor mot(configMotor);

        auto motorProp = SimCore::setSquare(0.3f,0.3f,prop,mot);

        drone->offsetCOG({0,0,0});
        controller->initAllocatorWithProps(motorProp.second);
        drone->addMotorsAndProps(motorProp);
        controller->setTargetPosition(0,0,30);

        sim = make_unique<droneSimulation>(std::move(drone),std::move(controller),std::move(estimator));
    }

    void TearDown() override {
        sim.reset();
    }   
};



TEST_F(droneSystemTest , validateXRotattion_Postive){
    sim->configure([](SimCore::simulation::configStruct& cfg){
        cfg.realTime = false;
        cfg.logging = false;
        //5 timesteps until halt
        cfg.maxSimTime = cfg.timeStep * 400;
    });

    auto controllerPtr = sim->getController();
    SimCore::PIDDroneController* controllerPIDPtr = dynamic_cast<SimCore::PIDDroneController*>(controllerPtr);

    controllerPIDPtr->setTargetPosition(0,10,20);

    

    auto vehiclePtr = sim->getDrone();
    droneBody *dronePtr = dynamic_cast<SimCore::droneBody*>(vehiclePtr);

    auto pose = dronePtr->getPose();
    auto dirVector = pose.dirVector;
    pose.printPose();
    EXPECT_FLOAT_EQ(dirVector[0],0.0f);


}


