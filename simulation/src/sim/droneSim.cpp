#include "../../include/sim/droneSimulation.h"
#include "../../include/dynamics/vehicle.h"
#include "../../include/dynamics/drone.h"
#include "../../include/control/dronePIDControl.h"
#include "../../include/dynamics/drone.h"
#include "../../include/subsystems/battery.h"
#include "../../include/subsystems/droneDependencyInjector.h"
#include "../../include/subsystems/propeller.h"
#include "../../include/subsystems/motor.h"
#include "../../include/control/stateEstimation.h"
#include "../../include/control/mochStateEstimation.h"
#include "../../include/subsystems/cameraBase.h"
#include "../../include/core/quaternion.h"

#define CONTROLLER_POLLING_RATE 0.004
#define ESTIMATOR_POLLING_RATE 0.002

SimCore::cameraBase createCamera(){

    SimCore::quaternionVehicle vQuant;

    vQuant.eulerRotation(M_PI,0,0);
    
    std::array<float,3> position{0,0,0};

    SimCore::cameraBase camera(vQuant.getPose(),position);

    return camera;
}


int main(int argc, char* argv[]){
    if (argc < 5) {
        std::cout << "Usage: " << argv[0] << " <motor_config> <battery_config> <propeller_config> <drone_config>\n";
        return 1; 
    }
    std::string configMotor = readFileAsString(argv[1]);    
    std::string configBattery = readFileAsString(argv[2]);   
    std::string configPropeller = readFileAsString(argv[3]);
    std::string configDrone = readFileAsString(argv[4]);
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
    controller->setTargetPosition(0,0,20);
    SimCore::cameraBase camera = createCamera();
    drone->setCamera(&camera);

    SimCore::droneSimulation sim(std::move(drone),std::move(controller),std::move(estimator));
    
    sim.configure([](SimCore::simulation::configStruct& cfg){
        cfg.mode = SimCore::simulation::SimMode::RealTime;
        cfg.maxSimTime = 0.0f;
        cfg.logging = true;
    });

    sim.modifyStep([](float t, SimCore::Vehicle& v, SimCore::stateEstimationBase& e, SimCore::droneControllerBase& c){


    });



    sim.run();
    while(true){}


    return 0;
}