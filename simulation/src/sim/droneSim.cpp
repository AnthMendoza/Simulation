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

#define CONTROLLER_POLLING_RATE 0.004
#define ESTIMATOR_POLLING_RATE 0.002

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
    controller->setTargetPosition(50,50,50);


    //SimCore::quaternionVehicle state;
    //state.eularRotation(0,0,M_PI_2);
    //auto pose = state.getPose();
    //drone->setStateVector(pose.dirVector,pose.fwdVector);
    
    SimCore::droneSimulation sim(std::move(drone),std::move(controller),std::move(estimator));
    
    sim.configure([](SimCore::simulation::configStruct& cfg){
        cfg.realTime = true;
        cfg.maxSimTime = 0.0f;
        cfg.logging = true;
    });



    sim.run();
    while(true){}


    return 0;
}