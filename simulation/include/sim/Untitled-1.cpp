
#include "../../include/dynamics/drone.h"
#include "../../include/dynamics/vehicle.h"
#include "../../include/subsystems/droneDependencyInjector.h"
using namespace SimCore;

#define FREQUENCY 0.01
int main(int argc, char* argv[]){
    if (argc < 5) {
        std::cout << "Usage: " << argv[0] << " <motor_config> <battery_config> <propeller_config> <drone_config>\n";
        return 1;
    }
    std::string configMotor = readFileAsString(argv[1]);    
    std::string configBattery = readFileAsString(argv[2]);   
    std::string configPropeller = readFileAsString(argv[3]);
    std::string configDrone = readFileAsString(argv[4]);

    auto controller = std::make_unique<PIDDroneController>(static_cast<float> (FREQUENCY));
    auto bat = std::make_unique<battery>(configBattery);
    auto drone = std::make_unique<droneBody>(
        std::move(bat),
        std::move(controller));
    
    drone->initDrone(configDrone);

    drone->controller->initController(configDrone,*drone);

    drone->offsetCOG({0,0,0});

    propeller prop(configPropeller);
    motor mot(configMotor,drone->getTimeStep());

    auto motorPropPair = setSquare(0.3f,0.3f,prop,mot);
    drone->addMotorsAndProps(motorPropPair);
}