#include "../../include/sim/MySim.h"
#include "../../include/control/PIDGains.h"
#include "../../include/control/PIDTestScenarios.h"
#include "../../include/control/PIDTypes.h"
#include <string>
#include <chrono>
#include <thread>
#include <future>
#include <tuple>

using namespace SimCore;
std::string readFileAsString(const std::string& filePath) {
    std::ifstream inFile(filePath);
    std::stringstream buffer;
    buffer << inFile.rdbuf();
    return buffer.str();
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


    unrealDrone drone(configMotor,configBattery,configDrone,configPropeller);
    float NUMBER_OF_RUNS = 100;
    droneControl* controller = drone.drone->controller.get();
    PIDPair PIDHover;
    dataPID PIDAOT;
    PIDPair PIDPITCHROLL;
    for(int i = 0 ; i < 2 ; i++){
        PIDHover = optimize(drone.drone.get(),drone.drone->getController(),NUMBER_OF_RUNS, hoverGroupDuel);
        hoverSetGainDuel(controller,PIDHover);
        PIDAOT = optimize(drone.drone.get(),drone.drone->getController(),NUMBER_OF_RUNS,aotGroup);
        aotSetGain(controller,PIDAOT);
        PIDPITCHROLL = optimize(drone.drone.get(),drone.drone->getController(),NUMBER_OF_RUNS,rollPitchGroup);
        rollPitchSetGain(controller,PIDPITCHROLL);
    }

    std::cout<< "\n";
    displayPID(PIDHover , "PIDHover");
    displayPID(PIDAOT , "PIDAOT");
    displayPID(PIDPITCHROLL , "PIDPITCHROLL");
}