#include "../../include/sim/MySim.h"
#include "../../include/control/PIDGains.h"
#include "../../include/core/pythonConnector.h"
#include "../../include/utility/utility.h"
#include <sstream>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <tuple>

using namespace SimCore;


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
    drone.setTargetPosition(50,50,300,0);
    drone.drone->turbulantZ->setStdDev(0.5);
    drone.drone->turbulantX->setStdDev(5);
    drone.drone->turbulantY->setStdDev(5);
    //in milliseconds (1 second)
    float TimePerTelemetry = 1000.0f; 
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        unrealDataDrone* data = drone.simFrameRequest(TimePerTelemetry / 1000.0f); 

        std::string bufferDashboard =
            drone.drone->display() + "\n" +
            drone.drone->droneDisplay();    
        printDynamicDisplay(bufferDashboard);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float, std::milli> duration = end - start;
        float delay = TimePerTelemetry - duration.count();
        if (delay < 0) {
            std::cerr << "Warning: TimePerTelemetry is too small or simulation is running too slow to display in real time.\n";
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay)));
        }
    }

}