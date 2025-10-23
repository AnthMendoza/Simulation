#include "../../include/sim/MySim.h"
//#include "../../include/control/PIDGains.h"
#include "../../include/core/pythonConnector.h"
#include "../../include/utility/utility.h"
#include <sstream>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <tuple>
#include <atomic>
#include <thread>
#include "../../include/utility/logger.h"

using namespace SimCore;

#define REAL_TIME_REPRESENTATION




int main(int argc, char* argv[]){
    if (argc < 5) {
        std::cout << "Usage: " << argv[0] << " <motor_config> <battery_config> <propeller_config> <drone_config>\n";
        return 1;
    }
    #ifdef DRONE_LOGGING
    const float maxDurationSeconds = 30;
    #endif

    std::string configMotor = readFileAsString(argv[1]);    
    std::string configBattery = readFileAsString(argv[2]);   
    std::string configPropeller = readFileAsString(argv[3]);
    std::string configDrone = readFileAsString(argv[4]);


    unrealDrone drone(configMotor,configBattery,configDrone,configPropeller);
    drone.setTargetPosition(0,0,20,0);
    drone.drone->turbulantZ->setStdDev(0.5);
    drone.drone->turbulantX->setStdDev(0.5);
    drone.drone->turbulantY->setStdDev(0.5);
    //drone.drone->setStateVector({0,0,1},{0,1,0});

    float TimePerTelemetry = 1000.0f; 
    bool running = true;

    while (running) {
        auto start = std::chrono::high_resolution_clock::now();

        unrealDataDrone* data = drone.simFrameRequest(TimePerTelemetry / 1000.0f); 

        std::string bufferDashboard =
            drone.drone->display() + "\n" +
            drone.drone->droneDisplay();    
        printDynamicDisplay(bufferDashboard);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float, std::milli> duration = end - start;
        float delay = TimePerTelemetry - duration.count();

        #ifdef DRONE_LOGGING

        if(drone.drone->getTime() >= maxDurationSeconds){
            running = false;
        }
        #endif

        #ifdef REAL_TIME_REPRESENTATION
        if (delay < 0) {
            std::cerr << "Warning: TimePerTelemetry is too small or simulation is running too slow to display in real time.\n";
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay)));
        }
        #endif
    }

    #ifdef DRONE_LOGGING
    log::logger->exportCSV("droneLog.csv");
    #endif
    return 0;
}