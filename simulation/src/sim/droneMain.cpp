#include "../../include/sim/MySim.h"
#include "../../include/control/PIDGains.h"
#include "../../include/core/pythonConnector.h"
#include <string>
#include <chrono>
#include <thread>
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
    drone.setTargetPosition(0,50,200,0);
    //in milliseconds (1 second)
    float TimePerTelemetry = 1000.0f; 
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        unrealDataDrone* data = drone.simFrameRequest(TimePerTelemetry / 1000.0f); 
        
        //drone.drone->display();
        //drone.drone->droneDisplay();

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