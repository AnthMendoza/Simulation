#include "../../include/sim/MySim.h"
#include "../../include/control/PIDGains.h"
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
    std::tuple<float,float,float> PID = optimize(drone.drone.get(),drone.drone->getController(),70,hoverTestIterator,hoverTestSetup);

    std::cout<<std::get<0>(PID) << ","<< std::get<1>(PID) << ","<< std::get<2>(PID)<<"\n";
}