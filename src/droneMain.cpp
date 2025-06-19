#include "../include/MySim.h"
#include <string>
#include <chrono>
#include <thread>
using namespace SimCore;
std::string readFileAsString(const std::string& filePath) {
    std::ifstream inFile(filePath);
    std::stringstream buffer;
    buffer << inFile.rdbuf();
    return buffer.str();
}

int main(int argc, char* argv[]){
    if(argv[1] == nullptr){
        //std::cout<< "Specify vehicle config file path";
        return 1;
    }

    std::string configMotor = readFileAsString(argv[1]);    
    std::string configBattery = readFileAsString(argv[2]);   
    std::string configPropeller = readFileAsString(argv[3]);
    std::string configDrone = readFileAsString(argv[4]);


    unrealDrone drone(configMotor,configBattery,configDrone,configPropeller);
    drone.setTargetPosition(0,0,100,0);
    while(true){
        drone.simFrameRequest(1);
        drone.drone->display();
        drone.drone->droneDisplay();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 1;
}