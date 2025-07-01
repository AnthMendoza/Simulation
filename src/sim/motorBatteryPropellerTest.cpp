#include "../../include/sim/MySim.h"
#include <iostream>
#include <math.h>
#include <chrono>
#include <array>
#include <string>
#include <cstdlib> 
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>
#include "../../include/subsystems/motor.h"
#include "../../include/subsystems/propeller.h"
#include "../../include/dynamics/aero.h"

namespace SimCore{
void writeCSV( const std::string& filename,const std::vector<std::shared_ptr<std::vector<float>>>& data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        //handle error
        return;
    }
    if(data.empty()) return;
    int numRows = (*(data[0])).size();
    for(int i = 0; i < numRows; ++i) {
        for(int j = 0 ; j < data.size() ; j++){
            if(j<data.size() - 1 )file << (*data[j])[i]<<",";
            else file << (*data[j])[i];
        }
        file << "\n";
    }

    file.close();
}


std::string readFileAsString(const std::string& filePath) {
    std::ifstream inFile(filePath);
    std::stringstream buffer;
    buffer << inFile.rdbuf();
    return buffer.str();
}

}
int main(int argc, char* argv[]){
    if(argv[1] == nullptr){
        //std::cout<< "Specify vehicle config file path";
        return 1;
    }

    std::string configMotor = SimCore::readFileAsString(argv[1]);    
    std::string configBattery = SimCore::readFileAsString(argv[2]);   
    std::string configPropeller = SimCore::readFileAsString(argv[3]);
    float timeStep = .001;
    std::vector<std::shared_ptr<std::vector<float>>> data; 
    std::shared_ptr<std::vector<float>> one = std::make_shared<std::vector<float>>();
    std::shared_ptr<std::vector<float>> two = std::make_shared<std::vector<float>>();
    std::shared_ptr<std::vector<float>> three = std::make_shared<std::vector<float>>();

    std::vector<std::unique_ptr<SimCore::motor>> motors;
    std::vector<std::unique_ptr<SimCore::propeller>> propellers;
    SimCore::battery bat(configBattery);
    float voltage = bat.getBatVoltage();
    for(int i = 0 ; i < 4 ; i ++){
        motors.push_back(std::make_unique<SimCore::motor>(configMotor,timeStep));
        propellers.push_back(std::make_unique<SimCore::propeller>(configPropeller));
    }
    
    for(int i = 0 ; i < 10000;i++){
        one->push_back(timeStep*i);
        float current = 0;
        float thrust = 0;
        float altitude = 10;
        for(int i = 0; i < motors.size(); i++){
            float density = SimCore::airDensity(altitude);
            motors[i]->updateMotorAngularVelocity(timeStep,propellers[i]->dragTorque(density,motors[i]->getCurrentAngularVelocity()),bat,100.0f);
            current += std::abs(motors[i]->getCurrentCurrent());
            thrust += propellers[i]->thrustForce(density,motors[i]->getCurrentAngularVelocity());
        }
        //std::cout<<thrust<<","<< motors[1]->getCurrentCurrent()<<"\n";
        bat.updateBattery(current);
        voltage = bat.getBatVoltage(); 
        two->push_back(motors[0]->getCurrentAngularVelocity());
        three->push_back(thrust);
    }
    data.push_back(one);
    data.push_back(two);
    data.push_back(three);
    SimCore::writeCSV("motordata.csv" , data);
    return 0;

}