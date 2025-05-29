
#include <math.h>
#include <chrono>
#include <array>
#include <string>
#include <cstdlib> 
#include <fstream>
#include <sstream>
#include <iostream>
#include "../include/vectorMath.h"
#include "../include/vehicle.h"
#include "../include/odeIterator.h"
#include "../include/logs.h"
#include "../include/control.h"
#include "../include/sensors.h"
#include "../include/rocket.h"
#include "../include/quaternion.h"
#include <string>

namespace SimCore{
void iterator(Rocket &rocket ,loggedData *data){
    while(rocket.getPositionVector()[2] > 0 && rocket.getIterations() < 1000000){
        rocket.drag();
        rocket.lift();
        //rocket.glideToTarget();
        rocket.reentryBurn();
        rocket.landingBurn();
        data->logRocketPosition(rocket);
        rocket.updateState();
        ++rocket;
    }
    data->writeCSV(rocket.outputFile,data->all());
}


 
//void initParameters(float drymass,
//                    float propellentMassLOX,
//                    float propellentMassFuel,
//                    float consumtionRateLOX,
//                    float consumtionRateFuel,
//                    float reentryAccel,
//                    float initPositionX,
//                    float initPositionY,
//                    float initPositionZ,
//                    float initVelocityX,
//                    float initVelocityY,
//                    float initVelocityZ,
//                    float initVehicleStateX,
//                    float initVehicleStateY,
//                    float initVehicleStateZ
//                    ){
//    
//    constants::dryMass = drymass;
//
//    constants::maxGAllowedEntry = reentryAccel;
//
//    constants::initFuel = propellentMassFuel;
//    constants::initLOX = propellentMassLOX;
//
//    constants::consumtionRateFuel = consumtionRateFuel;
//    constants::consumtionRateLOX = consumtionRateLOX;
//
//    constants::initPosition = {initPositionX , initPositionY , initPositionZ};                
//
//    constants::initVelocity = {initVelocityX , initVelocityY ,initVelocityZ}; 
//
//    constants::initVehicleState = {initVehicleStateX , initVehicleStateY , initVehicleStateZ};
//
//    constants::initVehicleState = normalizeVector(constants::initVehicleState);
//}





// Read TOML file into a string
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

    if(argc > 0 ){
    //    initializeVectors(20000); // argv[1] unique ID 
    //    constants::initPosition[2] = std::stof(argv[2]); // dry mass
    //    constants::initVelocity[1] = std::stof(argv[3]);
    //    constants::initVelocity[2] = std::stof(argv[4]);
    //    constants::initVelocity[0] = 0;
    //                std::stof(argv[3]), // propellentMassLOX
    //                std::stof(argv[4]), // propellentMassFuel
    //                std::stof(argv[5]), // consumtionRateLOX
    //                std::stof(argv[6]), // consumtionRateFuel
    //                std::stof(argv[7]), // reentryAccel
    //                std::stof(argv[8]), // Initial Position X
    //                std::stof(argv[9]), // Initial Position Y
    //                std::stof(argv[10]), // Initial Position Z
    //                std::stof(argv[11]), // Initial Velocity X
    //                std::stof(argv[12]), // Initial Velocity Y
    //                std::stof(argv[13]), // Initial Velocity Z
    //                std::stof(argv[14]), // Initial Orientation Vector X
    //                std::stof(argv[15]), // Initial Orientation Vector Y
    //                std::stof(argv[16]) // Initial Orientation Vector Z
    //                );
    }
    
    SimCore::loggedData* data = new SimCore::loggedData;
    std::string configFileData = SimCore::readFileAsString(argv[1]);
    SimCore::Rocket rocket(configFileData);
    rocket.init();
    SimCore::iterator(rocket , data);
    delete data;
    std::string outputFile = "../output.csv";
    std::string command = "python3 ../src/plot.py "+ outputFile;
    const char* com = command.c_str();

    system(com);

    return 0;

}
