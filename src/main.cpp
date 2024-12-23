#include<iostream>
#include <math.h>
#include <chrono>
#include <array>
#include "../include/constants.h"
#include "../include/vectorMath.h"
#include "../include/vehicle.h"
#include "../include/odeIterator.h"
#include "../include/logs.h"
#include "../include/control.h"
#include <string>




void iterator(Vehicle &rocket){

    while(rocket.Zposition > 0 && rocket.iterations < 1000000){
        rocket.drag();
        rocket.lift();
        reentryBurn(rocket);
        rocket.finVectors = rocket.getFinForceVectors();
        landingBurn(rocket);

        logRocketPosition(rocket);


        rocket.updateState();
        rocket.iterations++;
    }
}



void initParameters(float drymass,
                    float propellentMassLOX,
                    float propellentMassFuel,
                    float consumtionRateLOX,
                    float consumtionRateFuel,
                    float reentryAccel,
                    float initPositionX,
                    float initPositionY,
                    float initPositionZ,
                    float initVelocityX,
                    float initVelocityY,
                    float initVelocityZ,
                    float initVehicleStateX,
                    float initVehicleStateY,
                    float initVehicleStateZ
                    ){
    
    constants::dryMass = drymass;

    constants::maxGAllowedEntry = reentryAccel;

    constants::initFuel = propellentMassFuel;
    constants::initLOX = propellentMassLOX;

    constants::consumtionRateFuel = consumtionRateFuel;
    constants::consumtionRateLOX = consumtionRateLOX;

    constants::initPosition = {initPositionX , initPositionY , initPositionZ};                

    constants::initVelocity = {initVelocityX , initVelocityY ,initVelocityZ}; 

    constants::initVehicleState = {initVehicleStateX , initVehicleStateY , initVehicleStateZ};

    constants::initVehicleState = normalizeVector(constants::initVehicleState);
}





int main(int argc, char* argv[]){
    auto start = std::chrono::high_resolution_clock::now();

    #ifdef __linux__
        initializeVectors(20000); // argv[1] unique ID 

        initParameters( std::stof(argv[2]), // dry mass
                    std::stof(argv[3]), // propellentMassLOX
                    std::stof(argv[4]), // propellentMassFuel
                    std::stof(argv[5]), // consumtionRateLOX
                    std::stof(argv[6]), // consumtionRateFuel
                    std::stof(argv[7]), // reentryAccel
                    std::stof(argv[8]), // Initial Position X
                    std::stof(argv[9]), // Initial Position Y
                    std::stof(argv[10]), // Initial Position Z
                    std::stof(argv[11]), // Initial Velocity X
                    std::stof(argv[12]), // Initial Velocity Y
                    std::stof(argv[13]), // Initial Velocity Z
                    std::stof(argv[14]), // Initial Orientation Vector X
                    std::stof(argv[15]), // Initial Orientation Vector Y
                    std::stof(argv[16]) // Initial Orientation Vector Z
                    );
    #else
        initializeCSV();
    #endif

    Vehicle rocket;
    iterator(rocket);

    #ifdef __linux__
        dataToRam(argv[1]); // argv[1] unique ID 
    #else
        closeCSV();
    #endif

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Runtime: " << duration.count() << " seconds" << "\n";

    return 0;

}