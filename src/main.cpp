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

    constants::initFuel = propellentMassFuel;
    constants::initLOX = propellentMassLOX;
    constants::consumtionRateFuel = consumtionRateFuel;
    constants::consumtionRateLOX = consumtionRateLOX;

    constants::initPosition = {initPositionX , initPositionY , initPositionZ};                

    constants::initVelocity = {initVelocityX , initVelocityY ,initVelocityZ}; 

    constants::initVehicleState = {initVehicleStateX , initVehicleStateY , initVehicleStateZ};
}





int main(int argc, char* argv[]){
    auto start = std::chrono::high_resolution_clock::now();

    #ifdef __linux__
        initializeVectors(20000); // argv[1] unique ID 
        initParameters( argv[2], // dry mass
                        argv[3], // Initial Position X
                        argv[4], // Initial Position Y
                        argv[5], // Initial Position Z
                        argv[6], // Initial Velocity X
                        argv[7], // Initial Velocity Y
                        argv[8], // Initial Velocity Z
                        argv[9], // Initial Orientation Vector X
                        argv[10], // Initial Orientation Vector Y
                        argv[11], // Initial Orientation Vector Z
                        argv[12], 
                        argv[13],
                        argv[14],
                        argv[15],)
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
    std::cout << "Runtime: " << duration.count() << " seconds" << std::endl;

    return 0;



}