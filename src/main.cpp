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
}





int main(int argc, char* argv[]){
    auto start = std::chrono::high_resolution_clock::now();

    #ifdef __linux__
        initializeVectors(20000); // argv[1] unique ID 
        initParameters( argv[2], // dry mass
                        argv[3], // propellentMassLOX
                        argv[4], // propellentMassFuel
                        argv[5], // consumtionRateLOX
                        argv[6], // consumtionRateFuel
                        argv[7], // reentryAccel
                        argv[8], // Initial Position X
                        argv[9], // Initial Position Y
                        argv[10], // Initial Position Z
                        argv[11], // Initial Velocity X
                        argv[12], // Initial Velocity Y
                        argv[13], // Initial Velocity Z
                        argv[14], // Initial Orientation Vector X
                        argv[15], // Initial Orientation Vector Y
                        argv[16], // Initial Orientation Vector Z
                        )
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