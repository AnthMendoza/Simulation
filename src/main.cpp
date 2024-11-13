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

        if(rocket.iterations%40 == 0)logRocketPosition(rocket);


        rocket.updateState();
        rocket.iterations++;
    }
}





int main(int argc, char* argv[]){
    auto start = std::chrono::high_resolution_clock::now();

    #ifdef __linux__
        initializeVectors(10000);
    #else
        initializeCSV();
    #endif


    Vehicle rocket;
    iterator(rocket);

    #ifdef __linux__
        dataToRam(argv[1]);
    #else
        closeCSV();
    #endif

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Runtime: " << duration.count() << " seconds" << std::endl;

    return 0;



}