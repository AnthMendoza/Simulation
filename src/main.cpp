#include<iostream>
#include <math.h>
#include <chrono>
#include <array>
#include "../include/constants.h"
#include "../include/vectorMath.h"
#include "../include/vehicle.h"
#include "../include/odeIterator.h"
#include "../include/logs.h"




void iterator(Vehicle &rocket){
    int iterations = 0;
    while(rocket.Zposition > 0){
        rocket.drag();
        rocket.lift();
        
        if(iterations%40 == 0)logRocketPosition(rocket , iterations);

        if(iterations * constants::timeStep > 89 && iterations * constants::timeStep < 109) rocket.reentryBurn();

        rocket.updateState();

        iterations++;
    }
}



void lookAhead(Vehicle &rocket, float lookAheadTime){
    //make a copy of the rocket so that we can keep current conditions on the main vehicle and test parameters on the look ahead
    Vehicle lookAheadRocket = rocket; 
    float iterations = 0;
    float limit = lookAheadTime/constants::timeStep;

    while(iterations < limit && lookAheadRocket.Zposition > 0){
        lookAheadRocket.drag();
        lookAheadRocket.lift();
        lookAheadRocket.updateState();
        iterations++;
    }
}



int main(){
    auto start = std::chrono::high_resolution_clock::now();
    initializeCSV();

    Vehicle rocket;
    iterator(rocket);

    closeCSV();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Runtime: " << duration.count() << " seconds" << std::endl;

    return 0;



}