#include<iostream>
#include <math.h>
#include <chrono>
#include <array>
#include "../include/vectorMath.h"
#include "../include/vehicle.h"
#include "../include/odeIterator.h"
#include "../include/logs.h"


void iterator(Vehicle &rocket){
    int iterations = 0;
    while(rocket.Zposition > 0){
        rocket.drag();
        rocket.lift();
        
        if(iterations%4 == 0)logRocketPosition(rocket , iterations);
        rocket.updateState();

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