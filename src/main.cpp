#include<iostream>
#include <math.h>
#include <chrono>
#include "../include/vectorMath.h"
#include "../include/vehicle.h"
#include "../include/odeIterator.h"


void iterator(Vehicle &rocket){
    int iterations = 0;
    float timeStep = .001;
    while(rocket.Zposition > 0){
        rocket.updateState();
        iterations++;
    }
    std::cout<< "Time in seconds to hit ground in free fall " << iterations * timeStep;
}



int main(){
    auto start = std::chrono::high_resolution_clock::now();
    float MOI[3] = {.2,.2,.2};
    Vehicle rocket(0,0,500,0,1,0,MOI,10);
    iterator(rocket);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Runtime: " << duration.count() << " seconds" << std::endl;
    return 0;
}