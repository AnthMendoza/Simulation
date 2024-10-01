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
        rocket.updateState();
        if(iterations%20 == 0)logRocketPosition(rocket , iterations);
        iterations++;
    }
}



int main(){
    auto start = std::chrono::high_resolution_clock::now();
    initializeCSV();
    std::array<float , 3> MOI = {.2,.2,.2}; //get rid of this 

    Vehicle rocket(0,0,500,MOI, 42000);
    iterator(rocket);

    auto end = std::chrono::high_resolution_clock::now();

    closeCSV();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Runtime: " << duration.count() << " seconds" << std::endl;

    return 0;



}