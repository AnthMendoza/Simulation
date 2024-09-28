#include<iostream>
#include <math.h>
#include <chrono>
#include <array>
#include "../include/vectorMath.h"
#include "../include/vehicle.h"
#include "../include/odeIterator.h"


void iterator(Vehicle &rocket){
    int iterations = 0;
    float timeStep = .001;
    while(rocket.Zposition > 0){
        rocket.drag();
        rocket.updateState();
        //rocket.display();
        iterations++;
    }
    std::cout<< "Time in seconds to hit ground in free fall " << iterations * timeStep;
}



int main(){
    auto start = std::chrono::high_resolution_clock::now();

    std::array<float , 3> MOI = {.2,.2,.2};

    Vehicle rocket(0,0,500000,MOI, 42000);
    iterator(rocket);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Runtime: " << duration.count() << " seconds" << std::endl;

    return 0;



}