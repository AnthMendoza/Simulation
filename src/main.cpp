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

    while(rocket.Zposition > 0){
        rocket.drag();
        rocket.lift();
        reentryBurn(rocket);

        if(rocket.iterations%40 == 0)logRocketPosition(rocket);


        rocket.updateState();

        rocket.iterations++;
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