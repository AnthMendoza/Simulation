#include <stdio.h>
#include<iostream>
#include <cstdio>
#include <stdlib.h>
#include <stdbool.h>  
#include <math.h>
#include <thread>
#include <algorithm> 
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
    float MOI[3] = {.2,.2,.2};
    Vehicle rocket(0,0,50000,0,1,0,MOI,10);
    iterator(rocket);
    return 0;
}