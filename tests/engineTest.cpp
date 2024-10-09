#include <iostream>
#include <cmath>
#include <array>
#include "../include/vehicle.h"


int main(){
    Vehicle rocket;
    std::array<float,2> vect = {0,0};
    rocket.applyEngineForce(vect , 10.0f);

    std::cout<<rocket.vehicleState[0] << " "<< rocket.vehicleState[1] << " "<< rocket.vehicleState[2];

}