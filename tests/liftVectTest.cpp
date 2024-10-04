
#include <array>
#include <iostream>
#include "../include/vectorMath.h"
#include <array>


int main(){
    std::array<float , 3> airVelocityVector = {0,0,1}; 
    std::array<float , 3> vehicleState = {3,4,5}; 

    std::array<float , 3> normalAirVelocityVector = normalizeVector(airVelocityVector);
    std::array<float , 3> normalVehicleState = normalizeVector(vehicleState);
    
    float projection = vectorDotProduct( normalVehicleState , normalAirVelocityVector);

    std::array<float , 3> projectedVector;

    for(int i = 0 ; i < 3 ; i++){
        projectedVector[i] = projection * normalAirVelocityVector[i];
    }

    for(int i = 0 ; i < 3 ; i++){
        projectedVector[i] =  normalVehicleState[i] - projectedVector[i];
    }

    std::cout<< "vehicle state "  << normalVehicleState[0]<< " "<< normalVehicleState[1]<< " " << normalVehicleState[2] << "\n";

    std::cout<< "air vect      "  << normalAirVelocityVector[0]<< " "<< normalAirVelocityVector[1]<< " " << normalAirVelocityVector[2] << "\n";

    std::cout<< "projected     "  << projectedVector[0]<< " "<< projectedVector[1]<< " " << projectedVector[2] << "\n";

    return 0;
}