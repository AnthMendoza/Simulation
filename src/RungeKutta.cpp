#include <cmath>
#include "../include/RungeKutta.h"

namespace SimCore{

float acceleration(float force , float mass){
    return force / mass;
}

void RungeKutta4th(float force, float mass, float timeStep, float &velocity, float &position) {
    


    float deltaV = acceleration(force , mass) * timeStep;

    float k1x = (velocity + deltaV) * timeStep;

   
    float k2x = (velocity + k1x / 2.0f) * timeStep;


    float k3x = (velocity + k2x / 2.0f) * timeStep;


    float k4x = (velocity + k3x) * timeStep;

    // weighted averge of the samples
    velocity = velocity + deltaV;
    
    position = position + (k1x + 2 * k2x + 2 * k3x + k4x) / 6.0f;
}
}