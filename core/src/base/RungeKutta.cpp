#include <cmath>
#include "../include/base/RungeKutta.h"
#include <base/units.h>

namespace SimCore{

units::scalar acceleration(units::scalar force , units::scalar mass){
    return force / mass;
}

void RungeKutta4th(units::scalar force, units::scalar mass, units::scalar timeStep, units::scalar &velocity, units::scalar &position) {
    


    units::scalar deltaV = acceleration(force , mass) * timeStep;

    units::scalar k1x = (velocity + deltaV) * timeStep;

   
    units::scalar k2x = (velocity + k1x / 2.0f) * timeStep;


    units::scalar k3x = (velocity + k2x / 2.0f) * timeStep;


    units::scalar k4x = (velocity + k3x) * timeStep;

    // weighted averge of the samples
    velocity = velocity + deltaV;
    
    position = position + (k1x + 2 * k2x + 2 * k3x + k4x) / 6.0f;
}
}