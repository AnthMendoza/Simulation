#include "../include/base/odeIterator.h"
#include <cmath>
#include <assert.h>
#include <iostream>
#include <base/units.h>

//this can be done at all at once by adding the forces. then run this for each component 
namespace SimCore{

void Ode(units::scalar force , units::scalar mass , units::scalar timeStep ,units::scalar &velocity ,units::scalar &position){

    units::scalar acceleration = force / mass;

    units::scalar deltaVelocity = acceleration * timeStep;

    velocity += deltaVelocity;

    units::scalar deltaPosition = velocity * timeStep;

    position += deltaPosition;


}




//take the sum of moments 


units::scalar rotationalOde(units::scalar moment , units::scalar MOI , units::scalar timeStep ,units::scalar &angularVelocity){
    if (MOI <= 0.0f) throw std::runtime_error("MOI must be > 0 in rotationalOde()\n");
    units::scalar AngularAcceleration = moment / MOI;
    
    units::scalar deltaAngularVelocity = AngularAcceleration;

    angularVelocity = angularVelocity + deltaAngularVelocity;

    units::scalar deltaAngularPosition = deltaAngularVelocity * timeStep;

    return deltaAngularPosition;
}


}