#include "../include/odeIterator.h"
#include <cmath>
#include <assert.h>
#include <iostream>
//this can be done at all at once by adding the forces. then run this for each component 
namespace SimCore{

void Ode(float force , float mass , float timeStep ,float &velocity ,float &position){

    float acceleration = force / mass;

    float deltaVelocity = acceleration * timeStep;

    velocity += deltaVelocity;

    float deltaPosition = velocity * timeStep;

    position += deltaPosition;


}




//take the sum of moments 


float rotationalOde(float moment , float MOI , float timeStep ,float &angularVelocity){
    if (MOI <= 0.0f) throw std::runtime_error("MOI must be > 0 in rotationalOde()\n");
    float AngularAcceleration = moment / MOI;
    
    float deltaAngularVelocity = AngularAcceleration;

    angularVelocity = angularVelocity + deltaAngularVelocity;

    float deltaAngularPosition = deltaAngularVelocity * timeStep;

    return deltaAngularPosition;
}


}