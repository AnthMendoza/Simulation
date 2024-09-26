#include <vector>
#include "../include/odeIterator.h"

//this can be done at all at once by adding the forces. then run this for each component


void Ode(float force , float mass , float timeStep ,float &velocity ,float &position){

    float acceleration = force / mass;

    float deltaVelocity = acceleration * timeStep;

    velocity = velocity + deltaVelocity;

    float deltaPosition = velocity * timeStep;

    position = position + deltaPosition;


}


//take the sum of moments 


void rotationalOde(float moment , float MOI , float timeStep ,float &angularVelocity ,float &rotationalPosition){

    float deltaAngularAcceleration = moment/MOI;

    float deltaAngularVelocity = deltaAngularAcceleration * timeStep;

    angularVelocity = angularVelocity + deltaAngularVelocity;

    float deltaAngularPosition = deltaAngularVelocity * timeStep;

    rotationalPosition = rotationalPosition + deltaAngularPosition;
}
