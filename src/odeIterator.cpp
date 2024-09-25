
#include <vector>


//this can be done at all at once by adding the forces... 


void Ode(float force , float mass , float timeStep ,float &velocity ,float &position){

    float acceleration = force / mass;

    float deltaVelocity = acceleration * timeStep;

    float deltaPosition = deltaVelocity * timeStep;

    velocity = velocity + deltaVelocity;
    
    position = position + deltaPosition;


}