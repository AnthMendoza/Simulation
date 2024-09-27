#include <iostream>

#include "../include/vehicle.h"
#include "../include/odeIterator.h"
#include "../include/constants.h"
#include "../include/vectorMath.h"
#include "../include/aero.h"





Vehicle::Vehicle(float x, float y, float z, float mroll, float mpitch, float myaw , float mMOI[3],float mMass)
    : Xposition(x), Yposition(y), Zposition(z), roll(mroll), pitch(mpitch), yaw(myaw) , mass(mMass){
    memcpy(MOI, mMOI, 3 * sizeof(float));

    Xvelocity = 0;
    Yvelocity = 0;
    Zvelocity = 0;
    rollvelocity = 0;
    pitchvelocity = 0;
    yawvelocity = 0;

    sumOfForces[0] = 0;
    sumOfForces[1] = 0;
    sumOfForces[2] = 0;

    sumOfMoments[0] = 0;
    sumOfMoments[1] = 0;
    sumOfMoments[2] = 0;
    }



// Method implementation to display the vehicle's state
void Vehicle::display() {
    std::cout << "Position: (" << Xposition << ", " << Yposition << ", " << Zposition << ")\n"
              << "Orientation (Roll, Pitch, Yaw): (" << roll << ", " << pitch << ", " << yaw << ")\n"<< MOI[2] << ", " << Xvelocity;
}   

float Vehicle::drag(){
    float velocityVector[3];
    float vehicleVector[3];
    velocityVector[0] = Xvelocity;
    velocityVector[1] = Yvelocity;
    velocityVector[2] = Zvelocity;

    vehicleVector[0] = Xposition;
    vehicleVector[1] = Yposition;
    vehicleVector[2] = Zposition;
    
    float absVelocity = vectorMag(velocityVector);
    float dragAngle = vectorAngleBetween(velocityVector , vehicleVector);
    float drag = .5 * (absVelocity * absVelocity) * area *coeDrag * airDensity(Zposition);
    return drag;
}

void  Vehicle::addForce(float stateVector[3] , float forceIncident , float forceVector[3]){
    sumOfForces[0] += forceVector[0];
    sumOfForces[1] += forceVector[1];
    sumOfForces[2] += forceVector[2];
}

void Vehicle::updateState(){       
    sumOfForces[2] = sumOfForces[2] + constant::gravitationalAcceleration * mass; //adding gravity to the force of Z, becuase this is an acceleration and not a force; The addForce function cannot handle it
    
    Ode(sumOfForces[0] , mass , constant::timeStep , Xvelocity,Xposition);
    Ode(sumOfForces[1] , mass , constant::timeStep , Yvelocity,Yposition);
    Ode(sumOfForces[2] , mass , constant::timeStep , Zvelocity,Zposition);


    rotationalOde(sumOfMoments[0] , MOI[0], constant::timeStep , );
    rotationalOde(sumOfMoments[1] , MOI[1], constant::timeStep , );
    rotationalOde(sumOfMoments[2] , MOI[2], constant::timeStep , );

    sumOfForces[0] = 0; //reset forces to zero for next iteration
    sumOfForces[1] = 0;
    sumOfForces[2] = 0;
    
    sumOfMoments[0] = 0;
    sumOfMoments[1] = 0;
    sumOfMoments[2] = 0;

    
}