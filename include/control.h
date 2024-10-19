#ifndef CONTROL_H
#define CONTROL_H

#include "../include/vehicle.h"

    
Vehicle &lookAhead(Vehicle &rocket, float lookAheadTime);

float PID(float target , float currentState , float &previousError , float &sumOfError , float timeStep, float Pgain , float Igain , float Dgain);


void reentryBurn(Vehicle &rocket);

void glidToTarget(Vehicle &rocket);

void landingBurn(Vehicle &rocket);



#endif