#ifndef CONTROL_H
#define CONTROL_H

#include "../include/vehicle.h"

    
Vehicle &lookAhead(Vehicle &rocket, float lookAheadTime);


void reentryBurn(Vehicle &rocket);

void glidToTarget(Vehicle &rocket);

void landingBurn(Vehicle &rocket);


#endif