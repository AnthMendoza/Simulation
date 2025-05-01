#ifndef CONTROL_H
#define CONTROL_H

#include "../include/vehicle.h"
#include "../include/logs.h"


//Predicition Vehicle is used to simulate the vehicle in ideal conditions.
//perfect state estimation contorl will be active unlike lookAhead.
class predictionVehicle{

    Vehicle predictVehicle;

    public:

    predictionVehicle(Vehicle &vehicle);
    //short term Prediction that the vehicle will weight heavily as the vehicle approches the gorund
    //horizion is time in seconds for casting the predition {keep short}
    void landingPrediction(float horizion);


    

};
    
Vehicle &lookAhead(Vehicle &rocket, float lookAheadTime);

float PID(float target , float currentState , float &previousError , float &sumOfError , float timeStep, float Pgain , float Igain , float Dgain);


void reentryBurn(Vehicle &rocket , loggedData *data);

void glidToTarget(Vehicle &rocket);

void landingBurn(Vehicle &rocket);



#endif