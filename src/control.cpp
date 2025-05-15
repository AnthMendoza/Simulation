
#include <vector>
#include <functional>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "../include/control.h"
#include "../include/vehicle.h"
#include "../include/constants.h"
#include "../include/logs.h"
#include "../include/vectorMath.h"
#include "../include/getRotation.h"
#include "../include/sensors.h"

//initally stanley controller will be driving towards a becon regardless of z position. 
//The becon being the landing spot defined by a line {xLandingPoint,yLandingPoint,z} where z is all reals.
//After proof of concept the target should be a generated spline to the target.

float StanleyController::computeSteering(float headingError, float crossTrackError, float velocity){
    velocity = std::max(velocity, 0.1f);  // Avoid division by zero
    float cteTerm = std::atan2(k * crossTrackError, velocity);
    float steering = headingError + cteTerm;

    steering = std::clamp(steering, -maxSteering, maxSteering);
    return steering;
}


//predictionVehicle::predictionVehicle(Vehicle &vehicle){
//    
//}
//
//void predictionVehicle::landingPrediction(float horizion){
//
//}


