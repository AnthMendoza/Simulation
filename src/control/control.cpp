
#include <vector>
#include <functional>
#include <algorithm>

#include <cmath>
#include <algorithm>
#include "../../include/control/control.h"
#include "../../include/dynamics/vehicle.h"

#include "../../include/sim/logs.h"
#include "../../include/core/vectorMath.h"
#include "../../include/core/getRotation.h"
#include "../../include/subsystems/sensors.h"

//initally stanley controller will be driving towards a becon regardless of z position. 
//The becon being the landing spot defined by a line {xLandingPoint,yLandingPoint,z} where z is all reals.
//After proof of concept the target should be a generated spline to the target.
namespace SimCore{
StanleyController::StanleyController(float gain, float maxSteeringAngle): k(gain), maxSteering(maxSteeringAngle){}

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

}
