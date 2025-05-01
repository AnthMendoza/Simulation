
#include <vector>
#include <functional>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "../include/control.h"
#include "../include/vehicle.h"
#include "../include/constants.h"
#include "../include/logs.h"
#include "../include/vectorMath.h"
#include "../include/getRotation.h"
#include "../include/sensors.h"



predictionVehicle::predictionVehicle(Vehicle &vehicle){
    
}

void predictionVehicle::landingPrediction(float horizion){

}

std::vector<float> lookAhead(Vehicle &rocket, float lookAheadTime , std::function<float(Vehicle&)> valueToLog){
    //make a copy of the rocket so that we can keep current conditions on the main vehicle and test parameters on the look ahead
    int initalIterations = rocket.iterations;
    float limit = lookAheadTime/constants::timeStep;
    std::vector<float> log;
    while(rocket.iterations - initalIterations < limit && rocket.Zposition > 0){
        
        rocket.drag();
        rocket.lift();
        rocket.updateState();
        log.push_back(valueToLog(rocket));
        rocket.iterations++;
    }

    return log;
}



float PID(float target , float currentState , float &previousError , float &sumOfError , float timeStep, float Pgain , float Igain , float Dgain){

    float error = target - currentState;

    sumOfError = error * timeStep;

    float slope = (error - previousError) / timeStep;

    previousError = error;

    return Pgain * error + Igain * sumOfError + Dgain * slope;


}

// make sure sensors play well with lookahead
void reentryBurn(Vehicle &rocket , loggedData *data){
    
    if(rocket.reentry == true) return;

    if(rocket.Zposition < 55000 && rocket.reentry == false){

        rocket.reentry = true;
        float currentMaxGForce = constants::maxGAllowedEntry + 1;
        float lastMaxGForce = currentMaxGForce;
        //main loop moves forward 1 second every lookAHead cylce
        float stepInterval = 1;
        std::array<float,2> direction = {0,0};
        float count = 0;
        while(currentMaxGForce > constants::maxGAllowedEntry){
            Vehicle lookAheadRocket = rocket;
            lookAheadRocket.initSensors();
            std::vector<float> gForces = lookAhead(lookAheadRocket , 105 , [](Vehicle &r) { return r.gForce; }); // 105 is the lookahead time in seconds. this may be stoppped earlier if the vehicle hits the ground
            if(gForces.size()!= 0)currentMaxGForce = *std::max_element(gForces.begin(),gForces.end());
            else currentMaxGForce = constants::maxGAllowedEntry + 1;
            if(currentMaxGForce < constants::maxGAllowedEntry && count > 0) return;
            if(currentMaxGForce > lastMaxGForce && count > 0) return;
            lastMaxGForce = currentMaxGForce;
            float currentIteration = rocket.iterations;
            count++;
            while(rocket.iterations < currentIteration + stepInterval/constants::timeStep && rocket.Zposition > 0){
                rocket.drag();
                rocket.lift();
                rocket.applyEngineForce(direction , constants::maxThrust * .6 * 3); 
                rocket.finVectors = rocket.getFinForceVectors();
                data->logRocketPosition(rocket);
                rocket.updateState();
                rocket.iterations++;
            }
        }
    }
}



void lookAheadGlide(Vehicle &rocket ,float* log , int logSize){
    //make a copy of the rocket so that we can keep current conditions on the main vehicle and test parameters on the look ahead

    int initalIterations = rocket.iterations;
    while(rocket.iterations - initalIterations < logSize && rocket.Zposition > 0){
        rocket.drag();
        rocket.lift();
        rocket.updateState();
        log[initalIterations - rocket.iterations] = rocket.Xvelocity; 
        rocket.iterations++;

        logSize = 1;
    }
    
}





void glidToTarget(Vehicle &rocket){
    //rate of change is what we are looking for here
    //lookahead to see what the rate of change given 
    //goal: arrest any drift
    //vehicle vertical is not needed but would be nice
    if(rocket.reentry == false) return;

    float delta  = rocket.Xposition - rocket.targetLandingPosition[1];

    std::array<float , 3> velo = {rocket.Xvelocity , rocket.Yvelocity , rocket.Zvelocity};
    std::array<float,3> targetVector = {0,0,1};

    float stanley = vectorAngleBetween(velo , targetVector) + atanf(2.0f * (delta/rocket.getVelocity()));
    
    rocket.sumOfMoments[0] = stanley * 10000;
    

}



void landingBurn(Vehicle &rocket){

    if(rocket.reentry == false) return;
    if(0 != rocket.iterations % 20 && rocket.landingInProgress){
        rocket.engineGimbal( rocket.PIDOutputY , rocket.PIDOutputX );
        rocket.applyEngineForce(rocket.landingGimbalDirection, rocket.landingRequiredThrust);
        return;
    }
    std::array<float,3> velocity = rocket.getEstimatedVelocity();
    std::array<float,3> rotation = rocket.getEstimatedRotation();
    
    float landingAccelZ = ((velocity[2] * velocity[2]) / (2 * rocket.Zposition)) - constants::gravitationalAcceleration;

    float landingBurnDuration =  velocity[2] / landingAccelZ;

    float landingAccelX = velocity[0]/ landingBurnDuration;

    float landingAccelY = velocity[1]/ landingBurnDuration;
    // as landing duration approches 0 landing acceleration X and Y grows rapidly 
    // lim 1/x as x approches +0 is infinity 
    if(landingBurnDuration < 1.0f ){ 
        landingAccelX = 0;
        landingAccelY = 0;
    }
    
    float landingForceX = landingAccelX * rocket.mass;

    float landingForceY = landingAccelY * rocket.mass;

    float landingForceZ = landingAccelZ * rocket.mass;

    rocket.landingRequiredThrust = sqrtf(landingForceX * landingForceX + landingForceY * landingForceY + landingForceZ * landingForceZ);

    if(rocket.landingRequiredThrust>= constants::landingThrust){
        rocket.landingInProgress = true;
    }

    if(rocket.landingRequiredThrust <= constants::minThrust || rocket.landingInProgress == false){
        rocket.landingInProgress = false; 
        return;
    }

    if(rocket.landingRequiredThrust > constants::maxThrust) rocket.landingRequiredThrust = constants::maxThrust;
    

    std::array<float,3> forceVector = {landingForceX , landingForceY , landingForceZ};


    std::array<float , 3> directionVector = normalizeVector(forceVector);

    rocket.landingGimbalDirection= {0,0};
        
    std::array<float,2> twoDState = {rotation[1], rotation[2]};

    std::array<float,2> targetState = {directionVector[1] , directionVector[2]};


    float error = twodAngleDiffrence( twoDState, targetState); 

    rocket.error = error;

    rocket.twoDAngle = twoDState;

    rocket.PIDOutputY = PID(0,error,rocket.vehicleYError, rocket.sumOfVehicleYError, constants::timeStep , 2 ,0 , 1);
    
    //std::cout<< twoDState[0] << "," << twoDState[1] << "  " << targetState[0] << "," << targetState[1]<< "   "<< error  << "   " << PIDOutputY << "\n";

    twoDState = {rotation[0] , rotation[2]};

    targetState = {directionVector[0] , directionVector[2]};

    error  = twodAngleDiffrence( twoDState, targetState);

    rocket.PIDOutputX = PID(0,error,rocket.vehicleXError, rocket.sumOfVehicleXError, constants::timeStep , 2 ,0, 1);



    rocket.engineGimbal( rocket.PIDOutputY , rocket.PIDOutputX );


    rocket.landingGimbalDirection[1] = -rocket.gimbalY;
    rocket.landingGimbalDirection[0] = rocket.gimbalX;


    rocket.applyEngineForce(rocket.landingGimbalDirection, rocket.landingRequiredThrust);



}





