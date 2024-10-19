
#include <vector>
#include <functional>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "../include/vehicle.h"
#include "../include/constants.h"
#include "../include/logs.h"
#include "../include/vectorMath.h"
#include "../include/getRotation.h"



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


void reentryBurn(Vehicle &rocket){
    
    if(rocket.reentry == true) return;


    if(rocket.Zposition < 55000 && rocket.reentry == false){

        rocket.reentry = true;
        float currentMaxGForce = constants::maxGAllowedEntry + 1;
        //main loop moves forward 1 second every lookAHead cylce
        float stepInterval = 1;
        std::array<float,2> direction = {0,0};
        while(currentMaxGForce > constants::maxGAllowedEntry){
            Vehicle lookAheadRocket = rocket;
            std::vector<float> gForces = lookAhead(lookAheadRocket , 105 , [](Vehicle &r) { return r.gForce; }); 
            
            currentMaxGForce = *std::max_element(gForces.begin(),gForces.end());
            std::cout<<currentMaxGForce<< std::endl;
            if(currentMaxGForce < constants::maxGAllowedEntry) return;
            float currentIteration = rocket.iterations;
            while(rocket.iterations < currentIteration + stepInterval/constants::timeStep && rocket.Zposition > 0){
                rocket.drag();
                rocket.lift();
                rocket.applyEngineForce(direction , constants::maxThrust * .6 * 3); 
                rocket.finVectors = rocket.getFinForceVectors();
                if(rocket.iterations%40 == 0)logRocketPosition(rocket);
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
    }
    

}




void glidToTarget(Vehicle &rocket){
    //rate of change is what we are looking for here
    //lookahead to see what the rate of change given 
    //goal: arrest any drift
    //vehicle vertical is not needed but would be nice
    if(rocket.reentry == false) return;

    float delta  = rocket.Xposition - constants::LandingTarget[1];

    std::array<float , 3> velo = {rocket.Xvelocity , rocket.Yvelocity , rocket.Zvelocity};
    std::array<float,3> targetVector = {0,0,1};

    float stanley = vectorAngleBetween(velo , targetVector) + atanf(2.0f * (delta/rocket.getVelocity()));
    
    rocket.sumOfMoments[0] = stanley * 10000;
    

}



void landingBurn(Vehicle &rocket){

    if(rocket.reentry == false || rocket.glidePhase == false || rocket.Zposition > 4600) return;
        
    float vehicleNormalForce = -constants::gravitationalAcceleration * constants::mass;

    std::array<float,3> velo =  {rocket.Xvelocity, rocket.Yvelocity , rocket.Zvelocity};

    velo = normalizeVector(velo);

    velo[0] = velo[0] * 600000;

    velo[1] = velo[1] * 600000;

    velo[2] = velo[2] * 600000 + vehicleNormalForce;


    std::array<float , 3> directionVector = normalizeVector(velo);


    std::array<float,2> direction = {0,0};
        
    std::array<float,2> twoDState = {rocket.vehicleState[1] , rocket.vehicleState[2]};

    std::array<float,2> targetState = {directionVector[1] , directionVector[2]};


    float error = twodAngleDiffrence( twoDState, targetState); 

    
    rocket.error = error;

    rocket.twoDAngle = twoDState;
    
    
    float PIDOutputY = PID(0,error,rocket.gimbalYError, rocket.sumOFGimbalYError, constants::timeStep , 25 ,3 , 3);
    
    //std::cout<< twoDState[0] << "," << twoDState[1] << "  " << targetState[0] << "," << targetState[1]<< "   "<< error  << "   " << PIDOutputY <<std::endl;

    twoDState = {rocket.vehicleState[0] , rocket.vehicleState[2]};

    targetState = {directionVector[0] , directionVector[2]};

    error  = twodAngleDiffrence( twoDState, targetState);

    

    float PIDOutputX = PID(0,error,rocket.gimbalXError, rocket.sumOFGimbalXError, constants::timeStep , 25 ,3, 3);


    if(PIDOutputX > constants::maxGimbleAngle){
        PIDOutputX = constants::maxGimbleAngle;
    }else if(PIDOutputX < -constants::maxGimbleAngle){
        PIDOutputX = -constants::maxGimbleAngle;
    }

    if(PIDOutputY > constants::maxGimbleAngle){
        PIDOutputY = constants::maxGimbleAngle;
    }else if(PIDOutputY < -constants::maxGimbleAngle){
        PIDOutputY = -constants::maxGimbleAngle;
    }
    

    direction[0] = PIDOutputY;
    direction[1] = -PIDOutputX;


    rocket.applyEngineForce(direction , vectorMag(velo));
    


}




