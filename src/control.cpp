
#include <vector>
#include <functional>
#include <algorithm>
#include <iostream>
#include "../include/vehicle.h"
#include "../include/constants.h"
#include "../include/logs.h"



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




void reentryBurn(Vehicle &rocket){
    
    if(rocket.reentry == true) return;


    if(rocket.Zposition < 55000 && rocket.reentry == false){

        rocket.reentry = true;
        float currentMaxGForce = constants::maxGAllowedEntry + 1;
        //main loop moves forward 1 second every lookAHead cylce
        float stepInterval = 1;
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
                std::array<float,2> direction = {0,0};
                rocket.applyEngineForce(direction , constants::maxThrust * .8 * 3); 
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
    //      vehicle vertical is not needed but would be nice


    
    if(rocket.reentry == false) return;

    // a small lookAhead to see drift
    float deltaX = 0 ,deltaY = 0;

    Vehicle lookAheadRocket = rocket;

    

    while(true){
        
    }
    
    

}



void landingBurn(Vehicle &rocket){

    if(rocket.reentry == false || rocket.glidePhase == false) return;

    while(true){

    }

}
