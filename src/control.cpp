
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


    if(rocket.Zposition < 70000 && rocket.reentry == false){

        rocket.reentry = true;
        float currentMaxGForce = constants::maxGAllowedEntry + 1;
        
        
        //main loop moves forward 1 second every lookAHead cylce
        float stepInterval = 1;
        int count = 0;
        //std::cout<< "currentMaxGForce : "<< currentMaxGForce << " maxAllowedGForce : "<< maxAllowedGForce << std::endl;
        while(currentMaxGForce > constants::maxGAllowedEntry){
            Vehicle lookAheadRocket = rocket;
            std::vector<float> gForces = lookAhead(lookAheadRocket , 105 , [](Vehicle &r) { return r.gForce; }); 
            currentMaxGForce = *std::max_element(gForces.begin(),gForces.end());
            //std::cout<< "lookAhead count : "<< count << std::endl;
            //std::cout<< "gforce : "<< currentMaxGForce << std::endl;
            if(currentMaxGForce < constants::maxGAllowedEntry) return;
            
            count++;
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



void glidToTarget(){

}
