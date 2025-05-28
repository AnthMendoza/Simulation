#include "../include/drone.h"
#include <iostream>
namespace SimCore{
droneBody::droneBody(){
    
}

void droneBody::init(){
    propLocationsSet = false;
}

void droneBody::setSquare(float x ,float y , float propellerMOI){
    if(propLocationsSet) return;
    propLocationsSet = true;
    x = x/2;
    y = y/2;
    //set in a local frame to the drone
    propLocations.push_back({x,y,0});
    propLocations.push_back({-x,y,0});
    propLocations.push_back({x,-y,0});
    propLocations.push_back({-x,-y,0});
    for(int i = 0 ; i < 4 ; i++){
        propMOI.push_back(propellerMOI);
        propForceVector.push_back({0,0,1});
    }
}

//adjust COG at the start of the simulation. Can be adjusted in sim.
void droneBody::offsetCOG(std::array<float ,3> offset){
    cogLocation = offset;
}

void droneBody::motorThrust(){

}
//a prop spinning creates a moment about the motor. This will be on spin up and staticlly 
void droneBody::motorMoment(){
    
}


droneControl::droneControl(){
    
}

void droneControl::init(){

}
//Create drone with characteristics required then move it intoi the control class.
void droneControl::addDrone(droneBody&& body){
    if(drone != nullptr) std::cerr<< "Warning drone already exists. Overwriting existing drone.";
    drone = std::make_unique<droneBody> (std::move(body));
}

}