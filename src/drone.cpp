#include "../include/drone.h"
#include "../include/battery.h"
#include <iostream>
#include <utility>
namespace SimCore{
droneBody::droneBody(){
    
}

void droneBody::init(string& motorConfig , string& batteryConfig){
    Vehicle::init();
    propLocationsSet = false;
    for(int i = 0 ; i < propLocations.size() ; i++){
        motors.push_back(std::make_unique<motor>(motorConfig));
    }
    droneBattery = std::make_unique<battery>(batteryConfig);
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

void droneBody::motorThrust(float motorRPM){

}
//a prop spinning creates a moment about the motor. This will be on spin up and staticlly 
void droneBody::motorMoment(){
    
}
void droneBody::thrustRequest(vector<float>& thrust){
    if(thrust.size() != propLocations.size()){
        throw std::runtime_error("Thrust Request does not match number of motors on drone body");
    }

}

//.first represents location .second is the given props force vector(normal vector).
//note location is relative to the vehicle center. 
std::pair<vector<array<float,3>>, vector<array<float,3>>>& droneBody::transposedProps(){

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