#include "../include/drone.h"
namespace SimCore{
droneBody::droneBody(){
    
}

void droneBody::init(){

}

void droneBody::motorThrust(){

}


droneControl::droneControl(){
    
}

void droneControl::init(){
    drone = std::make_unique<droneBody>();
}

}