#include "../include/drone.h"
#include "../include/battery.h"
#include <iostream>
#include <utility>
namespace SimCore{
droneBody::droneBody(){
    
}

droneBody::~droneBody(){

}

void droneBody::init(string& motorConfig){
    Vehicle::init();

    propLocationsSet = false;
    transposeCalls = 0;
    for(int i = 0 ; i < propLocations.size() ; i++){
        motors.push_back(std::make_unique<motor>());
        motors[i]->init(motorConfig);
    }
    droneBattery = std::make_unique<battery>();
    std::array<float,3> vect1 = {1,0,0};
    std::array<float,3> vect2 = {1,1,0};
    pose = std::make_unique<quaternionVehicle>(vect2,vect1);
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
void droneBody::transposedProps(){
    //rezero transpose every 100 timeSteps
    if(transposeCalls >= 100){
        transposeCalls = 0;
        //set direction axis to main axis
        std::array<float,3> quaternionDirectionVector = pose->getdirVector();

        float angle = vectorAngleBetween(quaternionDirectionVector,index.nominal.dir);
        const float EPSILON = 1e-4f;
        if(std::abs(angle -M_PI) < EPSILON){
            resetHelper();
            float adjust = .1;
            Quaternion qx = fromAxisAngle({1,0,0}, adjust);
            Quaternion qy = fromAxisAngle({0,1,0}, adjust);
            Quaternion qz = fromAxisAngle({0,0,1}, adjust);

            Quaternion combined = qx * qy * qz;

            combined.normalized();
            rotationHelper(combined);
            angle = vectorAngleBetween(quaternionDirectionVector,index.dirVector);
        }
        std::array<float,3> cross;
        vectorCrossProduct(quaternionDirectionVector, index.dirVector, cross);
        Quaternion qcross = fromAxisAngle(cross, angle);
        rotationHelper(qcross);

        float yawAngle = vectorAngleBetween(index.fwdVector,pose->getfwdVector());
    }
}

void droneBody::rotationHelper(Quaternion& q){
    rotateMultiVectors(q,propLocationsTranspose);
    rotateMultiVectors(q,propForceVectorTranspose);
    rotateVector(q,index.dirVector);
    rotateVector(q,index.fwdVector);
    rotateVector(q,index.rightVector);
    rotateVector(q,cogLocationTranspose);
}

void droneBody::resetHelper(){
    indexCoordinates reset;
    index = reset;
    propLocationsTranspose = propLocations;
    propForceVectorTranspose = propForceVector;
    cogLocationTranspose = cogLocation;
}




void droneBody::updateState(){
    for(int i = 0 ; i < motors.size();i++){
        motors[i]->update();
    }
    droneBattery->updateBattery();
    Vehicle::updateState();
}


droneControl::droneControl(){
}

void droneControl::init(){
    body = make_unique<droneBody>();
}

}//SimCore
