#include "../include/drone.h"
#include "../include/battery.h"
#include "../include/control.h"
#include "../include/toml.h"
#include "../include/PIDController.h"
#include "../include/aero.h"
#include "../include/propeller.h"
#include <iostream>
#include <utility>
#include <cmath>
namespace SimCore{
droneBody::droneBody(){
    
}

droneBody::~droneBody(){

}

void droneBody::init(string& motorConfig,string& batteryConfig , string& droneConfig){
    Vehicle::init(droneConfig);
    controller = make_unique<droneControl>();
    propLocationsSet = false;
    transposeCalls = 0;
    for(int i = 0 ; i < propellers.size() ; i++){
        motors.push_back(std::make_unique<motor>(motorConfig));
        motors[i]->init(motorConfig);
    }
    droneBattery = std::make_unique<battery>();
    droneBattery->init(batteryConfig);
    std::array<float,3> vect1 = {1,0,0};
    std::array<float,3> vect2 = {1,1,0};
    pose = std::make_unique<quaternionVehicle>(vect2,vect1);
    
}
//Set Square allows the creation of a rectagular prop profile.
//positive x = front , positive y = right, positive Z = top
// prop is a basis object location will be overwritten in setSquare
void droneBody::setSquare(float x, float y, propeller prop) {
    x = x / 2;
    y = y / 2;

    std::vector<std::unique_ptr<propeller>> props;

    std::array<std::array<float, 3>, 4> positions = {{
        { x,  y, 0},
        {-x,  y, 0},
        { x, -y, 0},
        {-x, -y, 0}
    }};

    for (const auto& pos : positions) {
        auto p = std::make_unique<propeller>(prop);
        p->location = pos;
        p->locationTransposed = pos;
        p->direction = {0, 0, 1};
        p->directionTransposed = {0, 0, 1};
        props.push_back(std::move(p));
    }
    //updates allocator with new motor layout
    allocatorHelper();
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
        if(std::abs(angle - M_PI) < EPSILON){
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
    for(int i = 0 ; i < propellers.size();i++){
        rotateVector(q,propellers[i]->locationTransposed);
        rotateVector(q,propellers[i]->directionTransposed);
    }
    rotateVector(q,index.dirVector);
    rotateVector(q,index.fwdVector);
    rotateVector(q,index.rightVector);
    rotateVector(q,cogLocationTranspose);
}

void droneBody::resetHelper(){
    indexCoordinates reset;
    index = reset;
    for(int i = 0 ; i < propellers.size() ; i++){
        propellers[i]->directionTransposed = propellers[i]->direction;
        propellers[i]->locationTransposed = propellers[i]->location;
    }
    cogLocationTranspose = cogLocation;
}

void droneBody::allocatorHelper(){
    vector<array<float,3>> pos;
    vector<array<float,3>> thrustVect;
    vector<float> coef;
    for(auto &props:propellers){
        pos.push_back(props->location);
        thrustVect.push_back(props->direction);
        coef.push_back(props->dragCoefficient);
    }
    controlAllocator allocate(pos,thrustVect,coef);
    controller->allocator = std::make_unique<controlAllocator>(allocate);
}




void droneBody::updateState(){
    updateController();
    for(int i = 0 ; i < motors.size();i++){
        motors[i]->update();
    }
    droneBattery->updateBattery();
    Vehicle::updateState();
    transposedProps();
}


droneControl::droneControl(){
}

void droneControl::init(){

}

void droneControl::initpidControl(string droneConfig , float timeStep){
    toml::tomlParse PIDPrase;
    PIDPrase.parseConfig(droneConfig,"PID");
    float kp = PIDPrase.floatValues["Kp"];
    float ki = PIDPrase.floatValues["Ki"];
    float kd = PIDPrase.floatValues["Kd"];
    if (PIDX == nullptr)
        PIDX = std::make_unique<PIDController>(kp, ki, kd, timeStep);

    if (PIDY == nullptr)
        PIDY = std::make_unique<PIDController>(kp, ki, kd, timeStep);

    if (PIDZ == nullptr)
        PIDZ = std::make_unique<PIDController>(
            PIDPrase.floatValues["ZKp"],
            PIDPrase.floatValues["ZKi"],
            PIDPrase.floatValues["ZKd"],
            timeStep);

    PIDX->setOutputLimits(-1,1);
    PIDY->setOutputLimits(-1,1);
    PIDZ->setOutputLimits(-1,1);
}

void droneControl::pidControl(std::array<float,3> pos){
    controlOutput = {   PIDX->update(pos[0]),
                        PIDY->update(pos[1]),
                        PIDZ->update(pos[2])};
}

void droneControl::setpidControl(float xTarget , float yTarget , float zTarget){
    PIDX->setTarget(xTarget);
    PIDY->setTarget(yTarget);
    PIDZ->setTarget(zTarget);
}
//Thrust model. Motor Rpm to thrust via a propellor model. 
//For now the simulation assumes that thrust is only a function of angular velocity of the motor.
//A more indepth model would have include the vechicle air velocity vector.
std::vector<float> droneBody::thrust(){
    std::vector <float> thrusts;
    for(int i  = 0 ; i < motors.size(); i++){
        //std::array<float,3> velo = this->getVelocityVector();
        thrusts.push_back(airDensity(getPositionVector()[2]) * pow(motors[i]->getCurrentAngularVelocity(),2) * pow(propellers[i]->diameter,4) * propellers[i]->thrustCoefficient);
    }
    return thrusts;
}




}//SimCore
