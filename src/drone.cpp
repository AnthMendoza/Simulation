#include "../include/drone.h"
#include "../include/battery.h"
#include "../include/control.h"
#include "../include/toml.h"
#include "../include/PIDController.h"
#include "../include/aero.h"
#include "../include/propeller.h"
#include "../include/motor.h"
#include "../include/Eigen/Eigen"
#include "../include/Eigen/Dense"
#include "toml.h"
#include <cassert>
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
    configFile = droneConfig;
    initSensors();
    controller = make_unique<droneControl>();
    propLocationsSet = false;
    transposeCalls = 0;
    for(int i = 0 ; i < propellers.size() ; i++){
        motors.push_back(std::make_unique<motor>(motorConfig,timeStep));
        motors[i]->init(motorConfig,timeStep);
    }
    droneBattery = std::make_unique<battery>(batteryConfig);
    std::array<float,3> vect1 = {1,0,0};
    std::array<float,3> vect2 = {1,1,0};
    pose = std::make_unique<quaternionVehicle>(vect2,vect1);
    cogLocation = {0,0,0};
    cogLocationTranspose = cogLocation;
    toml::tomlParse toml;
    toml.parseConfig(droneConfig,"vehicle");
    mass = toml.getFloat("mass");
    MOI[0] = toml.getArray("MOI")[0];
    MOI[1] = toml.getArray("MOI")[1];
    MOI[2] = toml.getArray("MOI")[2];
}
//Set Square allows the creation of a rectagular prop profile.
//positive x = front , positive y = right, positive Z = top
// prop is a basis object location will be overwritten in setSquare
void droneBody::setSquare(float x, float y, propeller& prop, motor& mot) {
    if(x <= 0 || y <= 0) throw runtime_error("X and Y values of SetSquare cannot be <=0 \n");
    x = x / 2;
    y = y / 2;


    std::array<std::array<float, 3>, 4> positions = {{
        { x,  y, 0},
        {-x,  y, 0},
        { x, -y, 0},
        {-x, -y, 0}
    }};

    for (const auto& pos : positions) {
        motors.push_back(std::make_unique<motor>(mot));
        auto p = std::make_unique<propeller>(prop);
        p->location = pos;
        p->locationTransposed = pos;
        p->direction = {0, 0, 1};
        p->directionTransposed = {0, 0, 1};
        propellers.push_back(std::move(p));
;    }
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

//note location is relative to the vehicle center. 
void droneBody::transposedProps(){
    //rezero transpose every 100 timeSteps
    if(transposeCalls >= 100){
        for(auto& p:propellers){
            p->directionTransposed= normalizeVector(p->directionTransposed);
        }
        transposeCalls = 0;
        //set direction axis to main axis
        std::array<float,3> quaternionDirectionVector = pose->getdirVector();

        float angle = vectorAngleBetween(quaternionDirectionVector,index.nominal.dir);
        assert(!std::isnan(angle));
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
            assert(!std::isnan(angle));
        }
        std::array<float,3> cross;
        assert(!std::isnan(cross[0]));
        vectorCrossProduct(quaternionDirectionVector, index.dirVector, cross);
        Quaternion qcross = fromAxisAngle(cross, angle);
        rotationHelper(qcross);

        float yawAngle = vectorAngleBetween(index.fwdVector,pose->getfwdVector());
        assert(!std::isnan(yawAngle));
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
    vector<float> coef = {  propellers[0]->dragCoefficient,
                            -propellers[0]->dragCoefficient,
                            propellers[0]->dragCoefficient,
                            -propellers[0]->dragCoefficient};
    for(int i = 0 ; i < propellers.size();i++){
        pos.push_back(propellers[i]->location);
        thrustVect.push_back(propellers[i]->direction);
        //coef.push_back(propellers[i]->dragCoefficient);
    }
    controlAllocator allocate(pos,thrustVect,coef);
    controller->allocator = std::make_unique<controlAllocator>(allocate);
}




void droneBody::updateState(){
    vector<float> controllerThrusts = updateController();
    lift(aeroAreaDrone,coefOfLiftDrone);
    drag(aeroAreaDrone,coefOfDragDrone);
    float current = 0;
    float density = airDensity(Zposition);
    std::cout<<Zposition<< "," <<getEstimatedPosition()[2]<< "\n";
    for(int i = 0 ; i < motors.size() ; i++){
        if(controllerThrusts.size() != motors.size()) throw runtime_error("Controller error thrust request does not equal motor count \n");
        
        float torqueLoad = propellers[i]->dragTorque(density,motors[i]->getCurrentAngularVelocity());
        float angularVelocityRequest = propellers[i]->desiredAngularVelocity(density,controllerThrusts[i]);
        if(angularVelocityRequest < 0) angularVelocityRequest = 0;
        std::cout<<angularVelocityRequest;
        motors[i]->updateMotorAngularVelocity(timeStep,torqueLoad,*droneBattery,angularVelocityRequest);
        current += abs(motors[i]->getCurrentCurrent());
        std::array<float,3> thrustVector = propellers[i]->directionTransposed;
        float currentThrust = propellers[i]->thrustForce(density,motors[i]->getCurrentAngularVelocity());
        thrustVector[0] = thrustVector[0] * currentThrust;
        thrustVector[1] = thrustVector[1] * currentThrust;
        thrustVector[2] = thrustVector[2] * currentThrust;
        
        //std::cout<<"thrust"<<thrustVector[0]<<","<< thrustVector[1]<<","<<thrustVector[2]<<"\n"; 
        addForce(thrustVector);
    }
    assert(!std::isnan(current));
    droneBattery->updateBattery(current);
    Vehicle::updateState();
    //TransposedProps move the transposed cordinates of the props in the prop objects within propellers.
    transposedProps();
    //calls a step in iteration
    iterations++;
}


droneControl::droneControl(){
}

void droneControl::init(){

}

void droneControl::initpidControl(string droneConfig , float timeStep){
    toml::tomlParse PIDPrase;
    PIDPrase.parseConfig(droneConfig,"PID");
    float kp = PIDPrase.getFloat("Kp");
    float ki = PIDPrase.getFloat("Ki");
    float kd = PIDPrase.getFloat("Kd");
    if (PIDX == nullptr)
        PIDX = std::make_unique<PIDController>(kp, ki, kd, timeStep);

    if (PIDY == nullptr)
        PIDY = std::make_unique<PIDController>(kp, ki, kd, timeStep);

    if (PIDZ == nullptr)
        PIDZ = std::make_unique<PIDController>(
            PIDPrase.getFloat("ZKp"),
            PIDPrase.getFloat("ZKi"),
            PIDPrase.getFloat("ZKd"),
            timeStep);

    PIDX->setOutputLimits(-1,1);
    PIDY->setOutputLimits(-1,1);
    PIDZ->setOutputLimits(-1,1);

    if (APIDX == nullptr)
        APIDX = std::make_unique<PIDController>(kp, ki, kd, timeStep);

    if (APIDY == nullptr)
        APIDY = std::make_unique<PIDController>(kp, ki, kd, timeStep);


    APIDX->setOutputLimits(-1,1);
    APIDY->setOutputLimits(-1,1);

    
}

void droneControl::pidControl(std::array<float,3> pos){
    controlOutput = {   PIDX->update(pos[0]),
                        PIDY->update(pos[1]),
                        PIDZ->update(pos[2])};
    std::cout<<PIDZ->lastError()<<","<<PIDZ->getTarget()<<","<<PIDZ->getPreviousSample()<< ","<< PIDZ->getTarget()<<","<< PIDZ->getPreviousError()<<","<<controlOutput[2]<<"\n";
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
void droneControl::forceMomentProfile(){
    
}
//current state this is very temporary
void droneControl::aot(){
    float maxAngle = .7;
    float xAngle = maxAngle * controlOutput[0];
    float yAngle = maxAngle * controlOutput[1];
    desiredNormal = {0,0,1};
    Quaternion quantX = fromAxisAngle( {1,0,0}, xAngle);
    Quaternion quantY = fromAxisAngle({0,1,0} , yAngle);
    Quaternion combined = quantX * quantY;
    std::array<float,3> adjustedVect = rotateVector(combined,desiredNormal);
    currentFlightTargetNormal = adjustedVect;
}

 vector<float> droneControl::update(std::array<float,3> estimatedPostion,std::array<float,3> estimatedState,std::array<float,3> estimatedVelocity , float mass, float gravitationalAcceleration){
    pidControl(estimatedPostion);
    //change this asap. thrustCoeff used as test metric
    float thrustCoeff = 400;
    assert(allocator != nullptr);
    VectorXd desired  = allocator->toVectorXd({0.0f,0.0f,controlOutput[2] * thrustCoeff,0.0f,0.0f,0.0f});
    VectorXd thrusts = allocator->allocate(desired);
    //VectorXd to vector<float> can be converted into a method if needed further
    std::vector<float> result;
    result.reserve(thrusts.size());
    for (int i = 0; i < thrusts.size(); ++i) {
        result.push_back(static_cast<float>(thrusts[i]));
    }
    return result;
}


}//SimCore
