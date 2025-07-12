#include "../../include/dynamics/drone.h"
#include "../../include/subsystems/battery.h"
#include "../../include/control/control.h"
#include "../../include/sim/toml.h"
#include "../../include/control/PIDController.h"
#include "../../include/dynamics/aero.h"
#include "../../include/subsystems/propeller.h"
#include "../../include/subsystems/motor.h"
#include "../../include/core/forceApplied.h"
#include "../../include/subsystems/motorDyno.h"
#include "../../include/dynamics/aero.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "sim/toml.h"
#include <cassert>
#include <iostream>
#include <utility>
#include <cmath>


namespace SimCore{
droneBody::droneBody(){
    
}

droneBody::~droneBody(){

}

void droneBody::initDrone(string& motorConfig,string& batteryConfig , string& droneConfig){
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
        p->directionTransposed = p->direction;
        propellers.push_back(std::move(p));
;    }
    //updates allocator with new motor layout
    allocatorHelper();
    dynoSystem();
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
    vector<float> coef = {  propellers[0]->powerCoefficient,
                            -propellers[0]->powerCoefficient,
                            propellers[0]->powerCoefficient,
                            -propellers[0]->powerCoefficient};
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
    //turbulantWind();
    //lift(aeroAreaDrone,coefOfLiftDrone);
    drag(aeroAreaDrone,coefOfDragDrone);
    float current = 0;
    float density = airDensity(Zposition);
    float totalThrust = 0;
    for(int i = 0 ; i < motors.size() ; i++){
        if(controllerThrusts.size() != motors.size()) throw runtime_error("Controller error thrust request does not equal motor count \n");
        
        float torqueLoad = propellers[i]->dragTorque(density,motors[i]->getCurrentAngularVelocity());
        float angularVelocityRequest = propellers[i]->desiredAngularVelocity(density,controllerThrusts[i]);
        if(angularVelocityRequest < 0) angularVelocityRequest = 0;
        motors[i]->updateMotorAngularVelocity(timeStep,torqueLoad,*droneBattery,angularVelocityRequest);
        current += abs(motors[i]->getCurrentCurrent());
        std::array<float,3> thrustVector = normalizeVector(propellers[i]->directionTransposed);
        float currentThrust = propellers[i]->thrustForce(density,motors[i]->getCurrentAngularVelocity());
        for(int i = 0 ; i < thrustVector.size();i++) thrustVector[i] = thrustVector[i] * currentThrust;
        totalThrust += currentThrust;
        addForce(thrustVector);
        auto leverArm = addVectors(cogLocationTranspose , propellers[i]->locationTransposed);
        auto force = forceToMoment(thrustVector,leverArm);
        addMoment(force);
        
    }
    std::cout<< "Total Thrust : "<< totalThrust<<"\n";
    droneBattery->updateBattery(current);
    Vehicle::updateState();
    //TransposedProps move the transposed cordinates of the props in the prop objects within propellers.
    transposedProps();
    //calls a step in iteration
    iterations++;

}
/// @brief Computes the moment (torque) produced by a propeller's thrust about the drone's center of gravity (COG).
/// @param prop Reference to the propeller object (non const because it modifies `direction`; consider redesign).
/// @param mot Constant reference to the motor object.
/// @param cogLocation Reference to a 3D array representing the center of gravity's location (x, y, z).
/// @param airDensity Reference to the current air density value.
/// @return A 3D vector representing the torque/moment produced by the propeller's thrust about the COG.
std::array<float,3> droneControl::thrustMoment(const propeller& prop ,const motor& mot, std::array<float,3> &cogLocation ,const float& airDensity){
    std::array<float,3> moments;
    std::array<float,3> torqueVector;
    torqueVector[0] = prop.location[0] - cogLocation[0];
    torqueVector[1] = prop.location[1] - cogLocation[1];
    torqueVector[2] = prop.location[2] - cogLocation[2];

    float  thrust = prop.thrustForce(airDensity,mot.getCurrentAngularVelocity());

    if(isZeroVector(prop.direction)) throw runtime_error("Prop Direction is a zero vector. In thrustMoment \n");
    std::array<float,3> direction = normalizeVector(prop.direction);


    std::array<float,3> thrustVector;
    thrustVector[0] = direction[0] * thrust;
    thrustVector[1] = direction[1] * thrust;
    thrustVector[2] = direction[2] * thrust;

    vectorCrossProduct(torqueVector,thrustVector,moments);
    

    return moments;
}

droneControl::droneControl(){
}


void droneControl::init(){

}


void droneControl::initpidControl(string droneConfig , float timeStep){
    toml::tomlParse PIDPrase;
    PIDPrase.parseConfig(droneConfig,"PID");

    //#########################################################################
    //Position control loop
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


    //#########################################################################
    //Velocity Control loop
    float kvp = PIDPrase.getFloat("Kvp");
    float kvi = PIDPrase.getFloat("Kvi");
    float kvd = PIDPrase.getFloat("Kvd");

    if (PIDVX == nullptr)
        PIDVX = std::make_unique<PIDController>(kvp, kvi, kvd, timeStep);

    if (PIDVY == nullptr)
        PIDVY = std::make_unique<PIDController>(kvp, kvi, kvd, timeStep);

    if (PIDVZ == nullptr)
        PIDVZ = std::make_unique<PIDController>(
            PIDPrase.getFloat("ZKvp"),
            PIDPrase.getFloat("ZKvi"),
            PIDPrase.getFloat("ZKvd"),
            timeStep);
    toml::tomlParse vParse;
    vParse.parseConfig(droneConfig,"vehicle");
    float maxCruiseVelocity = vParse.getFloat("maxCruiseVelocity");
    // OutputLimits: Enforce a circular boundary instead of clamping x and y independently,
    // ensuring the (x, y) pair remains within a circle rather than forming a square region.
    PIDVX->setOutputLimits(-maxCruiseVelocity,maxCruiseVelocity);
    PIDVY->setOutputLimits(-maxCruiseVelocity,maxCruiseVelocity);
    // m/s need to find an elligant way to calculate this limit
    //currently hardcoded in 
    PIDVZ->setOutputLimits(-1,1);

    //#########################################################################
    //Angular control loop
    float Akp = PIDPrase.getFloat("AKp");
    float Aki = PIDPrase.getFloat("AKi");
    float Akd = PIDPrase.getFloat("AKd");

    if (APID == nullptr)
        APID = std::make_unique<PIDController>(Akp, Aki, Akd, timeStep);


    APID->setOutputLimits(-1,1);

    
}

// Cascading PID Controller:
// Implements a multi-layer PID control structure for drone navigation to waypoints.
// The controller is organized into nested control loops:
// - Position PID computes desired velocity based on position error
// - Velocity PID computes desired acceleration or thrust based on velocity error
// - Attitude and Rate PIDs can follow to control orientation and angular velocity
// Used in flight control systems like PX4 and ArduPilot.
std::array<float,3> droneControl::pidControl(const std::array<float,3> pos ,std::array<float,3> velo , const std::array<float,3> state, const float& maxAngleAOT){
    //wayPoint should be set prior to call via setPidControl();
    controlOutputVelocity = {   PIDX->update(pos[0]),
                                PIDY->update(pos[1]),
                                PIDZ->update(pos[2])};
    //Contorl outPut for velocity will take the output command for 
    PIDVX->setTarget(controlOutputVelocity[0]);
    PIDVY->setTarget(controlOutputVelocity[1]);
    PIDVZ->setTarget(controlOutputVelocity[2]);

    controlOutput = {   PIDVX->update(velo[0]),
                        PIDVY->update(velo[1]),
                        PIDVZ->update(velo[2])};

    std::pair<std::array<float,3>,float> momentAxis = aot(maxAngleAOT,state);

    return axisMomentToEularMoment(momentAxis.first,momentAxis.second);
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
//Takes the PIDX and PIDY values and creates a desired vector that is within the vehicles thrust profile. 
// maxAngle is calculated using the dyno numbers when the battery is at full charge with minimal voltage sag
//the desired Vector is located in the array named currentFlightTargetNormal.
std::pair<std::array<float,3> , float> droneControl::aot(float maxAngleAOT , std::array<float,3> currentState){

    float xAngle = maxAngleAOT * controlOutput[0];
    float yAngle = maxAngleAOT * controlOutput[1];
    desiredNormal = {0,0,1};
    Quaternion quantX = fromAxisAngle({0,1,0}, xAngle);
    Quaternion quantY = fromAxisAngle({1,0,0} , yAngle);
    Quaternion combined = quantX * quantY;
    std::array<float,3> adjustedVect = rotateVector(combined,desiredNormal);


    return aotControl(adjustedVect,currentState);

}

std::pair<std::array<float,3> , float> droneControl::aotControl(std::array<float,3> desiredNormal,std::array<float,3> currentState){
    currentFlightTargetNormal = desiredNormal;
    //Drone
    //Create Torque around adjust vector which was used a pivot between disired and current angle.
    float angleBetween = vectorAngleBetween(currentFlightTargetNormal,currentState); 
    std::array<float,3> rotationAxis;
    vectorCrossProduct(currentFlightTargetNormal,currentState,rotationAxis); 
    // set target is always 0 because the currentState and targetNormal ideally should have a 0 degree angleBetween
    APID->setTarget(0);
    float output = APID->update(angleBetween);
    std::pair<array<float,3>,float> result = {rotationAxis,output};

    return result;

}

/**
 * @brief Computes the drone's control output based on state estimation.
 * @param estimatedPostion       Estimated 3D position of the drone(m).
 * @param estimatedState         Estimated orientation/state (direction vector normal).
 * @param estimatedVelocity      Estimated 3D velocity of the drone(m/s).
 * @param mass                   Mass of the drone (in kilograms).
 * @param gravitationalAcceleration  Acceleration due to gravity (-9.81 m/s^2).
 * @param thrustLimit            Maximum allowable thrust.
 * @param maxAngleOfAttack       Maximum allowable angle of attack (in radians).
 * @return std::vector<float>    Thrust commands
 * 
 * This function encapsulates the drone's core control logic.
 */

vector<float> droneControl::update(const std::array<float,3>& estimatedPostion,const std::array<float,3>& estimatedState,const std::array<float,3>& estimatedVelocity ,const float mass,const float gravitationalAcceleration, float thrustLimit , float maxAngleOfAttack){
    std::array<float,3> moments =  pidControl(estimatedPostion, estimatedVelocity , estimatedState,maxAngleOfAttack);
    //change this asap. thrustCoeff used as test metric
    std::array<float,3> gravityNormal = {0,0,1};
    float angleFromGravity = vectorAngleBetween(gravityNormal,estimatedState);
    std::cout<<"requested:"<< static_cast<float>((abs(gravitationalAcceleration) + controlOutput[2]) * mass * cosf(angleFromGravity))<< "\n";
    VectorXd desired  = allocator->toVectorXd({0.0f,0.0f,static_cast<float>((abs(gravitationalAcceleration) + controlOutput[2]) * mass * cosf(angleFromGravity)),moments[0],moments[1],moments[2]});
    VectorXd thrusts = allocator->allocate(desired);
    VectorXd output = allocator->computeWrench(thrusts);
    /*std::cout << "moment: ["
          << estimatedState[0] << ", "
          << estimatedState[1] << ", "
          << estimatedState[2] << ", " << std::endl;
    std::cout << "Wrench Output: ["
          << output[0] << ", "
          << output[1] << ", "
          << output[2] << ", "
          << output[3] << ", "
          << output[4] << ", "
          << output[5] << "]" << std::endl;*/

    //VectorXd to vector<float> can be converted into a method if needed further
    std::vector<float> computedThrust;
    computedThrust.reserve(thrusts.size());
    for (int i = 0; i < thrusts.size(); ++i) {
        computedThrust.push_back(static_cast<float>(thrusts[i]));
    }
    return computedThrust;
}




//total thrust limit used in PID control loop. Can also be used in more advance control loops with constraints
void droneBody::dynoSystem(){
    totalThrustLimit = 0 ;

    for(int i = 0 ;i < motors.size() ; i++){
        auto limits = thrustLimits(*motors[i],*propellers[i],*droneBattery,timeStep);
        propellers[i]->thrustLimits = limits;
        totalThrustLimit += limits.second;

    }
    maxAngleAOT = acos(mass/totalThrustLimit);
}

droneBody::droneBody(const droneBody& other)
    : Vehicle(other),  // Call base class copy constructor
      thrustRequestVect(other.thrustRequestVect),
      propLocationsSet(other.propLocationsSet),
      transposeCalls(other.transposeCalls),
      totalThrustLimit(other.totalThrustLimit),
      maxAngleAOT(other.maxAngleAOT),
      cogLocation(other.cogLocation),
      cogLocationTranspose(other.cogLocationTranspose),
      index(other.index),
      droneConfig(other.droneConfig)
{
    //Deep Copies
    for (const auto& mot : other.motors) {
        motors.push_back(std::make_unique<motor>(*mot));
    }

    for (const auto& prop : other.propellers) {
        propellers.push_back(std::make_unique<propeller>(*prop));
    }

    if (other.droneBattery) {
        droneBattery = std::make_unique<battery>(*other.droneBattery);
    }

    if (other.pose) {
        pose = std::make_unique<quaternionVehicle>(*other.pose);
    }

    if (other.controller) {
        controller = std::make_unique<droneControl>(*other.controller);
    }
}

droneControl::droneControl(const droneControl& other)
    : controlOutput(other.controlOutput),
      controlOutputVelocity(other.controlOutputVelocity),
      desiredNormal(other.desiredNormal),
      currentFlightTargetNormal(other.currentFlightTargetNormal),
      aotVect(other.aotVect)
{
    if (other.PIDX)  PIDX  = std::make_unique<PIDController>(*other.PIDX);
    if (other.PIDY)  PIDY  = std::make_unique<PIDController>(*other.PIDY);
    if (other.PIDZ)  PIDZ  = std::make_unique<PIDController>(*other.PIDZ);
    
    if (other.PIDVX) PIDVX = std::make_unique<PIDController>(*other.PIDVX);
    if (other.PIDVY) PIDVY = std::make_unique<PIDController>(*other.PIDVY);
    if (other.PIDVZ) PIDVZ = std::make_unique<PIDController>(*other.PIDVZ);
    
    if (other.APID)  APID  = std::make_unique<PIDController>(*other.APID);
    
    if (other.allocator) allocator = std::make_unique<controlAllocator>(*other.allocator);
}




}//SimCore
