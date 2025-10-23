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
#include "../../include/utility/utility.h"
#include "../../include/control/droneControllerBase.h"
#include "../../include/thirdparty/eigenWrapper.h"
#include "sim/toml.h"
#include "../../include/utility/logger.h"
#include "../../include/core/coordinateSystem.h"
#include <cassert>
#include <iostream>
#include <utility>
#include <cmath>
#include <stdexcept>


namespace SimCore{



droneBody::~droneBody(){

}

void droneBody::initDrone(string& droneConfig){
    Vehicle::init(droneConfig);
    configFile = droneConfig;
    transposeCalls = 0;
    pose = std::make_unique<quaternionVehicle>();
    cogLocation = {0,0,0};
    cogLocationTranspose = cogLocation;

    sensors = std::make_unique<droneSensorSuite>(droneConfig);
}


//adjust COG at the start of the simulation. Can be adjusted in sim.
void droneBody::offsetCOG(std::array<float ,3> offset){
    cogLocation = offset;
}

//note location is relative to the vehicle center. 
void droneBody::transposedProps(const Quaternion& quant){
    //rezero transpose every 100 timeSteps
    if(transposeCalls >= 100){
        for(auto& p:propellers){
            p->directionTransposed= normalizeVector(p->directionTransposed);
        }
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

            combined = combined.normalized();
            rotationHelper(combined);
            angle = vectorAngleBetween(quaternionDirectionVector,index.dirVector);
        
        }
        std::array<float,3> cross;
        
        vectorCrossProduct(quaternionDirectionVector, index.dirVector, cross);
        Quaternion qcross = fromAxisAngle(cross, angle);
        rotationHelper(qcross);

        float yawAngle = vectorAngleBetween(index.fwdVector,pose->getfwdVector());

    }else{
        
        rotationHelper(quant);
    }
}

void droneBody::rotationHelper(const Quaternion& q){
    for(int i = 0 ; i < propellers.size();i++){
        propellers[i]->locationTransposed = rotateVector(q,propellers[i]->locationTransposed);
        resizeVectorInPlace(propellers[i]->locationTransposed,propellers[i]->locationVectorLength);
        propellers[i]->directionTransposed = rotateVector(q,propellers[i]->directionTransposed);
        normalizeVectorInPlace(propellers[i]->directionTransposed);
    }
    index.dirVector = rotateVector(q,index.dirVector);
    index.fwdVector = rotateVector(q,index.fwdVector);
    index.rightVector = rotateVector(q,index.rightVector);
    cogLocationTranspose = rotateVector(q,cogLocationTranspose);
}

void droneBody::resetHelper(){
    indexCoordinates reset;
    index = reset;
    for(int i = 0 ; i < propellers.size() ; i++){
        propellers[i]->directionTransposed = propellers[i]->direction;
        propellers[i]->locationTransposed = propellers[i]->getLocation();
    }
    cogLocationTranspose = cogLocation;
}

void droneBody::allocatorHelper(){ 
    vector<array<float,3>> pos;
    vector<array<float,3>> thrustVect;
    vector<float> coef;
    for(int i = 0 ; i < propellers.size();i++){
        pos.push_back(propellers[i]->getLocation());
        thrustVect.push_back(propellers[i]->direction);
        coef.push_back(propellers[i]->powerCoefficient * static_cast<float>(propellers[i]->rotationDirection));
    }
}


void droneBody::rotateLocalEntities(const Quaternion& quant){
    Vehicle::rotateLocalEntities(quant);
    transposedProps(quant);

}


void droneBody::updateState(float time,std::optional<controlPacks::variantPackets> controlInput){
    vector<float> controllerThrusts;
    if(controlInput){
        controlPacks::variantPackets& packet = controlInput.value();
        auto* controlPack = std::get_if<controlPacks::motorOnlyPacket>(&packet);
        controllerThrusts = controlPack->thrust;
    }
    
    //turbulantWind();
    //lift(aeroAreaDrone,coefOfLiftDrone);
    //drag(aeroAreaDrone,coefOfDragDrone);
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
        auto moment = forceToMoment(thrustVector,leverArm);

        addMoment(moment);
    }

    droneBattery->updateBattery(current,getTime());
    Vehicle::updateState(time);
    //TransposedProps move the transposed cordinates of the props in the prop objects within propellers.



    dataLog();
    setGlobals();

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




//total thrust limit used in PID control loop. Can also be used in more advance control loops with constraints
void droneBody::dynoSystem(){
    totalThrustLimit = 0 ;
    for(int i = 0 ;i < motors.size() ; i++){
        motor testMotor(*motors[i]);
        propeller testPropeller(*propellers[i]);
        battery testBattery(*droneBattery);
        testMotor.resetMotor();
        float simTimeStep = 0.001f;
        auto limits = thrustLimits(testMotor,testPropeller,testBattery, simTimeStep);
        testPropeller.thrustLimits = limits;
        totalThrustLimit += limits.second;

    }
    if(mass <= 0.0f) throw std::runtime_error("\nMass cannot <= 0\n");
    if(totalThrustLimit <= 0.0f ) throw std::runtime_error("\ntotalThrustLimit cannot <= 0\n");
}


droneBody::droneBody(const droneBody& other)
    : Vehicle(other), 
      thrustRequestVect(other.thrustRequestVect),
      transposeCalls(other.transposeCalls),
      totalThrustLimit(other.totalThrustLimit),
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

}


void droneBody::setEntitiesPose(const poseState& pose) {

    poseState basisPose = CoordinateSystem::WORLD_BASIS;
    vehicleReferenceFrame basisReference(pose,basisPose);

    for(auto &prop:propellers){
        prop->locationTransposed = prop->getLocation();
        prop->directionTransposed = prop->direction;
        basisReference.realign(prop->locationTransposed);
        resizeVectorInPlace(prop->locationTransposed,prop->locationVectorLength);
        basisReference.realign(prop->directionTransposed);
    }
}



void droneBody::dataLog(){
    auto pA = getPositionVector();
    LOG_VEC3("PositionActual",getTime(),pA[0],pA[1],pA[2]);
    auto vA = getVelocityVector();
    LOG_VEC3("VelocityActual",getTime(),vA[0],vA[1],vA[2]);
}





}//SimCore
