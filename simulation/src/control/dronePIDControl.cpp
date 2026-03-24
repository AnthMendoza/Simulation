#include "../../include/control/dronePIDControl.h"
#include "../../include/core/coordinateSystem.h"
#include "../../include/control/stateInfo.h"
#include "../../include/utility/utility.h"
#include <algorithm>
#include <iostream>
#include <cmath>


namespace SimCore{


PIDDroneController::PIDDroneController(float frequency):droneControllerBase(frequency){

}


PIDDroneController::PIDDroneController(const PIDDroneController& other):
    droneControllerBase(other),
    mass(other.mass),
    maxAcceleration(other.maxAcceleration),
    gravitationalAcceleration(other.gravitationalAcceleration)
    {
    firstPose = true;
    if(other.manager) manager = std::make_unique<timeManager>(*other.manager);
    if (other.PIDX)  PIDX  = std::make_unique<PIDController>(*other.PIDX);
    if (other.PIDY)  PIDY  = std::make_unique<PIDController>(*other.PIDY);
    if (other.PIDZ)  PIDZ  = std::make_unique<PIDController>(*other.PIDZ);
    
    if (other.PIDVX) PIDVX = std::make_unique<PIDController>(*other.PIDVX);
    if (other.PIDVY) PIDVY = std::make_unique<PIDController>(*other.PIDVY);
    if (other.PIDVZ) PIDVZ = std::make_unique<PIDController>(*other.PIDVZ);

    if (other.APIDVX)  APIDVX  = std::make_unique<PIDController>(*other.APIDVX);
    if (other.APIDVY)  APIDVY  = std::make_unique<PIDController>(*other.APIDVY);
    
    if (other.APIDX)  APIDX  = std::make_unique<PIDController>(*other.APIDX);
    if (other.APIDY)  APIDY  = std::make_unique<PIDController>(*other.APIDY);
}




void PIDDroneController::initController(std::string droneConfig) {
    initVehicleParams(droneConfig);
    initPositionLoop(droneConfig);
    initVelocityLoop(droneConfig);
    initAngularVelocityLoop(droneConfig);
    initAngularPositionLoop(droneConfig);
    updateCalculatedValues();
}

void PIDDroneController::initVehicleParams(const std::string& droneConfig) {
    toml::tomlParse parse;
    parse.parseConfig(droneConfig, "vehicle");
    mass = parse.getFloat("mass");
    gravitationalAcceleration = parse.getFloat("gravitationalAcceleration");
}

void PIDDroneController::initPositionLoop(const std::string& droneConfig) {
    toml::tomlParse parse;
    parse.parseConfig(droneConfig, "PID");

    float kp = parse.getFloat("Kp");
    float ki = parse.getFloat("Ki");
    float kd = parse.getFloat("Kd");

    if (!PIDX) PIDX = std::make_unique<PIDController>(kp, ki, kd);
    if (!PIDY) PIDY = std::make_unique<PIDController>(kp, ki, kd);
    if (!PIDZ) PIDZ = std::make_unique<PIDController>(
        parse.getFloat("ZKp"),
        parse.getFloat("ZKi"),
        parse.getFloat("ZKd")
    );

    toml::tomlParse vParse;
    vParse.parseConfig(droneConfig, "vehicle");
    float maxCruiseVelocity = vParse.getFloat("maxCruiseVelocity");

    PIDX->setOutputLimits(-maxCruiseVelocity, maxCruiseVelocity);
    PIDY->setOutputLimits(-maxCruiseVelocity, maxCruiseVelocity);
    PIDZ->setOutputLimits(-10, 10);
}

void PIDDroneController::initVelocityLoop(const std::string& droneConfig) {
    toml::tomlParse parse;
    parse.parseConfig(droneConfig, "PID");

    float kvp = parse.getFloat("Kvp");
    float kvi = parse.getFloat("Kvi");
    float kvd = parse.getFloat("Kvd");

    if (!PIDVX) PIDVX = std::make_unique<PIDController>(kvp, kvi, kvd);
    if (!PIDVY) PIDVY = std::make_unique<PIDController>(kvp, kvi, kvd);
    if (!PIDVZ) PIDVZ = std::make_unique<PIDController>(
        parse.getFloat("ZKvp"),
        parse.getFloat("ZKvi"),
        parse.getFloat("ZKvd")
    );
}

void PIDDroneController::initAngularVelocityLoop(const std::string& droneConfig) {
    toml::tomlParse parse;
    parse.parseConfig(droneConfig, "PID");

    float kp = parse.getFloat("AVKp");
    float ki = parse.getFloat("AVKi");
    float kd = parse.getFloat("AVKd");

    if (!APIDVX) APIDVX = std::make_unique<PIDController>(kp, ki, kd);
    if (!APIDVY) APIDVY = std::make_unique<PIDController>(kp, ki, kd);

    toml::tomlParse vParse;
    vParse.parseConfig(droneConfig, "vehicle");
    auto MOI = vParse.getArray("MOI");

    constexpr float MAX_ROT_ACCEL = 50.0f;
    float MAX_MOMENT = MOI[0] * MAX_ROT_ACCEL;

    APIDVX->setOutputLimits(-MAX_MOMENT, MAX_MOMENT);
    APIDVY->setOutputLimits(-MAX_MOMENT, MAX_MOMENT);
    APIDVX->setIntegralClamp(ki * 10);
    APIDVY->setIntegralClamp(ki * 10);
}

void PIDDroneController::initAngularPositionLoop(const std::string& droneConfig) {
    toml::tomlParse parse;
    parse.parseConfig(droneConfig, "PID");

    float kp = parse.getFloat("AKp");
    float ki = parse.getFloat("AKi");
    float kd = parse.getFloat("AKd");

    if (!APIDX) APIDX = std::make_unique<PIDController>(kp, ki, kd);
    if (!APIDY) APIDY = std::make_unique<PIDController>(kp, ki, kd);

    constexpr float MAX_ROT_VELOCITY = 18.85f;
    APIDX->setOutputLimits(-MAX_ROT_VELOCITY, MAX_ROT_VELOCITY);
    APIDY->setOutputLimits(-MAX_ROT_VELOCITY, MAX_ROT_VELOCITY);

    APIDX->setIntegralClamp(ki * 10);
    APIDY->setIntegralClamp(ki * 10);
}



void PIDDroneController::updateCalculatedValues(){
    if(mass <= 0){
        throw std::runtime_error("Mass Cannot be <= 0.");
    }
    maxAcceleration = 10.0f;
    PIDVX->setOutputLimits(-maxAcceleration,maxAcceleration);
    PIDVY->setOutputLimits(-maxAcceleration,maxAcceleration);
    PIDVZ->setOutputLimits(gravitationalAcceleration,maxAcceleration + abs(gravitationalAcceleration));
}

static constexpr float MINIMUM_THROTTLE_AT_IDLE = 0.1;
static constexpr float MINIMUM_ACCELERATION_Z = 0.3;

//flight envolope. zaccel is dominate. 
//NOTE: acceleration includes gravity. Hovering drone is accelerating at G in positive z;

//request.vehicleState is the desired angle of attack. the force is based on the current pose. The current Pose demands a thrust to satisfy z thrust.
requestedVehicleState PIDDroneController::aotFeedForward(accelerations& accelerationCommand, float mass ,poseState& pose){
    if(mass <= 0) throw std::runtime_error("mass cannot be <=0 , in aotFeedForward\n");
    requestedVehicleState request;
    threeDState gravityCompensationVector = {0,0,-gravitationalAcceleration};
    auto poseDirVector = normalizeVector(pose.dirVector);

    threeDState accelerationCommandVector = {accelerationCommand.xAccel,accelerationCommand.yAccel,accelerationCommand.zAccel};

    accelerationCommandVector[2] = std::max(accelerationCommandVector[2],MINIMUM_ACCELERATION_Z * -gravityCompensationVector[2]);

    accelerationCommandVector = addVectors(accelerationCommandVector,gravityCompensationVector);


    request.vehicleState = normalizeVector(accelerationCommandVector);


    auto adjustedThrustVector = scaleVectorToZ(poseDirVector,accelerationCommandVector[2]);

    for(auto& acceleration:adjustedThrustVector){
        acceleration *= mass;
    }

    float adjustedThrust = vectorMag(adjustedThrustVector);
    float EPSILON = 0.01;

    if(NEAR(adjustedThrust,0.0f,EPSILON)){
        request.vehicleState = {0,0,1};
        request.force = 0;
        return request;
    }


    request.force = adjustedThrust;
    return request;
}




float PIDDroneController::yawControl(){
    return 0;
}





accelerations PIDDroneController::computeAccelerationRequest(const std::array<float,3>& pos ,const std::array<float,3>& velo ,float actualDeltaTime){
    controlOutputVelocity = {   PIDX->update(pos[0],actualDeltaTime),
                                PIDY->update(pos[1],actualDeltaTime),
                                PIDZ->update(pos[2],actualDeltaTime)};
    //Control outPut for velocity will take the output command for 
    PIDVX->setTarget(controlOutputVelocity[0]);
    PIDVY->setTarget(controlOutputVelocity[1]);
    PIDVZ->setTarget(controlOutputVelocity[2]);

    accelerations accelerationCommand;

    accelerationCommand.xAccel = PIDVX->update(velo[0],actualDeltaTime);
    accelerationCommand.yAccel = PIDVY->update(velo[1],actualDeltaTime);
    accelerationCommand.zAccel = PIDVZ->update(velo[2],actualDeltaTime);

    return accelerationCommand;
}

PIDDroneController::eulerAOT PIDDroneController::computeVehicleBasisAOT(poseState& state, requestedVehicleState& request){
    eulerAOT euler;

    poseState basisPose = CoordinateSystem::WORLD_BASIS;
    vehicleReferenceFrame referenceFrame(state, basisPose);    
    threeDState basisAOT = referenceFrame.realign(request.vehicleState);


    std::array<float,2> vectorX2d = {basisAOT[0], basisAOT[2]};
    euler.angleY = -signedAngle(vectorX2d);

    std::array<float,2> vectorY2d = {basisAOT[1], basisAOT[2]};
    euler.angleX = signedAngle(vectorY2d);

    //print(euler.angleX,"Angle X");
    //print(euler.angleY,"Angle Y");
    return euler;
}


rotationRate PIDDroneController::computeAngularVelocityFromAOT(eulerAOT euler, float actualDeltaTime) {
    rotationRate angularVelo;

    APIDX->setTarget(euler.angleX);
    APIDY->setTarget(euler.angleY);

    angularVelo.rollRate =  APIDX->update(0.0f, actualDeltaTime);
    angularVelo.pitchRate = APIDY->update(0.0f, actualDeltaTime);

    return angularVelo;
}



PIDDroneController::eulerMoments PIDDroneController::computeMomentFromAOT( poseState& state, rotationRate desiredAngularVelo,float actualDeltaTime){
    eulerMoments moments;

    APIDVX->setTarget(desiredAngularVelo.rollRate);
    APIDVY->setTarget(desiredAngularVelo.pitchRate);

    auto worldBasis = CoordinateSystem::WORLD_BASIS;
    vehicleReferenceFrame rotationFrame(lastState, worldBasis);


    poseState lastStateInWorld = lastState;
    poseState currentStateInWorld = state;
    rotationFrame.realignPose(lastStateInWorld);
    rotationFrame.realignPose(currentStateInWorld);

    
    poseDifference.setStartPose(lastStateInWorld);
    poseDifference.setEndPose(currentStateInWorld);
    auto angularVelocity = poseDifference.getRotationRate(actualDeltaTime);

    
    lastState = state;
    
    logger.pitchDesiredVelo = desiredAngularVelo.pitchRate;
    logger.rollDesiredVelo = desiredAngularVelo.rollRate;

    logger.pitchVelo = angularVelocity.pitchRate;
    logger.rollVelo = angularVelocity.rollRate;
    
    moments.moments[0] =  -APIDVX->update(angularVelocity.rollRate,actualDeltaTime);
    moments.moments[1] =  -APIDVY->update(angularVelocity.pitchRate,actualDeltaTime);


    return moments;
}


float PIDDroneController::computeThrustForce(float requestForce){
    auto rotationError = poseDifference.getDifference();
    float minimumThurst = mass*std::fabs(gravitationalAcceleration) * MINIMUM_THROTTLE_AT_IDLE;
    if(requestForce < minimumThurst){
        requestForce = minimumThurst;
    }
    return requestForce;
}


// Cascading PID Controller:
// Implements a multi-layer PID control structure for drone navigation to waypoints.
// The controller is organized into nested control loops:
// - Position PID computes desired velocity based on position error
// - Velocity PID computes desired acceleration or thrust based on velocity error
// - Attitude and Rate PIDs can follow to control orientation and angular velocity
// Used in flight control systems like PX4 and ArduPilot.
controlPacks::forceMoments PIDDroneController::pidControl(const std::array<float,3> pos ,const std::array<float,3> velo , poseState& state){

    if(!controlEnabled){
        controlPacks::forceMoments packet;
        return packet;
    }

    if(firstPose){
        lastState = state;
        firstPose = false;
    }

    if(!manager) std::cerr<<"Drone Contoller manager not found\n";
    auto actualDeltaTime = manager->getActualDeltaTime();
    
    auto accelerationCommand = computeAccelerationRequest(pos,velo,actualDeltaTime);

    requestedVehicleState request = aotFeedForward(accelerationCommand,mass,state);

    auto euler = computeVehicleBasisAOT(state,request);

    auto desiredAngularVelo = computeAngularVelocityFromAOT(euler, actualDeltaTime);

    auto moment  = computeMomentFromAOT(state,desiredAngularVelo,actualDeltaTime);

    controlPacks::forceMoments momentsForces;
    momentsForces.moments = moment.moments;
    yawControl();

    momentsForces.force[2] = computeThrustForce(request.force);


    //---- logger ----

    logger.requestedAOT = request.vehicleState;
    logger.reportedAOT = state.dirVector;
    logger.AOTerror_Rad = vectorAngleBetween(logger.requestedAOT,logger.reportedAOT);
    logger.requestedMoment = momentsForces.moments;


    return momentsForces; 
}


void PIDDroneController:: setTargetPosition(float xTarget , float yTarget , float zTarget){
    if (PIDX && PIDY && PIDZ) {
        PIDX->setTarget(xTarget);
        PIDY->setTarget(yTarget);
        PIDZ->setTarget(zTarget);
    } else {
        std::cerr << "One or more PID controllers are null!" << std::endl;
    }
}


/**
 * @brief Computes the drone's control output based on state estimation.
 */
controlPacks::variantPackets PIDDroneController::update(float time,stateInfo statePacket){ 
    
    if (!manager->shouldTrigger(time)) {
        return computedControlPacket;
    }

    logEstimationPacket = statePacket;
    telemetry();

    controlPacks::forceMoments request = pidControl(statePacket.position, statePacket.velocity,statePacket.pose);

    if (!allocator) {
        throw std::runtime_error("Control allocator not initialized in PIDDroneController::update");
    }
    
    
    auto allocatorPacket = allocator->computeAllocation(request);


    computedControlPacket.thrust = allocatorPacket.thrusts;

    return computedControlPacket;
}



controlPacks::forceMoments PIDDroneController::updateWithoutAllocator(float time,stateInfo statePacket){
    if (!manager->shouldTrigger(time)) {
        return computedForceMoments;
    }
    controlPacks::forceMoments request = pidControl(statePacket.position, statePacket.velocity,statePacket.pose);

    computedForceMoments = request;

    return computedForceMoments;

}




void dragEstimation(){

}


float PIDDroneController::yawAngleDifference(const poseState& currentPose, const poseState& desiredPose) {
    auto currentDirVector = currentPose.dirVector;
    auto desiredDirVector = desiredPose.dirVector;
    
    // Find rotation axis and angle between direction vectors
    float angleBetween = vectorAngleBetween(currentDirVector, desiredDirVector);
    threeDState axis;
    vectorCrossProduct(currentDirVector, desiredDirVector, axis);
    
    // Validate rotation direction
    threeDState validateRotation = currentDirVector;
    auto quantValidate = fromAxisAngle(axis, angleBetween);
    rotateVector(quantValidate, validateRotation);
    if (!similarVector(validateRotation, desiredDirVector)) {
        angleBetween = -angleBetween;
    }
    
    // Rotate current pose to align direction vectors
    auto quant = fromAxisAngle(axis, angleBetween);
    quaternionVehicle quantVehicle;
    quantVehicle.setVehicleQuaternionState(currentPose.dirVector, currentPose.fwdVector);
    quantVehicle.rotatePose(quant);
    auto rotatedPose = quantVehicle.getPose();
    
    // Calculate signed yaw angle
    float yawAngle = vectorAngleBetween(rotatedPose.fwdVector, desiredPose.fwdVector);
    
    // Determine yaw direction using cross product with direction vector
    threeDState yawAxis;
    vectorCrossProduct(rotatedPose.fwdVector, desiredPose.fwdVector, yawAxis);
    float yawDirection = vectorDotProduct(yawAxis, desiredDirVector);
    
    if (yawDirection < 0) {
        yawAngle = -yawAngle;
    }
    
    return yawAngle;
}
//this method is only used for testing 
controlPacks::forceMoments PIDDroneController::AOTHold(float actualDeltaTime,poseState state ,poseState requestedPose){

    requestedVehicleState request;
    request.vehicleState = requestedPose.dirVector;

    //force requested is = to hover thrust indpendent of AOT
    request.force = mass * gravitationalAcceleration;

    auto euler = computeVehicleBasisAOT(state,request);

    auto desiredAngularVelo = computeAngularVelocityFromAOT(euler, actualDeltaTime);

    auto moment  = computeMomentFromAOT(state,desiredAngularVelo,actualDeltaTime);

    controlPacks::forceMoments momentsForces;
    momentsForces.moments = moment.moments;
        
    yawControl();

    momentsForces.force[2] = computeThrustForce(request.force);




    return momentsForces; 
}

controlPacks::variantPackets PIDDroneController::updateAOTHold(float time,stateInfo statePacket, poseState requestedPose){
    if (!manager->shouldTrigger(time)) {
        return computedControlPacket;
    }

    if (!allocator) {
        throw std::runtime_error("Control allocator not initialized in PIDDroneController::update");
    }

    
    controlPacks::forceMoments request = AOTHold(manager->getActualDeltaTime(), statePacket.pose,requestedPose);

    auto allocatorPacket = allocator->computeAllocation(request);


    computedControlPacket.thrust = allocatorPacket.thrusts;

    graph.dataEntry(time,logger.rollDesiredVelo ,  logger.pitchDesiredVelo,logger.pitchVelo ,logger.rollVelo,vectorAngleBetween(statePacket.pose.dirVector,requestedPose.dirVector));

    return computedControlPacket;
}


void PIDDroneController::telemetry(){

}



}