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




void PIDDroneController::initController(string droneConfig){
    toml::tomlParse droneParse;
    droneParse.parseConfig(droneConfig,"vehicle");
    float timeStep = droneParse.getFloat("timeStep");
    mass = droneParse.getFloat("mass");
    gravitationalAcceleration = droneParse.getFloat("gravitationalAcceleration");
    toml::tomlParse PIDPrase;
    PIDPrase.parseConfig(droneConfig,"PID");

    //#########################################################################
    //Position control loop
    // requests a velocity
    float kp = PIDPrase.getFloat("Kp");
    float ki = PIDPrase.getFloat("Ki");
    float kd = PIDPrase.getFloat("Kd");
    if (PIDX == nullptr)
        PIDX = std::make_unique<PIDController>(kp, ki, kd);

    if (PIDY == nullptr)
        PIDY = std::make_unique<PIDController>(kp, ki, kd);

    if (PIDZ == nullptr)
        PIDZ = std::make_unique<PIDController>(
            PIDPrase.getFloat("ZKp"),
            PIDPrase.getFloat("ZKi"),
            PIDPrase.getFloat("ZKd"));
    toml::tomlParse vParse;
    vParse.parseConfig(droneConfig,"vehicle");
    float maxCruiseVelocity = vParse.getFloat("maxCruiseVelocity");
    PIDX->setOutputLimits(-maxCruiseVelocity,maxCruiseVelocity);
    PIDY->setOutputLimits(-maxCruiseVelocity,maxCruiseVelocity);
    PIDZ->setOutputLimits(-10,10);


    //#########################################################################
    //Velocity Control loop
    //requests an acceleration
    float kvp = PIDPrase.getFloat("Kvp");
    float kvi = PIDPrase.getFloat("Kvi");
    float kvd = PIDPrase.getFloat("Kvd");

    if (PIDVX == nullptr)
        PIDVX = std::make_unique<PIDController>(kvp, kvi, kvd);

    if (PIDVY == nullptr)
        PIDVY = std::make_unique<PIDController>(kvp, kvi, kvd);

    if (PIDVZ == nullptr)
        PIDVZ = std::make_unique<PIDController>(
            PIDPrase.getFloat("ZKvp"),
            PIDPrase.getFloat("ZKvi"),
            PIDPrase.getFloat("ZKvd"));


    //#########################################################################
    //Angular control loop
    //velocity
    float AVkp = PIDPrase.getFloat("AVKp");
    float AVki = PIDPrase.getFloat("AVKi");
    float AVkd = PIDPrase.getFloat("AVKd");

    if (APIDVX == nullptr) APIDVX = std::make_unique<PIDController>(AVkp, AVki, AVkd);
    if (APIDVY == nullptr) APIDVY = std::make_unique<PIDController>(AVkp, AVki, AVkd);

    float MAX_ROTATIONAL_ACCELERATION = 50; //rad/s^2

    auto MOI = droneParse.getArray("MOI");
    float MAX_MOMENT = MOI[0] * MAX_ROTATIONAL_ACCELERATION;

    APIDVX->setOutputLimits(-MAX_MOMENT,MAX_MOMENT);
    APIDVY->setOutputLimits(-MAX_MOMENT,MAX_MOMENT);

    APIDVX->setIntegralClamp(AVki * 10);
    APIDVX->setIntegralClamp(AVki * 10);
    //position

    float Akp = PIDPrase.getFloat("AKp");
    float Aki = PIDPrase.getFloat("AKi");
    float Akd = PIDPrase.getFloat("AKd");

    if (APIDX == nullptr) APIDX = std::make_unique<PIDController>(Akp, Aki, Akd);
    if (APIDY == nullptr) APIDY = std::make_unique<PIDController>(Akp, Aki, Akd);
    
    constexpr float MAX_ROTATIONAL_ANGULAR_VELOCITY = 18.85;//rad/s {18.85 rad/s = 3 rps}

    APIDX->setOutputLimits(-MAX_ROTATIONAL_ANGULAR_VELOCITY,MAX_ROTATIONAL_ANGULAR_VELOCITY);
    APIDY->setOutputLimits(-MAX_ROTATIONAL_ANGULAR_VELOCITY,MAX_ROTATIONAL_ANGULAR_VELOCITY);

    APIDX->setIntegralClamp(Aki * 10);
    APIDX->setIntegralClamp(Aki * 10);

    APIDX->setOutputLimits(-2.5,2.5);
    APIDY->setOutputLimits(-2.5,2.5);
    updateCalculatedValues();

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



//flight envolope. zaccel is dominate. 
//NOTE: acceleration includes gravity. Hovering drone is accelerating at G in positive z;

//request.vehicleState is the desired angle of attack. the force is based on the current pose. The current Pose demands a thrust to satisfy z thrust.
requestedVehicleState PIDDroneController::aotFeedForward(accelerations& accelerationCommand, float mass ,poseState& pose){
    if(mass <= 0) throw std::runtime_error("mass cannot be <=0 , in aotFeedForward\n");
    requestedVehicleState request;
    threeDState gravitiyCompensationVector = {0,0,-gravitationalAcceleration};
    auto poseDirVector = normalizeVector(pose.dirVector);

    threeDState accelerationCommandVector = {accelerationCommand.xAccel,accelerationCommand.yAccel,accelerationCommand.zAccel};
    
    accelerationCommandVector = addVectors(accelerationCommandVector,gravitiyCompensationVector);

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

static constexpr float MINIMUM_THROTTLE_AT_IDLE = 0.1;


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

PIDDroneController::eulerAOT PIDDroneController::computeVehicleBasisAOT(poseState& state , requestedVehicleState& request){

    eulerAOT euler;

    poseState basisPose = CoordinateSystem::WORLD_BASIS;

    vehicleReferenceFrame referenceFrame(state , basisPose);    
    threeDState basisAOT = referenceFrame.realign(request.vehicleState);

    float EPSILON = 1e-4;
    float MAX_Angle = 0.8f;

    if(std::fabs(basisAOT[2]) <= EPSILON) basisAOT[2] = EPSILON;

    std::array<float,2> vectorX2d = {basisAOT[0], basisAOT[2]};
    euler.angleY = -signedAngle(vectorX2d);

    std::array<float,2> vectorY2d = {basisAOT[1], basisAOT[2]};
    euler.angleX = signedAngle(vectorY2d);


    //Another angle limit can be added if not symetrical
    ellipsoidalClamp2D(euler.angleX,euler.angleY,MAX_Angle,MAX_Angle);

    return euler;
}


rotationRate PIDDroneController::computeAngularVelocityFromAOT(eulerAOT euler, float actualDeltaTime) {
    rotationRate angularVelo;

    APIDX->setTarget(0.0f);
    APIDY->setTarget(0.0f);

    angularVelo.rollRate =  APIDX->update(euler.angleX, actualDeltaTime);
    angularVelo.pitchRate = APIDY->update(euler.angleY, actualDeltaTime);

    return angularVelo;
}



PIDDroneController::eulerMoments PIDDroneController::computeMomentFromAOT( poseState& state, rotationRate desiredAngularVelo,float actualDeltaTime){
    eulerMoments moments;

    //std::cout<< desiredAngularVelo.rollRate << " , " << desiredAngularVelo.pitchRate << "\n";

    APIDVX->setTarget(desiredAngularVelo.rollRate);
    APIDVY->setTarget(desiredAngularVelo.pitchRate);


    auto worldBasis = CoordinateSystem::WORLD_BASIS;
    vehicleReferenceFrame rotationFrame(lastState,worldBasis);
    lastState = state;
    rotationFrame.realignPose(state);
    poseDifference.setStartPose(worldBasis);
    poseDifference.setEndPose(state);


    auto angularVelocity = poseDifference.getRotationRate(actualDeltaTime);
    
    moments.moments[0] = APIDVX->update(angularVelocity.rollRate,actualDeltaTime);
    //moments.moments[1] = APIDVY->update(angularVelocity.pitchRate,actualDeltaTime);

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







std::pair<std::array<float,3>, float> PIDDroneController::aotControl(requestedVehicleState request, std::array<float,3> currentState) {
    
    std::array<float,3> current = normalizeVector(currentState);

    std::array<float,3> rotationAxis;
    vectorCrossProduct(current, request.vehicleState,rotationAxis);
    float crossMagnitude = vectorMag(rotationAxis);
    if (crossMagnitude < 1e-6f) {
        return {{0,0,0},0};
    }
    rotationAxis = normalizeVector(rotationAxis);


    std::pair<std::array<float,3>, float> result = {rotationAxis, 0.0f};

    return result;
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



    //---- logger ----

    logger.requestedAOT = request.vehicleState;
    logger.reportedAOT = state.dirVector;
    logger.AOTerror_Rad = vectorAngleBetween(logger.requestedAOT,logger.reportedAOT);
    logger.requestedMoment = momentsForces.moments;


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

    return computedControlPacket;
}



}