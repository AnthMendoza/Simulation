#include "../../include/control/dronePIDControl.h"
#include "../../include/core/coordinateSystem.h"
#include "../../include/control/stateInfo.h"
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
    gravitationalAcceleation(other.gravitationalAcceleation)
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
    gravitationalAcceleation = droneParse.getFloat("gravitationalAcceleration");
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

    constexpr float MAX_ROTATIONAL_ANGULAR_VELOCITY = 18.85;//rad/s {18.85 rad/s = 3 rps}

    APIDVX->setOutputLimits(-MAX_ROTATIONAL_ANGULAR_VELOCITY,MAX_ROTATIONAL_ANGULAR_VELOCITY);
    APIDVY->setOutputLimits(-MAX_ROTATIONAL_ANGULAR_VELOCITY,MAX_ROTATIONAL_ANGULAR_VELOCITY);
    
    //position

    float Akp = PIDPrase.getFloat("AKp");
    float Aki = PIDPrase.getFloat("AKi");
    float Akd = PIDPrase.getFloat("AKd");

    if (APIDX == nullptr) APIDX = std::make_unique<PIDController>(Akp, Aki, Akd);
    if (APIDY == nullptr) APIDY = std::make_unique<PIDController>(Akp, Aki, Akd);


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
    PIDVZ->setOutputLimits(gravitationalAcceleation,maxAcceleration + abs(gravitationalAcceleation));
}

//accelerationFromPose takes the current pose of the vehicle and calculates the required thrust given the CURRENT conditions.
//if aotFeedForwad calculates the thrust needed at the desired angle of attack for the desired postion and not the current position the motors will generate the thrust required faster than the vehicle can rotate to the position.
// resulting in over delay from thrust to angle requested
static float accelerationFromPose(const threeDState accels,poseState& pose){
    threeDState newAccelVector;
    auto dir = pose.dirVector;
    newAccelVector = scaleVectorToZ(dir,accels[2]);
    
    return vectorMag(newAccelVector);

}


//flight envolope. zaccel is dominate. 
//NOTE: acceleration includes gravity. Hovering drone is accelerating at G in positive z;
requestedVehicleState PIDDroneController::aotFeedForward(accelerations& accels, float gravitaionalAcceleration , float mass ,poseState& pose){
    if(mass <= 0) throw std::runtime_error("mass cannot be <=0 , in aotFeedForward\n");
    //negative Z acceleration greater than gravity means the vehicle is upsideDown. For now this is not possible.
    threeDState vectorRepresentation = {accels.xAccel,accels.yAccel,accels.zAccel};
    
    if(gravitaionalAcceleration >= 0) throw std::runtime_error("GravitationalAcceleration is not negative in PIDDroneController.\n");
    vectorRepresentation[2] += -gravitaionalAcceleration;
    vectorRepresentation[2] = std::clamp(vectorRepresentation[2],gravitaionalAcceleration,std::numeric_limits<float>::max());
    // FRACTION_OF_GRAVITY ensures some position acceleration is present to resist gravity.
    //preventing an inverted command.
    constexpr float FRACTION_OF_GRAVITY = 0.1f;
    //if(accels.zAccel <= 0) accels.zAccel = -gravitaionalAcceleration * FRACTION_OF_GRAVITY;

    if(mass <= 0) throw std::runtime_error("Mass Cannot be <=0.\n");
    //limitMagnitudeWithFixedZ is a possible inprovment on limiting accelerationrequest. more devlopment is needed
    //vectorRepresentation = limitMagnitudeWithFixedZ(vectorRepresentation,maxAcceleration);
    //resizeVectorIfLarger(vectorRepresentation,maxAcceleration);
    if(vectorRepresentation[0] == 0 && vectorRepresentation[1] == 0){
        //vectorRepresentation[2] = std::clamp(vectorRepresentation[2],0.0f,maxAcceleration);
        vectorRepresentation[2] = std::clamp(vectorRepresentation[2],0.0f,std::numeric_limits<float>::max());
    }
    float magnitudeOfAcceleration = vectorMag(vectorRepresentation);
    if(magnitudeOfAcceleration == 0){
        requestedVehicleState request;
        request.vehicleState = {0,0,1};
        request.force = 0;
        return request;
    }

    requestedVehicleState request;
    request.vehicleState = normalizeVector(vectorRepresentation);
    request.force = accelerationFromPose(vectorRepresentation,pose) * mass;
    return request;
}

static constexpr float MINIMUM_THROTTLE_AT_IDLE = 0.1;


// Cascading PID Controller:
// Implements a multi-layer PID control structure for drone navigation to waypoints.
// The controller is organized into nested control loops:
// - Position PID computes desired velocity based on position error
// - Velocity PID computes desired acceleration or thrust based on velocity error
// - Attitude and Rate PIDs can follow to control orientation and angular velocity
// Used in flight control systems like PX4 and ArduPilot.
controlPacks::forceMoments PIDDroneController::pidControl(const std::array<float,3> pos ,std::array<float,3> velo , poseState& state){

    if(!controlEnabled){
        controlPacks::forceMoments packet;
        return packet;
    }
    if(firstPose){
        lastState = state;
        firstPose = false;
    }
    auto actualDeltaTime = manager->getActualDeltaTime();
    //wayPoint should be set prior to call via setPidControl();
    if(!manager) std::cerr<<"Drone Contoller manager not found\n";
    float deltaTime = manager->getActualDeltaTime();

    controlOutputVelocity = {   PIDX->update(pos[0],actualDeltaTime),
                                PIDY->update(pos[1],actualDeltaTime),
                                PIDZ->update(pos[2],actualDeltaTime)};
    //Control outPut for velocity will take the output command for 
    PIDVX->setTarget(controlOutputVelocity[0]);
    PIDVY->setTarget(controlOutputVelocity[1]);
    PIDVZ->setTarget(controlOutputVelocity[2]);

    accelerations accels;

    accels.xAccel = PIDVX->update(velo[0],actualDeltaTime);
    accels.yAccel = PIDVY->update(velo[1],actualDeltaTime);
    accels.zAccel = PIDVZ->update(velo[2],actualDeltaTime);
    

    requestedVehicleState request = aotFeedForward(accels,gravitationalAcceleation,mass,state);

    poseState basisPose = CoordinateSystem::WORLD_BASIS;

    vehicleReferenceFrame referenceFrame(state , basisPose);
    threeDState basisAOT = referenceFrame.realign(request.vehicleState);

    float angleX,angleY;
    float EPSILON = 1e-4;
    float MAX_Angle = 1.0f;
    if(std::fabs(basisAOT[2]) <= EPSILON) basisAOT[2] = EPSILON;

    std::array<float,2> vectorX2d = {basisAOT[0], basisAOT[2]};
    angleX = signedAngle(vectorX2d);

    std::array<float,2> vectorY2d = {basisAOT[1], basisAOT[2]};
    angleY = signedAngle(vectorY2d);

    //Another angle limit can be added if not symetrical
    ellipsoidalClamp2D(angleX,angleY,MAX_Angle,MAX_Angle);

    APIDX->setTarget(0.0f);
    APIDY->setTarget(0.0f);

    float angularVeloX = APIDX->update(angleX,actualDeltaTime);
    float angularVeloY = -APIDY->update(angleY,actualDeltaTime);

    threeDState APIDMoments = {0,0,0};

    APIDVX->setTarget(angularVeloX);
    APIDVY->setTarget(angularVeloY);


    vehicleReferenceFrame rotationFrame(lastState,basisPose);
    lastState = state;
    rotationFrame.realignPose(state);
    poseDifference.setStartPose(basisPose);
    poseDifference.setEndPose(state);

    //poseDifference.setStartPose(lastState);
    //poseDifference.setEndPose(state);
    //lastState = state;

    auto angularVelocity = poseDifference.getRotationRate(deltaTime);
    
    APIDMoments[1] = -APIDVX->update(angularVelocity.pitchRate,actualDeltaTime);
    APIDMoments[0] = -APIDVY->update(angularVelocity.rollRate,actualDeltaTime);
    controlPacks::forceMoments momentsForces;
    momentsForces.moments = APIDMoments;
    momentsForces.force[2] = request.force;
    auto rotationError = poseDifference.getDifference();
    float minimumThurst = mass*std::fabs(gravitationalAcceleation) * MINIMUM_THROTTLE_AT_IDLE;
    if(request.force < minimumThurst){
        request.force = minimumThurst;
    }

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
    
    VectorXd desired  = allocator->toVectorXd({request.force[0],request.force[1],request.force[2],request.moments[0],request.moments[1],request.moments[2]});
    VectorXd thrusts = allocator->allocate(desired);
    VectorXd output = allocator->computeWrench(thrusts);


    //VectorXd to vector<float> can be converted into a method if needed further
    computedControlPacket.thrust.clear();
    for (int i = 0; i < thrusts.size(); ++i) {
        computedControlPacket.thrust.push_back(static_cast<float>(thrusts[i]));
    }
    
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


}