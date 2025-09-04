#include "../include/control/dronePIDControl.h"
#include <algorithm>
#include <iostream>
#include <cmath>


namespace SimCore{
PIDDroneController::PIDDroneController(const PIDDroneController& other):
    droneControllerBase(other),
    maxThrust(other.maxThrust),
    mass(other.mass),
    maxAcceleration(other.maxAcceleration),
    gravitationalAcceleation(other.gravitationalAcceleation)
    {
    
    if (other.PIDX)  PIDX  = std::make_unique<PIDController>(*other.PIDX);
    if (other.PIDY)  PIDY  = std::make_unique<PIDController>(*other.PIDY);
    if (other.PIDZ)  PIDZ  = std::make_unique<PIDController>(*other.PIDZ);
    
    if (other.PIDVX) PIDVX = std::make_unique<PIDController>(*other.PIDVX);
    if (other.PIDVY) PIDVY = std::make_unique<PIDController>(*other.PIDVY);
    if (other.PIDVZ) PIDVZ = std::make_unique<PIDController>(*other.PIDVZ);
    
    if (other.APIDX)  APIDX  = std::make_unique<PIDController>(*other.APIDX);
    if (other.APIDY)  APIDY  = std::make_unique<PIDController>(*other.APIDY);
}



void PIDDroneController::initController(string droneConfig, const droneBody& drone){
    toml::tomlParse droneParse;
    droneParse.parseConfig(droneConfig,"vehicle");
    float timeStep = droneParse.getFloat("timeStep");

    toml::tomlParse PIDPrase;
    PIDPrase.parseConfig(droneConfig,"PID");

    //#########################################################################
    //Position control loop
    // requests a velocity
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
    toml::tomlParse vParse;
    vParse.parseConfig(droneConfig,"vehicle");
    float maxCruiseVelocity = vParse.getFloat("maxCruiseVelocity");
    PIDX->setOutputLimits(-maxCruiseVelocity,maxCruiseVelocity);
    PIDY->setOutputLimits(-maxCruiseVelocity,maxCruiseVelocity);
    PIDZ->setOutputLimits(-1,1);


    //#########################################################################
    //Velocity Control loop
    //requests an acceleration
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


    //#########################################################################
    //Angular control loop
    float Akp = PIDPrase.getFloat("AKp");
    float Aki = PIDPrase.getFloat("AKi");
    float Akd = PIDPrase.getFloat("AKd");

    if (APIDX == nullptr) APIDX = std::make_unique<PIDController>(Akp, Aki, Akd, timeStep);
    if (APIDY == nullptr) APIDY = std::make_unique<PIDController>(Akp, Aki, Akd, timeStep);


    APIDX->setOutputLimits(-1,1);
    APIDY->setOutputLimits(-1,1);
    updateCalculatedValues(drone);

}


void PIDDroneController::updateCalculatedValues(const droneBody& drone){
    maxThrust = drone.totalThrustLimit;
    if(maxThrust < 0 ) std::cerr<<"\nWarning: MaxThrust in initController is <0.\n";
    mass = drone.getMass();
    gravitationalAcceleation = drone.getGravitationalAcceleration();
    maxAcceleration = maxThrust/mass;
    PIDVX->setOutputLimits(-maxAcceleration,maxAcceleration);
    PIDVY->setOutputLimits(-maxAcceleration,maxAcceleration);
    PIDVZ->setOutputLimits(gravitationalAcceleation,maxAcceleration - gravitationalAcceleation);
}




//flight envolope. zaccel is dominate. 
//NOTE: acceleration includes gravity. Hovering drone is accelerating at G in positive z;
requestedVehicleState PIDDroneController::aotFeedForward(accelerations& accels, float gravitaionalAcceleration , float mass ,float maxThrust){
    //negative Z acceleration greater than gravity means the vehicle is upsideDown. For now this is not possible.
    threeDState vec = {accels.xAccel,accels.yAccel,accels.zAccel};

    if(gravitaionalAcceleration > 0) throw std::runtime_error("GravitationalAcceleration is not negative in PIDDroneController.\n");
    accels.zAccel = std::clamp(accels.zAccel,gravitaionalAcceleration,std::numeric_limits<float>::max());
    accels.zAccel += -gravitaionalAcceleration;
    // FRACTION_OF_GRAVITY ensures some position acceleration is present to resist gravity.
    //preventing an inverted command.
    constexpr float FRACTION_OF_GRAVITY = 0.1f;
    if(accels.zAccel <= 0) accels.zAccel = -gravitaionalAcceleration * FRACTION_OF_GRAVITY;
    threeDState vectorRepresentation = {accels.xAccel,accels.yAccel,accels.zAccel};

    if(mass <= 0) throw std::runtime_error("Mass Cannot be <=0.\n");
    float maxAcceleration = maxThrust/mass;
    vectorRepresentation = limitMagnitudeWithFixedZ(vectorRepresentation,maxAcceleration);
    if(vectorRepresentation[0] == 0 && vectorRepresentation[1] == 0){
        vectorRepresentation[2] = std::clamp(vectorRepresentation[2],0.0f,maxAcceleration);
    }
    float magnitudeOfAcceleration = vectorMag(vectorRepresentation);

    if(magnitudeOfAcceleration == 0){
        requestedVehicleState request;
        request.vehicleState = {0,0,1};
        request.force = 0;
        return request;
    }
    float force = magnitudeOfAcceleration * mass;

    requestedVehicleState request;
    request.vehicleState = normalizeVector(vectorRepresentation);
    request.force = force;

    return request;
}


// Cascading PID Controller:
// Implements a multi-layer PID control structure for drone navigation to waypoints.
// The controller is organized into nested control loops:
// - Position PID computes desired velocity based on position error
// - Velocity PID computes desired acceleration or thrust based on velocity error
// - Attitude and Rate PIDs can follow to control orientation and angular velocity
// Used in flight control systems like PX4 and ArduPilot.
momentForceRequest PIDDroneController::pidControl(const std::array<float,3> pos ,std::array<float,3> velo , poseState& state, float maxThrust){
    if(!controlEnabled) return {{0,0,0},{0,0,0}};
    //wayPoint should be set prior to call via setPidControl();
    
    controlOutputVelocity = {   PIDX->update(pos[0]),
                                PIDY->update(pos[1]),
                                PIDZ->update(pos[2])};
    print(PIDY->lastError(),"PIDY Last Error");
    //Control outPut for velocity will take the output command for 
    PIDVX->setTarget(controlOutputVelocity[0]);
    PIDVY->setTarget(controlOutputVelocity[1]);
    PIDVZ->setTarget(controlOutputVelocity[2]);

    accelerations accels;


    accels.xAccel = PIDVX->update(velo[0]);
    accels.yAccel = PIDVY->update(velo[1]);
    accels.zAccel = PIDVZ->update(velo[2]);
    std::cout << "[Velocities] X: " << velo[0]
          << " Y: " << velo[1]
          << " Z: " << velo[2] << "  |  "
          << "[Accels] X: " << accels.xAccel
          << " Y: " << accels.yAccel
          << " Z: " << accels.zAccel << '\n';
    requestedVehicleState request = aotFeedForward(accels,gravitationalAcceleation,mass,maxThrust);
    print(state.dirVector,"state");
    print(request.vehicleState,"request.vehicleState");
    vehicleRefranceFrame refranceFrame(state);
    threeDState basisAOT = refranceFrame.realign(request.vehicleState);
    float angleX,angleY;
    print(basisAOT,"basisAOT");
    float EPSILON = 1e-4;
    float MAX_Angle = 1.2f;
    if(std::fabs(basisAOT[2]) <= EPSILON) basisAOT[2] = EPSILON;
    angleX = std::atanf(basisAOT[0] / basisAOT[2]);
    angleY = std::atanf(basisAOT[1] / basisAOT[2]);
    angleX = std::clamp(angleX,-MAX_Angle,MAX_Angle);
    angleY = std::clamp(angleY,-MAX_Angle,MAX_Angle);
    std::cout << "angleX: " << angleX << " (basisAOT[0]: " << basisAOT[0] << ", basisAOT[2]: " << basisAOT[2] << ")\n";
    std::cout << "angleY: " << angleY << " (basisAOT[1]: " << basisAOT[1] << ", basisAOT[2]: " << basisAOT[2] << ")\n";
    APIDX->setTarget(0.0f);
    APIDY->setTarget(0.0f);

    threeDState APIDMoments = {0,0,0};

    APIDMoments[0] = APIDX->update(angleX);
    APIDMoments[1] = APIDY->update(angleY);
    
    momentForceRequest momentsForces;
    momentsForces.moments = APIDMoments;
    momentsForces.forces[2] = request.force;
    
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

vector<float> PIDDroneController::update(const std::array<float,3>& estimatedPostion,poseState state,const std::array<float,3>& estimatedVelocity ,float time ){ 
    
    if (time - lastComputeTime < frequency ) {
        return computedThrust;
    }
    lastComputeTime = time;
    momentForceRequest request = pidControl(estimatedPostion, estimatedVelocity,state,maxThrust);
    if (!allocator) {
        throw std::runtime_error("Control allocator not initialized in PIDDroneController::update");
    }

    
    VectorXd desired  = allocator->toVectorXd({request.forces[0],request.forces[1],request.forces[2],request.moments[0],request.moments[1],request.moments[2]});
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
    computedThrust.clear();
    for (int i = 0; i < thrusts.size(); ++i) {
        computedThrust.push_back(static_cast<float>(thrusts[i]));
    }
    return computedThrust;
}




PIDDroneController::PIDDroneController(){
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