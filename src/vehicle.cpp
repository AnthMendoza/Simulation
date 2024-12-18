#include <iostream>
#include <array>
#include <cmath>
#include "../include/forceApplied.h"
#include "../include/vehicle.h"
#include "../include/vectorMath.h"
#include "../include/aero.h"
#include "../include/constants.h"
#include "../include/RungeKutta.h"
#include "../include/odeIterator.h"
#include "../include/rotationMatrix.h"
#include "../include/getRotation.h"
#include "../include/control.h"









Vehicle::Vehicle(){
    // roll pitch yaw (x,y,z) defines the direction vector, heading. ex. (0,0,1) is rocket pointing straight up.

    //(x,y,z)positon defines its location reltaive to an orgin

    dryMass = constants::dryMass;
    fuel = constants::initFuel;
    LOX = constants::initLOX;
    mass = constants::dryMass + constants::initFuel + constants::initLOX;
    fuelConsumptionRate = constants::consumtionRateFuel;
    LOXConsumptionRate = constants::consumtionRateLOX;

    MOI = constants::MOI;

    targetLandingPosition = constants::LandingTarget;

    Xposition = constants::initPosition[0];
    Yposition = constants::initPosition[1];
    Zposition = constants::initPosition[2];

    reentry = false;
    glidePhase = false; //false means yet to happen, true means already happened
    landing = false;
    landingInProgress = false;

    iterations = 0;

    engineState = {0,0,0};
    appliedVector = {0,0,0};

    gimbalErrorX = 0;
    gimbalErrorY = 0;
    gimbalX = 0;
    gimbalY = 0;
    gimbalVelocityX = 0;
    gimbalVelocityY = 0;

    twoDAngle = {0,0};
    error = 0;

    sumOfGimbalErrorX = 0;
    sumOfGimbalErrorY = 0;

    vehicleYError = 0;
    sumOfVehicleYError = 0;

    vehicleXError = 0;
    sumOfVehicleXError = 0;

    maxGimbalAcceleration = 10; // rad/s/s


    gForce = 0;

    angularVelocity = {0,0,0};
    

    centerOfPressure = constants::centerOfPressure; //meters
    cogToEngine = constants::cogToEngine;

    vehicleState[0] = constants::initVehicleState[0];
    vehicleState[1] = constants::initVehicleState[1];  //setting init value for vehicle state, logged in constants.h
    vehicleState[2] = constants::initVehicleState[2];

    Xvelocity = constants::initVelocity[0];
    Yvelocity = constants::initVelocity[1];   // Velocity vector , direction of movment relative to the ground
    Zvelocity = constants::initVelocity[2];

    logXInput = 0;
    logYInput = 0;
    logPosX = 0;
    logPosX = 0;
    

    sumOfForces[0] = 0;
    sumOfForces[1] = 0;
    sumOfForces[2] = 0;

    sumOfMoments[0] = 0;
    sumOfMoments[1] = 0;
    sumOfMoments[2] = 0;

    logMoment = {0,0,0};

    dragLog = {0,0,0};


    }



// Method implementation to display the vehicle's state
void Vehicle::display() {
    std::cout << "Position: (" << Xposition << ", " << Yposition << ", " << Zposition << ")\n"
              << "Orientation (Roll, Pitch, Yaw): (" << vehicleState[0] << ", " << vehicleState[1]  << ", " << vehicleState[2]  << ")\n";
}   

float Vehicle::getVelocity(){
    std::array<float ,3> vector = {Xvelocity , Yvelocity , Zvelocity};
    return vectorMag( vector );
}



float Vehicle::getGForce(){
    
    std::array<float ,3> vector = {sumOfForces[0]/mass,sumOfForces[1]/mass,sumOfForces[2]/mass};

    return vectorMag(vector)/9.8;
}




void Vehicle::drag(){
    
    std::array<float,3> airVelocityVector;

    airVelocityVector[0] = Xvelocity + constants::wind[0];
    airVelocityVector[1] = Yvelocity + constants::wind[1];
    airVelocityVector[2] = Zvelocity + constants::wind[2];


    float absVelocity = vectorMag(airVelocityVector);

    float dragAngle = vectorAngleBetween(airVelocityVector , vehicleState);

    
    float drag = -.5 * (absVelocity * absVelocity) * aeroArea(dragAngle) * coefOfDrag(dragAngle) * airDensity(Zposition);

    std::array<float,3> dragVector;
    std::array<float,3> normalVelocityVector = normalizeVector(airVelocityVector);
    
    dragVector[0] = drag * normalVelocityVector[0];
    dragVector[1] = drag * normalVelocityVector[1];
    dragVector[2] = drag * normalVelocityVector[2];

    addForce(dragVector);

    dragLog = forceToMoment(dragVector, vehicleState , centerOfPressure);

    addMoment(forceToMoment(dragVector, vehicleState , centerOfPressure));

}






void Vehicle::lift(){

    //lift acting on the center of pressure.

    std::array<float,3> airVelocityVector;

    airVelocityVector[0] = Xvelocity + constants::wind[0];
    airVelocityVector[1] = Yvelocity + constants::wind[1];
    airVelocityVector[2] = Zvelocity + constants::wind[2];


    float absVelocity = vectorMag(airVelocityVector);

    // Project vector V onto the plane with normal N: V_proj_plane = V - ((V • N) / (N • N)) * N (subtracting the projection onto N).
    //using normilzed vectors N • N is removed becuase it equals 1, this may or may not be faster than the equation abouve


    std::array<float , 3> normalAirVelocityVector = normalizeVector(airVelocityVector);

    float projection = vectorDotProduct( vehicleState , normalAirVelocityVector );

    std::array<float , 3> projectedVector;


    for(int i = 0 ; i < 3 ; i++){
        projectedVector[i] =  vehicleState[i] - projection * normalAirVelocityVector[i];
    }

    //we normilized this vector so that we can multiply it by a scalar with expected results
    projectedVector = normalizeVector(projectedVector); 

    std::array<float,3> reverseVehicleState;

    for(int i = 0 ; i < 3 ; i++) reverseVehicleState[i] = -vehicleState[i];


    float liftAngle = vectorAngleBetween(airVelocityVector , reverseVehicleState); 

    //create a mapping function for lift to force, this is a crude esimate.
    
    float lift = -.5 * (absVelocity * absVelocity) * aeroArea(liftAngle) * coefOfLift(liftAngle) * airDensity(Zposition); //calculating abs drag 
    
    std::array<float,3> liftVector;

    liftVector[0] = lift * projectedVector[0];
    liftVector[1] = lift * projectedVector[1];
    liftVector[2] = lift * projectedVector[2];

    if(directionality(normalAirVelocityVector , vehicleState) == false){
        liftVector[0] = -liftVector[0];
        liftVector[1] = -liftVector[1];
        liftVector[2] = -liftVector[2];
    }

    addForce(liftVector);

    addMoment(forceToMoment(liftVector, vehicleState , centerOfPressure));

    
}   


void Vehicle::applyEngineForce(std::array<float,2> twoDEngineRadians , float thrust){
    
    if(fuelConsumption(thrust) == false) return;

    thrust = -thrust;



    Matrix3x3 rotX = rotationMatrixX(twoDEngineRadians[0]);
    Matrix3x3 rotY = rotationMatrixY(twoDEngineRadians[1]);

    std::array<float,3> engineVector = {0,0,1};

    Eigen::Vector3d v1(engineVector[0], engineVector[1], engineVector[2]);

    Matrix3x3 combined = rotX * rotY;

    engineVector = combined.rotate(engineVector);


    std::array<float , 3> vehicleStateAntiparallel = vehicleState;
    // reverses the vector so that we can apply a thrust in the oposite direction

    for(int i = 0; i<3 ; i++){
        vehicleStateAntiparallel[i] = vehicleStateAntiparallel[i]; 
    } 

    Eigen::Vector3d v2( vehicleStateAntiparallel[0],
                        vehicleStateAntiparallel[1],
                        vehicleStateAntiparallel[2]);
    
    Eigen::Vector3d rotation = getRotationAngles(v1, v2);


    std::array<float,3> vehicleStateAngles = {static_cast<float>(rotation(0)) , static_cast<float>(rotation(1)) , static_cast<float>(rotation(2))};

    Matrix3x3 rotXVehicle = rotationMatrixX(vehicleStateAngles[0]); 
    Matrix3x3 rotYVehicle = rotationMatrixY(vehicleStateAngles[1]); 
    Matrix3x3 rotZVehicle = rotationMatrixZ(vehicleStateAngles[2]); 


    Matrix3x3 realign = rotXVehicle * rotYVehicle * rotZVehicle;

    engineVector = realign.rotate(engineVector);

    

    engineVector[0] = engineVector[0] *-thrust;
    engineVector[1] = engineVector[1] *-thrust;
    engineVector[2] = engineVector[2] *-thrust;

    
    
    addForce(engineVector);
    
    



    if(twoDEngineRadians[0] != 0 && twoDEngineRadians[1]  != 0){
        //addMoment(forceToMoment(engineVector, vehicleState , cogToEngine));
        //logMoment = forceToMoment(engineVector, vehicleState , cogToEngine);
    }
    
    appliedVector[0] = vehicleState[0] * cogToEngine;
    appliedVector[1] = vehicleState[1] * cogToEngine;
    appliedVector[2] = vehicleState[2] * cogToEngine;

    engineState = engineVector;

    engineForce = abs(thrust);

}



void  Vehicle::addForce(std::array<float,3> forceVector){
    sumOfForces[0] += forceVector[0];
    sumOfForces[1] += forceVector[1];
    sumOfForces[2] += forceVector[2];
}



void  Vehicle::addMoment(std::array<float,3> moments){
    sumOfMoments[0] += moments[0];
    sumOfMoments[1] += moments[1];
    sumOfMoments[2] += moments[2];
}



void Vehicle::updateState(){    
    //adding gravity to the force of Z, becuase this is an acceleration and not a force; The addForce function cannot handle it
    sumOfForces[2] = sumOfForces[2] + constants::gravitationalAcceleration * mass; 

    

    RungeKutta4th(sumOfForces[0] , mass , constants::timeStep , Xvelocity,Xposition);
    RungeKutta4th(sumOfForces[1] , mass , constants::timeStep , Yvelocity,Yposition);
    RungeKutta4th(sumOfForces[2] , mass , constants::timeStep , Zvelocity,Zposition);



    Matrix3x3 rotX = rotationMatrixX(rotationalOde(sumOfMoments[0] , MOI[0], constants::timeStep ,angularVelocity[0]));
    Matrix3x3 rotY = rotationMatrixY(rotationalOde(sumOfMoments[1] , MOI[1], constants::timeStep ,angularVelocity[1]));
    Matrix3x3 rotZ = rotationMatrixZ(rotationalOde(sumOfMoments[2] , MOI[2], constants::timeStep ,angularVelocity[2]));

    Matrix3x3 combined = rotX * rotY *rotZ;
    vehicleState = combined.rotate(vehicleState);
    vehicleState = normalizeVector(vehicleState);

    gForce = getGForce();

    sumOfForces[0] = 0; //reset forces to zero for next iteration
    sumOfForces[1] = 0;
    sumOfForces[2] = 0;
    
    sumOfMoments[0] = 0;
    sumOfMoments[1] = 0;
    sumOfMoments[2] = 0;

    engineState = {0,0,0};

    appliedVector = {0,0,0};

    //reset finVectors::: might remove 
    finVectors[0] =  {0,0,0};
    finVectors[1] =  {0,0,0};

    
}








std::array<std::array<float , 3> , 2> Vehicle::getFinForceVectors(){


    std::array<float,3> Xfin = {1,0,0};
    std::array<float,3> Yfin = {0,1,0};
    std::array<float,3> zrefrance = {0,0,1};

    Eigen::Vector3d v1( zrefrance[0],
                        zrefrance[1],
                        zrefrance[2]);


    Eigen::Vector3d v2( vehicleState[0],
                        vehicleState[1],
                        vehicleState[2]);
    
    Eigen::Vector3d rotation = getRotationAngles(v1, v2);

    //get vehicle rotation relative to a refrance vector then alter our preset force application vectors


    std::array<float,3> vehicleStateAngles = {static_cast<float>(rotation(0)) , static_cast<float>(rotation(1)) , static_cast<float>(rotation(2))};


    Matrix3x3 rotX = rotationMatrixX(vehicleStateAngles[0]);
    Matrix3x3 rotY = rotationMatrixY(vehicleStateAngles[1]);
    Matrix3x3 rotZ = rotationMatrixZ(vehicleStateAngles[2]);


    Matrix3x3 realign = rotX * rotY * rotZ;


    Xfin = realign.rotate(Xfin);
    Yfin = realign.rotate(Yfin);


    std::array<std::array<float , 3> , 2> finVectors = {Xfin , Yfin};

    return finVectors;

}


void Vehicle::applyFinForce(std::array<std::array<float,3>,2> forceVectors){

    if(forceVectors[0][0] != 0 && forceVectors[0][1] != 0 && forceVectors[0][2] != 0){
        addForce(forceVectors[0]);
        addMoment(forceToMoment(forceVectors[0], vehicleState , centerOfPressure));
    }          


    if(forceVectors[1][0] != 0 && forceVectors[1][1] != 0 && forceVectors[1][2] != 0){
        addForce(forceVectors[1]);
        addMoment(forceToMoment(forceVectors[1], vehicleState , centerOfPressure));
    }          

}



float Vehicle::getCurvature(){
    
    std::array<float ,3> velocity = {Xvelocity , Yvelocity , Zvelocity};
    std::array<float ,3> acceleration = {sumOfForces[0]/mass , sumOfForces[1]/mass , sumOfForces[2]/mass};
    std::array<float,3> veloCrossAccel = {0,0,0};

    //vectorCrossProduct(velocity , acceleration , veloCrossAccel);
    
    //return vectorMag(veloCrossAccel) / pow( vectorMag(velocity) , 3 );
    return 0;

    
}



bool Vehicle::fuelConsumption(float thrust){ 
    //returns true if vehicle has enough fuel to apply requested force. false should stop force application
    // numOfEngineOn can be fractional ex .8 is 80 percent thrust of 1 engine, or 3 is 3 engine at full power

    float numOfEngineOn = thrust / constants::maxThrust;

    float consumptionDuringTimeFuel  = fuelConsumptionRate * numOfEngineOn * constants::timeStep;
    float consumptionDuringTimeLOX  = LOXConsumptionRate * numOfEngineOn * constants::timeStep;

    if(fuel - consumptionDuringTimeFuel < 0 || LOX - consumptionDuringTimeLOX < 0) return false;

    fuel += -consumptionDuringTimeFuel;
    LOX += -consumptionDuringTimeLOX;

    mass = dryMass + fuel + LOX;

    return true;

}




void Vehicle::engineGimbal(float gimbalTargetX , float gimbalTargetY){

    setInBounds( gimbalTargetX , -constants::maxGimbalAngle , constants::maxGimbalAngle);
    setInBounds( gimbalTargetY , -constants::maxGimbalAngle , constants::maxGimbalAngle);
    
    gimbalErrorX = gimbalTargetX - gimbalX;
    gimbalErrorY = gimbalTargetY - gimbalY;

    float XInput = PID(gimbalTargetX , gimbalX , gimbalErrorX , sumOfGimbalErrorX , constants::timeStep , constants::gimbalPGain , constants::gimbalIGain , constants::gimbalDGain);

    float YInput = PID(gimbalTargetY , gimbalY , gimbalErrorY , sumOfGimbalErrorY , constants::timeStep , constants::gimbalPGain , constants::gimbalIGain , constants::gimbalDGain);


    setInBounds( XInput , -1.0f , 1.0f);
    setInBounds( YInput , -1.0f , 1.0f);


    logXInput = ((XInput * maxGimbalAcceleration) - constants::gimbalDamping * gimbalVelocityX * gimbalVelocityX);
    logYInput = ((XInput * maxGimbalAcceleration) - constants::gimbalDamping * gimbalVelocityX);
    
    if(gimbalVelocityX > 1) gimbalVelocityX += ((XInput * maxGimbalAcceleration) - constants::gimbalDamping * gimbalVelocityX * gimbalVelocityX) * constants::timeStep;
    else gimbalVelocityX += ((XInput * maxGimbalAcceleration) - constants::gimbalDamping * gimbalVelocityX) * constants::timeStep;

    if(gimbalVelocityY > 1) gimbalVelocityY += ((YInput * maxGimbalAcceleration) - constants::gimbalDamping * gimbalVelocityY * gimbalVelocityY) * constants::timeStep; 
    else  gimbalVelocityY += ((YInput * maxGimbalAcceleration) - constants::gimbalDamping * gimbalVelocityY) * constants::timeStep; 

    gimbalX += gimbalVelocityX * constants::timeStep;
    gimbalY += gimbalVelocityY * constants::timeStep;
    
}
