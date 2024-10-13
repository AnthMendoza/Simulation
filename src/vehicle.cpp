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









Vehicle::Vehicle(){
    // roll pitch yaw (x,y,z) defines the direction vector, heading. ex. (0,0,1) is rocket pointing straight up.

    //(x,y,z)positon defines its location reltaive to an orgin
    mass = constants::mass;
    MOI = constants::MOI;
    Xposition = constants::initPosition[0];
    Yposition = constants::initPosition[1];
    Zposition = constants::initPosition[2];
    reentry = false;
    glidePhase = false; //false means yet to happen, true means already happened
    iterations = 0;

    engineState = {0,0,0};
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
    

    sumOfForces[0] = 0;
    sumOfForces[1] = 0;
    sumOfForces[2] = 0;

    sumOfMoments[0] = 0;
    sumOfMoments[1] = 0;
    sumOfMoments[2] = 0;
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

    addForce(liftVector);

    addMoment(forceToMoment(liftVector, vehicleState , centerOfPressure));

    
}   


void Vehicle::applyEngineForce(std::array<float,2> twoDEngineRadians , float thrust){

    


    Matrix3x3 rotX = rotationMatrixX(twoDEngineRadians[0]);
    Matrix3x3 rotY = rotationMatrixY(twoDEngineRadians[1]);

    std::array<float,3> engineVector = {0,0,1};

    Eigen::Vector3d v1(engineVector[0], engineVector[1], engineVector[2]);

    Matrix3x3 combined = rotX * rotY;

    engineVector = combined.rotate(engineVector);


    std::array<float , 3> vehicleStateAntiparallel = vehicleState;
    // reverses the vector so that we can apply a thrust in the oposite direction

    for(int i = 0; i<3 ; i++){
        vehicleStateAntiparallel[i] = -vehicleStateAntiparallel[i]; 
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

    addMoment(forceToMoment(engineVector, vehicleState , cogToEngine));

    engineState = engineVector;

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

    
}








void Vehicle::finForce(){


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


    std::array<float,3> vehicleStateAngles = {static_cast<float>(rotation(0)) , static_cast<float>(rotation(1)) , static_cast<float>(rotation(2))};


    Matrix3x3 rotX = rotationMatrixX(vehicleStateAngles[0]);
    Matrix3x3 rotY = rotationMatrixY(vehicleStateAngles[1]);
    Matrix3x3 rotZ = rotationMatrixZ(vehicleStateAngles[2]);



    


}