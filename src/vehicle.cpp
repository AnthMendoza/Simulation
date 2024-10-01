#include <iostream>
#include <array>
#include "../include/vehicle.h"
#include "../include/vectorMath.h"
#include "../include/aero.h"
#include "../include/constants.h"
#include "../include/RungeKutta.h"
#include "../include/odeIterator.h"






Vehicle::Vehicle(float x, float y, float z,std::array<float, 3> mMOI ,float mMass)
    : Xposition(x), Yposition(y), Zposition(z), mass(mMass){
    std::copy(mMOI.begin(), mMOI.end(), MOI.begin());

    // roll pitch yaw (x,y,z) defines the direction vector, heading. ex. (0,0,1) is rocket pointing straight up.

    //(x,y,z)positon defines its location reltaive to an orgin

    angularVelocity = {0,0,0};

    centerOfPressure = 6; //meters

    vehicleState[0] = constants::initVehicleState[0];
    vehicleState[1] = constants::initVehicleState[1];  //setting init value for vehicle state, logged in constants.h
    vehicleState[2] = constants::initVehicleState[2];

    Xvelocity = 0;
    Yvelocity = 0;   // Velocity vector , direction of movment relative to the ground
    Zvelocity = 0;
    

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

void Vehicle::drag(){
    
    std::array<float,3> airVelocityVector;

    airVelocityVector[0] = Xvelocity + constants::wind[0];
    airVelocityVector[1] = Yvelocity + constants::wind[1];
    airVelocityVector[2] = Zvelocity + constants::wind[2];


    float absVelocity = vectorMag(airVelocityVector);
    float dragAngle = vectorAngleBetween(airVelocityVector , vehicleState);

    //std::cout  << velocityVector[0]<< " " << velocityVector[1]<<  " " << velocityVector[2]<<  std::endl;
    //std::cout  << "Vehicle"<< vehicleVector[0]<< " " << vehicleVector[1]<<  " " << vehicleVector[2]<<  std::endl;

    float drag = -.5 * (absVelocity * absVelocity) * aeroArea(dragAngle) * coefOfDrag(dragAngle) * airDensity(Zposition); //calculating abs drag 
    
    std::array<float,3> dragVector;
    std::array<float,3> normalVelocityVector = normalizeVector(airVelocityVector);
    
    dragVector[0] = drag * normalVelocityVector[0];
    dragVector[1] = drag * normalVelocityVector[1];
    dragVector[2] = drag * normalVelocityVector[2];

    addForce(dragVector);
}

void Vehicle::lift(){
    
    std::array<float,3> airVelocityVector;

    airVelocityVector[0] = Xvelocity + constants::wind[0];
    airVelocityVector[1] = Yvelocity + constants::wind[1];
    airVelocityVector[2] = Zvelocity + constants::wind[2];


    float absVelocity = vectorMag(airVelocityVector);
    float dragAngle = vectorAngleBetween(airVelocityVector , vehicleState);

    //std::cout  << velocityVector[0]<< " " << velocityVector[1]<<  " " << velocityVector[2]<<  std::endl;
    //std::cout  << "Vehicle"<< vehicleVector[0]<< " " << vehicleVector[1]<<  " " << vehicleVector[2]<<  std::endl;

    float drag = -.5 * (absVelocity * absVelocity) * aeroArea(dragAngle) * coefOfDrag(dragAngle) * airDensity(Zposition); //calculating abs drag 
    
    std::array<float,3> dragVector;
    std::array<float,3> normalVelocityVector = normalizeVector(airVelocityVector);
    
    dragVector[0] = drag * normalVelocityVector[0];
    dragVector[1] = drag * normalVelocityVector[1];
    dragVector[2] = drag * normalVelocityVector[2];

    addForce(dragVector);
    
}   


void  Vehicle::addForce(std::array<float,3> forceVector){
    sumOfForces[0] += forceVector[0];
    sumOfForces[1] += forceVector[1];
    sumOfForces[2] += forceVector[2];
}

void Vehicle::updateState(){    
    //adding gravity to the force of Z, becuase this is an acceleration and not a force; The addForce function cannot handle it
    sumOfForces[2] = sumOfForces[2] + constants::gravitationalAcceleration * mass; 
    

    RungeKutta4th(sumOfForces[0] , mass , constants::timeStep , Xvelocity,Xposition);
    RungeKutta4th(sumOfForces[1] , mass , constants::timeStep , Yvelocity,Yposition);
    RungeKutta4th(sumOfForces[2] , mass , constants::timeStep , Zvelocity,Zposition);


    rotationalOde(sumOfMoments[0] , MOI[0], constants::timeStep ,angularVelocity[0]);
    rotationalOde(sumOfMoments[1] , MOI[1], constants::timeStep ,angularVelocity[0]);
    rotationalOde(sumOfMoments[2] , MOI[2], constants::timeStep , angularVelocity[0]);

    sumOfForces[0] = 0; //reset forces to zero for next iteration
    sumOfForces[1] = 0;
    sumOfForces[2] = 0;
    
    sumOfMoments[0] = 0;
    sumOfMoments[1] = 0;
    sumOfMoments[2] = 0;

    
}