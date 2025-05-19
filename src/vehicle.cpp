#include <iostream>
#include <array>
#include <cmath>
#include <memory>
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
#include "../include/sensors.h"
#include "../include/toml.hpp"

namespace SimCore{
void Vehicle::initSensors(){
    auto config = toml::parse(constants::configFile, toml::spec::v(1,1,0));
    auto rocket = toml::find(config,"vehicle");
    auto map = std::make_shared<std::unordered_map<std::string,std::shared_ptr<sensor>>>();
    sensorMap = map;
    auto stateMap = std::make_shared<std::unordered_map<std::string,std::shared_ptr<sensor>>>();
    stateEstimationSensors = stateMap;
    auto list = std::vector<std::shared_ptr<sensor>>();
    sensorList = list;
    
    auto sensorConfig = toml::find(config , "sensors");
    //NOTE:: this is not templeted for diffrences sensor to sensor
    auto accelConfig= toml::find(sensorConfig , "accelerometer");
    std::shared_ptr<accelerometer> accel = std::make_shared<accelerometer>( toml::find<float>(accelConfig , "frequency"),
                                                                            toml::find<float>(accelConfig , "NoisePowerSpectralDensity"),
                                                                            toml::find<float>(accelConfig , "bandwidth"),
                                                                            toml::find<float>(accelConfig , "bias"));
    
    if(toml::find<bool>(accelConfig, "burst")){
        accel->setBurst(toml::find<float>(accelConfig, "burstStdDeviation") , toml::find<float>(accelConfig, "maxBurstDuration"));
    }
    sensorMap->insert({"accelerometer",accel});
    this->add(accel);

    auto gpsConfig= toml::find(sensorConfig , "gps");
    
    std::shared_ptr<GNSS> gps = std::make_shared<GNSS>(   toml::find<float>(gpsConfig , "frequency"),
                                                                            toml::find<float>(gpsConfig , "NoisePowerSpectralDensity"),
                                                                            toml::find<float>(gpsConfig , "bandwidth"),
                                                                            toml::find<float>(gpsConfig , "bias"));

    if(toml::find<bool>(gpsConfig, "burst")){
        gps->setBurst(toml::find<float>(gpsConfig, "burstStdDeviation") , toml::find<float>(gpsConfig, "maxBurstDuration"));
    }
    sensorMap->insert({"GNSS",gps});
    this->add(gps);

    auto gyroConfig= toml::find(sensorConfig , "gyro");

    std::shared_ptr<gyroscope> gyro = std::make_shared<gyroscope>( toml::find<float>(gyroConfig , "frequency"),
                                                                            toml::find<float>(gyroConfig , "NoisePowerSpectralDensity"),
                                                                            toml::find<float>(gyroConfig , "bandwidth"),
                                                                            toml::find<float>(gyroConfig , "bias"));

    if(toml::find<bool>(gyroConfig, "burst")){
        gyro->setBurst(toml::find<float>(gyroConfig, "burstStdDeviation") , toml::find<float>(gyroConfig, "maxBurstDuration"));
    }
    sensorMap->insert({"gyro",gyro});
    this->add(gyro);


    this->stateEstimationSensors = std::shared_ptr<std::unordered_map<std::string,std::shared_ptr<sensor>>>(this->sensorMap);

}


Vehicle::Vehicle(): stateEstimation(){

}


void Vehicle::operator++(int){
    iterations++;
}


void Vehicle::init(){
    auto config = toml::parse(constants::configFile, toml::spec::v(1,1,0));
    auto rocket = toml::find(config,"vehicle");

    iterations = 0;

    std::vector<float> initPosition = toml::find<std::vector<float>>(rocket ,"initPosition");

    Xposition = initPosition[0];
    Yposition = initPosition[1];
    Zposition = initPosition[2];

    gForce = 0;

    angularVelocity = {0,0,0};

    std::vector<float> initVehicleState = toml::find<std::vector<float>>(rocket ,"initVehicleState");

    vehicleState[0] = initVehicleState[0];
    vehicleState[1] = initVehicleState[1];  //setting init value for vehicle state, logged in constants.h
    vehicleState[2] = initVehicleState[2];

    std::vector<float> initVelocity = toml::find<std::vector<float>>(rocket ,"initVelocity");

    Xvelocity = initVelocity[0];
    Yvelocity = initVelocity[1];   // Velocity vector , direction of movment relative to the ground
    Zvelocity = initVelocity[2];


    sumOfForces[0] = 0;
    sumOfForces[1] = 0;
    sumOfForces[2] = 0;

    sumOfMoments[0] = 0;
    sumOfMoments[1] = 0;
    sumOfMoments[2] = 0;

    acceleration = {0,0,0};


}


Vehicle::Vehicle(const Vehicle& vehicle) = default;



float Vehicle::getVelocity(){
    std::array<float ,3> vector = {Xvelocity , Yvelocity , Zvelocity};
    return vectorMag( vector );
}



float Vehicle::getGForce(){
    
    std::array<float ,3> vector = {sumOfForces[0]/mass,sumOfForces[1]/mass,sumOfForces[2]/mass};

    return vectorMag(vector)/9.8;
}

void Vehicle::getAccel(std::array<float,3> &accel){
    for(int i = 0 ; i < 3 ; i++) accel[i] = sumOfForces[i] / mass;
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

    if(directionality(normalAirVelocityVector , vehicleState) == false){
        liftVector[0] = -liftVector[0];
        liftVector[1] = -liftVector[1];
        liftVector[2] = -liftVector[2];
    }

    addForce(liftVector);

    addMoment(forceToMoment(liftVector, vehicleState , centerOfPressure));

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
        if(iterations == 0){
            std::shared_ptr GPS = std::static_pointer_cast<GNSS>(sensorMap->find("GNSS")->second);
            GPS->setGNSSVelocity({Xvelocity,Yvelocity,Zvelocity});
        }
        updateSensors(this);
        updateEstimation(constants::timeStep);
    
        sumOfForces[2] = sumOfForces[2] + constants::gravitationalAcceleration * mass; 
        
        updateAcceleration();
    
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
    
        getAccel(acceleration);
    
        sumOfForces[0] = 0; //reset forces to zero for next iteration
        sumOfForces[1] = 0;
        sumOfForces[2] = 0;
        
        sumOfMoments[0] = 0;
        sumOfMoments[1] = 0;
        sumOfMoments[2] = 0;
}




float Vehicle::PID(float target , float currentState , float &previousError , float &sumOfError , float timeStep, float Pgain , float Igain , float Dgain){

    float error = target - currentState;

    sumOfError = error * timeStep;

    float slope = (error - previousError) / timeStep;

    previousError = error;

    return Pgain * error + Igain * sumOfError + Dgain * slope;


}




}

