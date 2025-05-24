
#include <array>
#include <cmath>
#include <memory>
#include <cmath>
#include "../include/forceApplied.h"
#include "../include/vehicle.h"
#include "../include/vectorMath.h"
#include "../include/aero.h"
#include "../include/RungeKutta.h"
#include "../include/odeIterator.h"
#include "../include/rotationMatrix.h"
#include "../include/getRotation.h"
#include "../include/control.h"
#include "../include/sensors.h"
#include "../include/toml.h"

namespace SimCore{
void Vehicle::initSensors(){
    toml::tomlParse vParse;
    vParse.parseConfig( configFile,"vehicle");
    
    sensorMap = std::make_unique<std::unordered_map<std::string, std::unique_ptr<sensor>>>();
    
    toml::tomlParse accelParse;
    accelParse.parseConfig( configFile,"accelerometer");

    // Accelerometer
    std::unique_ptr<accelerometer> accel = std::make_unique<accelerometer>(
        accelParse.floatValues["frequency"],
        accelParse.floatValues["NoisePowerSpectralDensity"],
        accelParse.floatValues["bandwidth"],
        accelParse.floatValues["bias"]
    );
    if (accelParse.boolValues["burst"] == true) {
        accel->setBurst(
            accelParse.floatValues["burstStdDeviation"],
            accelParse.floatValues["maxBurstDuration"]
        );
    }
    sensorMap->insert({"accelerometer", std::move(accel)});

    // GNSS
    toml::tomlParse gpsParse;
    gpsParse.parseConfig( configFile,"gps");

    std::unique_ptr<GNSS> gps = std::make_unique<GNSS>(
        gpsParse.floatValues["frequency"],
        gpsParse.floatValues["NoisePowerSpectralDensity"],
        gpsParse.floatValues["bandwidth"],
        gpsParse.floatValues["bias"]
    );

    if (gpsParse.boolValues["burst"] == true) {
        gps->setBurst(
            gpsParse.floatValues["burstStdDeviation"],
            gpsParse.floatValues["maxBurstDuration"]
        );
    }
    sensorMap->insert({"GNSS", std::move(gps)});

    // Gyroscope
    toml::tomlParse gyroParse;
    gyroParse.parseConfig( configFile,"gyro");
    
    std::unique_ptr<gyroscope> gyro = std::make_unique<gyroscope>(
        gyroParse.floatValues["frequency"],
        gyroParse.floatValues["NoisePowerSpectralDensity"],
        gyroParse.floatValues["bandwidth"],
        gyroParse.floatValues["bias"]
    );
    if (gyroParse.boolValues["burst"] == true) {
        gyro->setBurst(
            gyroParse.floatValues["burstStdDeviation"],
            gyroParse.floatValues["maxBurstDuration"]
        );
    }
    sensorMap->insert({"gyro",std::move(gyro)});


}


Vehicle::Vehicle(): stateEstimation(){

}


void Vehicle::operator++(){
    ++iterations;
}


Vehicle& Vehicle::operator=(const Vehicle& other){
    if (this == &other) return *this;
    
    Xposition = other.Xposition;
    Yposition = other.Yposition;
    Zposition = other.Zposition;
    Xvelocity = other.Xvelocity;
    Yvelocity = other.Yvelocity;
    Zvelocity = other.Zvelocity;
    timeStep = other.timeStep;
    iterations = other.iterations;
    mass = other.mass;
    centerOfPressure = other.centerOfPressure;
    gForce = other.gForce;
    wind = other.wind;
    angularVelocity = other.angularVelocity;
    vehicleState = other.vehicleState;
    MOI = other.MOI;
    sumOfForces = other.sumOfForces;
    sumOfMoments = other.sumOfMoments;
    acceleration = other.acceleration;
    gravitationalAcceleration = other.gravitationalAcceleration;
    configFile = other.configFile;
    outputFile = other.outputFile;


    return *this;
}


void Vehicle::init(){
    outputFile = "../output.csv";
    toml::tomlParse vParse;
    vParse.parseConfig( configFile,"vehicle");

    iterations = 0;

    // Read initPosition
    auto& pos = vParse.arrayValues["initPosition"];
    Xposition = pos[0];
    Yposition = pos[1];
    Zposition = pos[2];


    gForce = 0;
    angularVelocity = {0, 0, 0};

    // Read initVehicleState
    auto& state = vParse.arrayValues["initVehicleState"];

    vehicleState[0] = state[0];
    vehicleState[1] = state[1];
    vehicleState[2] = state[2];


    // Read initVelocity
    auto& velo = vParse.arrayValues["initVelocity"];
    Xvelocity = velo[0];
    Yvelocity = velo[1];
    Zvelocity = velo[2];


    gravitationalAcceleration = vParse.floatValues["gravitationalAcceleration"]; 

    auto& windVect = vParse.arrayValues["wind"];
    wind[0] = windVect[0];
    wind[1] = windVect[1];
    wind[2] = windVect[2];

    sumOfForces[0] = 0;
    sumOfForces[1] = 0;
    sumOfForces[2] = 0;

    sumOfMoments[0] = 0;
    sumOfMoments[1] = 0;
    sumOfMoments[2] = 0;

    acceleration = {0,0,0};
    
    timeStep = .001; //seconds

}





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

    airVelocityVector[0] = Xvelocity +  wind[0];
    airVelocityVector[1] = Yvelocity +  wind[1];
    airVelocityVector[2] = Zvelocity +  wind[2];


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

    airVelocityVector[0] = Xvelocity +  wind[0];
    airVelocityVector[1] = Yvelocity +  wind[1];
    airVelocityVector[2] = Zvelocity +  wind[2];


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
            auto& ptr = sensorMap->find("GNSS")->second;
            GNSS* GPS = dynamic_cast<GNSS*> (ptr.get());
            GPS->setGNSSVelocity({Xvelocity,Yvelocity,Zvelocity});
        }
        updateSensors(this);
        updateEstimation( timeStep);
    
        sumOfForces[2] = sumOfForces[2] + gravitationalAcceleration * mass; 
        
        updateAcceleration();
    
        RungeKutta4th(sumOfForces[0] , mass ,  timeStep , Xvelocity,Xposition);
        RungeKutta4th(sumOfForces[1] , mass ,  timeStep , Yvelocity,Yposition);
        RungeKutta4th(sumOfForces[2] , mass ,  timeStep , Zvelocity,Zposition);
    
    
    
        Matrix3x3 rotX = rotationMatrixX(rotationalOde(sumOfMoments[0] , MOI[0],  timeStep ,angularVelocity[0]));
        Matrix3x3 rotY = rotationMatrixY(rotationalOde(sumOfMoments[1] , MOI[1],  timeStep ,angularVelocity[1]));
        Matrix3x3 rotZ = rotationMatrixZ(rotationalOde(sumOfMoments[2] , MOI[2],  timeStep ,angularVelocity[2]));
    
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

