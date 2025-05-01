#include "../include/sensors.h"
#include <random>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <iostream>
#include "../include/vehicle.h"
#include "vectorMath.h"

//using the PSD to form a normal distabution 
sensor::sensor(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias){
    hz = 1/frequency;
    sampleFrequency = frequency;
    mean = bias;
    float variance = NoisePowerSpectralDensity * bandwidth;
    standardDeviation = std::sqrt(variance);
}


// returned value = actualValue + noise + burst noise
float sensor::applyNoise(float realValue , float currentTime){
    std::normal_distribution<float> normalDistribution{mean,standardDeviation};
    float noise = normalDistribution(gen);

    float noisyValue = realValue + noise + burstNoise(currentTime);
    clamp(noisyValue);

    return noisyValue ;
}

void sensor::sample(Vehicle *vehicle){
    
}
std::array<float,3> sensor::read(){
    return {0,0,0};
}

void sensor::clamp(float &input){
    if(isClamped == false) return ;
    if (input < lowerBound) input = lowerBound;
    if (input > upperBound) input = upperBound;
}


void sensor::setClamp(float low , float high){
    
    if(low > high) throw std::runtime_error("Lowerbound cannot be greater than upperbound");

    isClamped = true;

    lowerBound = low;
    upperBound = high;
}


//enable burst. by default turned off
void sensor::setBurst(float busrtStdDeviation , float maxBurstDuration){
    if(standardDeviation < 0 ) return;
    if(maxBurstDuration > 0.0f )burst = true;
    else return;
    
    busrtStdDev =  busrtStdDeviation;
    maxBurstDur = maxBurstDuration;
    burstDuration = 0;
    currentBurstMagnitude = 0;

}

// burst noise | is low frequency noise that can not be mathamaticlly predicted.
// An implmentation will be introduced to give a non-realistic simulation of burst noise
float sensor::burstNoise(float currentTime){
    if(burst == false) return 0.0f;
    if(currentTime - lastBurst >= burstDuration){
        lastBurst = currentTime;
        std::uniform_real_distribution<float> dis(0.0f, 1.0f);
        burstDuration =  dis(gen) * maxBurstDur;
        std::normal_distribution<float> normalDistribution{0,busrtStdDev};
        currentBurstMagnitude = normalDistribution(gen);
    }
    return currentBurstMagnitude;
}


sensorSuite::sensorSuite(){
    sensorMap = std::make_shared<std::unordered_map<std::string, std::shared_ptr<sensor>>>();
}


void sensorSuite::updateSensors(Vehicle *vehicle){

    for(auto& sen:sensorList){

        sen->sample(vehicle);
    }

}

void sensorSuite::add(std::shared_ptr<sensor> sensor){
    sensorList.push_back(sensor);
}


GNSS::GNSS(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias):sensor(frequency , NoisePowerSpectralDensity , bandwidth , bias){
    lastPosition = {0,0,0};
    lastSample = 0;
}

void GNSS::sample(Vehicle *vehicle ) {
    float time =  vehicle->getTime();
    if(time - lastSample >= hz){
        gpsPosition[0] = applyNoise(vehicle->Xposition, time );
        gpsPosition[1] = applyNoise(vehicle->Yposition, time );
        gpsPosition[2] = applyNoise(vehicle->Zposition, time );
        if(lastSample > 0 ){
            for(int i = 0 ; i < 3 ; i++) velocity[i] = (gpsPosition[i] - lastPosition[i])/ (time-lastSample);
        }
        lastSample = time;
        for (int i = 0; i < 3; i++) lastPosition[i] = gpsPosition[i];
        vehicle->gpsUpdate();
    }
}

std::array<float,3> GNSS::read(){
    return gpsPosition;
}


std::array<float,3> GNSS::getVelocity(){
    return velocity;
}


gyroscope::gyroscope(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias):sensor(frequency , NoisePowerSpectralDensity , bandwidth , bias){
    
}

void gyroscope::sample(Vehicle *vehicle){
    float time = vehicle->getTime();
    if(time - lastSample >= hz){
        float time = vehicle->getTime();
        rotationVector[0] = applyNoise(vehicle->vehicleState[0] , time );
        rotationVector[1] = applyNoise(vehicle->vehicleState[1] , time);
        rotationVector[2] = applyNoise(vehicle->vehicleState[2] , time);
        rotationVector = normalizeVector(rotationVector);
        lastSample = time;
    }
}

std::array<float,3> gyroscope::read(){
    return {0,0,0};
}

accelerometer::accelerometer(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias):sensor(frequency , NoisePowerSpectralDensity , bandwidth , bias){

}

void accelerometer::sample(Vehicle *vehicle){
    float time = vehicle->getTime();
    if(time - lastSample >= hz){
        accel[0] = applyNoise(vehicle->acceleration[0] , time);
        accel[1] = applyNoise(vehicle->acceleration[1] , time);
        accel[2] = applyNoise(vehicle->acceleration[2] , time);
        lastSample = time;
    }
}

std::array<float,3> accelerometer::read(){
    return accel;
}

stateEstimation::stateEstimation(){
    stateEstimationSensors = std::make_shared<std::unordered_map<std::string, std::shared_ptr<sensor>>>();
    alpha = .03;
}

float stateEstimation::lowPassFilter(float newData,float prevData){
    return alpha * newData + (1 - alpha) * prevData;
}

//gpsUpdate is only called when gps is updated... further calls will reset imu effects on the state estimation
void stateEstimation::gpsUpdate(){
    std::shared_ptr<GNSS> GPS = std::dynamic_pointer_cast<GNSS> (sensorMap->find("GNSS")->second);
    position = GPS->read();
    for(int i = 0 ; i < 3 ; i ++) velocity[i] = lowPassFilter(GPS->getVelocity()[i] , velocity[i]);
}

//call at main clock speed
void stateEstimation::updateEstimation(float timeStep){
    std::shared_ptr<accelerometer> accel = std::dynamic_pointer_cast<accelerometer>(sensorMap->find("accelerometer")->second);
    auto accelReading = accel->read();
    for (int i = 0; i < 3; ++i) velocity[i] += accelReading[i] * timeStep;
    for (int i = 0; i < 3; ++i) position[i] += velocity[i] * timeStep;
}

//gps cordiante and add IMU data from zero
//use a filtering system to blend imu data and gps

std::array<float,3> stateEstimation::getEstimatedPosition(){
    return position;
}

std::array<float,3> stateEstimation::getEstimatedRotation(){
    return sensorMap->find("gyro")->second->read();
}

std::array<float,3> stateEstimation::getEstimatedVelocity(){
return velocity;
}


 