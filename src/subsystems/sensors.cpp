#include "../../include/subsystems/sensors.h"
#include <random>
#include <cmath>
#include <stdexcept>
#include <memory>
#include "../../include/dynamics/vehicle.h"
#include "../../include/core/vectorMath.h"

namespace SimCore{
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
    sensorMap = std::make_unique<std::unordered_map<std::string, std::unique_ptr<sensor>>>();
}

//cycles through *sensorMap and calls on sample method of each sensor for potental update of data sample.
void sensorSuite::updateSensors(Vehicle *vehicle){
    //keyPtrSensors is a std::pair first is key second is unique_ptr.
    for (const auto& keyPtrSensors : *sensorMap){
        keyPtrSensors.second->sample(vehicle);
    }
}





GNSS::GNSS(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias):sensor(frequency , NoisePowerSpectralDensity , bandwidth , bias){
    lastPosition = {0,0,0};
    lastSample = 0;
}

void GNSS::sample(Vehicle *vehicle ) {
    float time =  vehicle->getTime();
    if(time - lastSample >= hz){
        auto pos = vehicle->getPositionVector();
        gpsPosition[0] = applyNoise(pos[0], time );
        gpsPosition[1] = applyNoise(pos[1], time );
        gpsPosition[2] = applyNoise(pos[2], time );
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


gyroscope::gyroscope(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias):sensor(frequency , NoisePowerSpectralDensity , bandwidth , bias){
    
}

void gyroscope::sample(Vehicle *vehicle){
    float time = vehicle->getTime();
    if(time - lastSample >= hz){
        float time = vehicle->getTime();
        auto state = vehicle->getState();
        rotationVector[0] = applyNoise(state[0] , time );
        rotationVector[1] = applyNoise(state[1] , time);
        rotationVector[2] = applyNoise(state[2] , time);
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
        std::array<float,3> accel;
        vehicle->getAccel(accel);
        accel[0] = applyNoise(accel[0] , time);
        accel[1] = applyNoise(accel[1] , time);
        accel[2] = applyNoise(accel[2] , time);
        lastSample = time;
    }
}

std::array<float,3> accelerometer::read(){
    return accel;
}

stateEstimation::stateEstimation(){
    alpha = .03;
}

SimCore::stateEstimation::stateEstimation(const stateEstimation& other)
    : sensorSuite(other),  
      position(other.position),
      velocity(other.velocity),
      rotation(other.rotation),
      alpha(other.alpha),
      firstGPSSample(other.firstGPSSample)
{
    
}

//when used add statement that sets newData = prevData when no prevData exists(first sample)
float stateEstimation::lowPassFilter(float newData,float prevData){

    return alpha * newData + (1 - alpha) * prevData;
}

//gpsUpdate is only called when gps is updated... further calls will reset imu effects on the state estimation
void stateEstimation::gpsUpdate(){
    auto& ptr = sensorMap->find("GNSS")->second;
    GNSS* GPS = dynamic_cast<GNSS*>(ptr.get());
    position = GPS->read();
    if(firstGPSSample == true){
        for(int i = 0 ; i < 3 ; i ++){
            float velo =  GPS->getVelocity()[i];
            velocity[i] = lowPassFilter(velo , velo);
        }
        firstGPSSample = false;
        return;
    }
    for(int i = 0 ; i < 3 ; i ++) velocity[i] = lowPassFilter(GPS->getVelocity()[i] , velocity[i]);
}

//call at main clock speed
void stateEstimation::updateEstimation(float timeStep){
    auto& ptr = sensorMap->find("accelerometer")->second;
    accelerometer* accel = dynamic_cast<accelerometer*> (ptr.get());
    auto accelReading = accel->read();
    for (int i = 0; i < 3; ++i) velocity[i] += accelReading[i] * timeStep;
    for (int i = 0; i < 3; ++i) position[i] += velocity[i] * timeStep;
}

sensorSuite::sensorSuite(const sensorSuite& other) {
    sensorMap = std::make_unique<std::unordered_map<std::string, std::unique_ptr<sensor>>>();
    for (const auto& [key, sensorPtr] : *other.sensorMap) {
        if (sensorPtr) {
            (*sensorMap)[key] = sensorPtr->clone(); 
        }
    }
}


sensor::sensor(const sensor& other)
    : mean(other.mean),
      standardDeviation(other.standardDeviation),
      lowerBound(other.lowerBound),
      upperBound(other.upperBound),
      currentBustValue(other.currentBustValue),
      isClamped(other.isClamped),
      burst(other.burst),
      busrtStdDev(other.busrtStdDev),
      maxBurstDur(other.maxBurstDur),
      burstDuration(other.burstDuration),
      lastBurst(other.lastBurst),
      currentBurstMagnitude(other.currentBurstMagnitude),
      sampleFrequency(other.sampleFrequency),
      lastSample(other.lastSample),
      hz(other.hz),
      gen(std::random_device{}())
{
}



//gps cordiante and add IMU data from zero
//use a filtering system to blend imu data and gps

//void radar::sample(Vehicle *vehicle){
    //sensorOrigin = vehicle->getRadarPosition();
    
    //vehicleVector = vehicle->vehicleState;

}
 
//}