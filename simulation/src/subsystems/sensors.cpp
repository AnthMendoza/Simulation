#include "../../include/subsystems/sensors.h"
#include <random>
#include <cmath>
#include <stdexcept>
#include <memory>
#include "../../include/dynamics/vehicle.h"
#include <base/base.h>

namespace SimCore{
//using the PSD to form a normal distribution 
sensor::sensor(units::scalar frequency , units::scalar NoisePowerSpectralDensity , units::scalar bandwidth, units::scalar bias){
    hz = 1/frequency;
    sampleFrequency = frequency;
    mean = bias;
    units::scalar variance = NoisePowerSpectralDensity * bandwidth;
    standardDeviation = std::sqrt(variance);
}


// returned value = actualValue + noise + burst noise
units::scalar sensor::applyNoise(units::scalar realValue , units::scalar currentTime){
    std::normal_distribution<units::scalar> normalDistribution{mean,standardDeviation};
    units::scalar noise = normalDistribution(gen);

    units::scalar noisyValue = realValue + noise + burstNoise(currentTime);
    clamp(noisyValue);

    return noisyValue ;
}

void sensor::clamp(units::scalar &input){
    if(isClamped == false) return ;
    if (input < lowerBound) input = lowerBound;
    if (input > upperBound) input = upperBound;
}


void sensor::setClamp(units::scalar low , units::scalar high){
    
    if(low > high) throw std::runtime_error("Lowerbound cannot be greater than upperbound");

    isClamped = true;

    lowerBound = low;
    upperBound = high;
}


//enable burst. by default turned off
void sensor::setBurst(units::scalar burstStdDeviation , units::scalar maxBurstDuration){
    if(standardDeviation < 0 ) return;
    if(maxBurstDuration > 0.0f )burst = true;
    else return;
    
    burstStdDev = burstStdDeviation;
    maxBurstDur = maxBurstDuration;
    burstDuration = 0;
    currentBurstMagnitude = 0;

}

// burst noise | is low frequency noise that can not be mathematically predicted.
// An implementation will be introduced to give a non-realistic simulation of burst noise
units::scalar sensor::burstNoise(units::scalar currentTime){
    if(burst == false) return 0.0f;
    if(currentTime - lastBurst >= burstDuration){
        lastBurst = currentTime;
        std::uniform_real_distribution<units::scalar> dis(0.0f, 1.0f);
        burstDuration =  dis(gen) * maxBurstDur;
        std::normal_distribution<units::scalar> normalDistribution{0,burstStdDev};
        currentBurstMagnitude = normalDistribution(gen);
    }
    return currentBurstMagnitude;
}









GNSS::GNSS(units::scalar frequency,
        units::scalar NoisePowerSpectralDensity,
        units::scalar bandwidth,
        units::scalar bias,
        units::scalar velocityNoisePowerSpectralDensity,
        units::scalar velocityBandwidth,
        units::scalar velocityBias,
        avionics::sensor::gps_coordinate gps)
    :sensor(frequency , NoisePowerSpectralDensity , bandwidth , bias),
    gpsCordOrigin(gps)
    {
    lastPosition = {0,0,0};
    lastSample = 0;
    geographicCartesian = GeographicLib::LocalCartesian(gpsCordOrigin.latitude,gpsCordOrigin.longitude,gpsCordOrigin.altitude);

    velocitySensor = std::make_unique<GNSSVelocity>(
        frequency,
        velocityNoisePowerSpectralDensity,
        velocityBandwidth,
        velocityBias
    );
}

GNSS::GNSS(const GNSS& other)
    : sensor(other)
    , gpsPosition(other.gpsPosition)
    , lastPosition(other.lastPosition)
    , gpsCordOrigin(other.gpsCordOrigin)
    , gpsCordCurrent(other.gpsCordCurrent)
    , geographicCartesian(other.geographicCartesian)
    , velocitySensor(nullptr){
    auto clone = other.velocitySensor->clone();
    velocitySensor = std::unique_ptr<GNSSVelocity>(static_cast<GNSSVelocity*>(clone.release()));


}

void GNSS::sample(Vehicle *vehicle ) {
    units::scalar time =  vehicle->getTime();
    if(time - lastSample >= hz){
        auto pos = vehicle->getPositionVector();
        gpsPosition[0] = applyNoise(pos[0], time );
        gpsPosition[1] = applyNoise(pos[1], time );
        gpsPosition[2] = applyNoise(pos[2], time );
        
        for (int i = 0; i < 3; i++) lastPosition[i] = gpsPosition[i];
        gpsCordCurrent = gpsCordOrigin;
        geographicCartesian.Reverse(gpsPosition[0],gpsPosition[1],gpsPosition[2],
            gpsCordCurrent.latitude,gpsCordCurrent.longitude,gpsCordCurrent.altitude);


        velocitySensor->sample(vehicle);

        lastSample = time;
    }
}

units::vec3 GNSS::read(){
    return gpsPosition;
}

avionics::sensor::gps_coordinate GNSS::readGNSS(){
    return gpsCordCurrent;
}

units::vec3 GNSS::readGNSSVelocity(){
    return velocitySensor->read();
}


GNSS::GNSSVelocity::GNSSVelocity(units::scalar frequency , units::scalar NoisePowerSpectralDensity , units::scalar bandwidth, units::scalar bias)
:sensor(frequency , NoisePowerSpectralDensity , bandwidth , bias){

}

void GNSS::GNSSVelocity::sample(Vehicle* vehicle){
    units::scalar time = vehicle->getTime();
    if(time - lastSample >= hz){
        auto velo = vehicle->getVelocityVector();
        gpsVelocity[0] = applyNoise(velo[0], time );
        gpsVelocity[1] = applyNoise(velo[1], time );
        gpsVelocity[2] = applyNoise(velo[2], time );
        lastSample = time;
    }
}

units::vec3 GNSS::GNSSVelocity::read(){
    return gpsVelocity; 
}


gyroscope::gyroscope(units::scalar frequency , units::scalar NoisePowerSpectralDensity , units::scalar bandwidth, units::scalar bias):sensor(frequency , NoisePowerSpectralDensity , bandwidth , bias){
    
}

void gyroscope::sample(Vehicle *vehicle){
    units::scalar time = vehicle->getTime();
    auto deltaTime = time - lastSample;
    if(deltaTime >= hz){
        poseState pose = vehicle->getPose();
        poseAngleDifference angleDifference(lastPose , pose);

        auto rates = angleDifference.getRotationRate(deltaTime);
        rates.pitchRate = applyNoise(rates.pitchRate,time);
        rates.rollRate = applyNoise(rates.rollRate,time);
        rates.yawRate = applyNoise(rates.yawRate,time);

        lastPose = pose;
        rotationVector = {rates.rollRate,rates.pitchRate,rates.yawRate};
        lastSample = time;
    }
}

units::vec3 gyroscope::read(){
    return {0,0,0};
}

accelerometer::accelerometer(units::scalar frequency , units::scalar NoisePowerSpectralDensity , units::scalar bandwidth, units::scalar bias):sensor(frequency , NoisePowerSpectralDensity , bandwidth , bias){

}

void accelerometer::sample(Vehicle *vehicle){
    units::scalar time = vehicle->getTime();
    if(time - lastSample >= hz){
        units::vec3 acceleration;
        acceleration = vehicle->getAcceleration();

        //removing gravitational acceleration from the z axis.
        //the simulation includes it as an acting acceleration to move the vehicle.
        //the sensor should read 0 when in free fall.

        acceleration[0] = applyNoise(acceleration[0] , time);
        acceleration[1] = applyNoise(acceleration[1] , time);
        acceleration[2] = applyNoise(acceleration[2] , time) - vehicle->getGravitationalAcceleration();
        
        accel = acceleration;
        lastSample = time;
    }
}

units::vec3 accelerometer::read(){
    return accel;
}


sensor::sensor(const sensor& other)
    : mean(other.mean),
      standardDeviation(other.standardDeviation),
      lowerBound(other.lowerBound),
      upperBound(other.upperBound),
      currentBustValue(other.currentBustValue),
      isClamped(other.isClamped),
      burst(other.burst),
      burstStdDev(other.burstStdDev),
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


//gps coordinate and add IMU data from zero
//use a filtering system to blend imu data and gps

//void radar::sample(Vehicle *vehicle){
    //sensorOrigin = vehicle->getRadarPosition();
    
    //vehicleVector = vehicle->vehicleState;

}
 
//}