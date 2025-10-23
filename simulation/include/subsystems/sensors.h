#ifndef SENSORS_H 
#define SENSORS_H


#pragma once

#include <random>
#include <optional>
#include "../core/vectorMath.h"
#include "../core/quaternion.h"

namespace SimCore{
class Vehicle;
class sensor{
    private:
    float mean;
    float standardDeviation;
    float lowerBound;
    float upperBound;
    float currentBustValue;
    bool isClamped = false;
    bool burst = false;
    float busrtStdDev;
    float maxBurstDur;
    float burstDuration;
    float lastBurst;
    float currentBurstMagnitude = 0;
    protected:
    float sampleFrequency;
    //Time of last sample
    float lastSample;
    //Random are not copyable 
    //thus seed and gen{seed()} delete the constructor 
    std::random_device seed; 
    std::mt19937 gen{seed()};

    float noise();
    float linearInterpolation();
    float burstNoise(float currentTime);
    void clamp(float &input);
    public:
    float hz;
    sensor(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias);
    sensor(const sensor& other);
    virtual std::unique_ptr<sensor> clone() const = 0;
    virtual ~sensor() =  default;
    float applyNoise(float realValue , float currentTime);
    void setClamp(float low , float high);
    void setBurst(float busrtStdDeviation , float maxBurstDuration);
    virtual void sample(Vehicle *vehicle) = 0;
    virtual std::array<float,3> read() = 0; 

    inline const float getTimeOfSample() const {
        return lastSample;
    }

};

template<typename sensorPacketType>
class sensorSuite{
    private:
    public:
    std::unique_ptr<std::unordered_map<std::string, std::shared_ptr<sensor>>> sensorMap;

    sensorSuite(){
    sensorMap = std::make_unique<std::unordered_map<std::string, std::shared_ptr<sensor>>>();
    }

    sensorSuite(const sensorSuite& other) {
        sensorMap = std::make_unique<std::unordered_map<std::string, std::shared_ptr<sensor>>>();
        for (const auto& [key, sensorPtr] : *other.sensorMap) {
            if (sensorPtr) {
                (*sensorMap)[key] = sensorPtr->clone(); 
            }
        }   
    }
    virtual ~sensorSuite() = default;
    //Each sensor base class has a limiter. Only allowing for samples if the frequency allows.
    //Call at simulation frequency or at the frequency of the fastest sensor.
    //cycles through *sensorMap and calls on sample method of each sensor for potental update of data sample.
    virtual void updateSensors(Vehicle *vehicle){
        //keyPtrSensors is a std::pair first is key second is unique_ptr.
        for (const auto& keyPtrSensors : *sensorMap){
            keyPtrSensors.second->sample(vehicle);
        }
    }

    virtual sensorPacketType getSensorData() = 0;
};


class GNSS : public sensor{
    private:
    std::array<float,3> gpsPosition;
    std::array<float,3> velocity;
    std::array<float,3> lastPosition;
    public:
    GNSS(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias);
    std::unique_ptr<sensor> clone() const override {
        return std::make_unique<GNSS>(*this);
    }

    void sample(Vehicle *vehicle) override; 
    std::array<float,3>  read() override;
    inline std::array<float,3> getVelocity(){
        return velocity;
    }
    inline void setGNSSVelocity(std::array<float,3> velo){
        velocity = velo;
    }
};

class accelerometer : public sensor{
    private:
    std::array<float,3> accel;
    public:
    accelerometer(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias);
    std::unique_ptr<sensor> clone() const override {
        return std::make_unique<accelerometer>(*this);
    }
    void sample(Vehicle *vehicle) override;
    std::array<float,3>  read() override;

};


class gyroscope : public sensor{
    private:
    //rotation around x y z axis realitve to the drone vehicle. 
    std::array<float,3> rotationVector;
    poseState lastPose;
    public:
    gyroscope(float frequency , float NoisePowerSpectralDensity , float bandwidth, float bias);
    std::unique_ptr<sensor> clone() const override {
        return std::make_unique<gyroscope>(*this);
    }
    void sample(Vehicle *vehicle) override;
    std::array<float,3>  read() override;

};


//class radar : public sensor{
//    private:
//    std::array<float,3> sensorOrigin;
//    std::vector<std::vector<std::array<float,3>>> points;
//    std::array<float,3> vehicleVector;
//    float quantization = .05;
//    const int rows = 16;           // vertical resolution
//    const int cols = 64;           // horizontal resolution
//    const float verticalFOV = 20;  // degrees
//    const float horizontalFOV = 90; // degrees
//    const float maxRange = 10.0f;   // meters
//    protected:
//
//    public:
//    void sample(Vehicle *Vehicle) override;
//        
//};


}
#endif
